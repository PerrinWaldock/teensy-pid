from abc import ABC, abstractmethod
import sys
import os
import numpy as np
from skopt import gp_minimize
from scipy import fft
from scipy.optimize import curve_fit, minimize
from scipy.interpolate import interp1d
import math
import matplotlib.pyplot as plt
from collections import deque

VALUE_LIMIT = sys.float_info.max/1e200

kKeys = ["kp", "ki", "kd"]

class Tunable(ABC):
    @abstractmethod
    def getOutputs(self, inputs: list[float], abortCondition: callable=None) -> tuple[list[float], list[float]]: pass
    
    @abstractmethod
    def reset(self) -> None: pass
    
    @property
    @abstractmethod
    def T(self) -> float: pass
    @T.setter
    @abstractmethod
    def T(self, value: float) -> None: pass
    
    @property
    @abstractmethod
    def kp(self) -> float: pass
    @kp.setter
    @abstractmethod
    def kp(self, value: float) -> None: pass
    
    @property
    @abstractmethod
    def ki(self) -> float: pass
    @ki.setter
    @abstractmethod
    def ki(self, value: float) -> None: pass
    
    @property
    @abstractmethod
    def kd(self) -> float: pass
    @kd.setter
    @abstractmethod
    def kd(self, value: float) -> None: pass
    
    @property
    @abstractmethod
    def setpoint(self) -> float: pass
    @setpoint.setter
    @abstractmethod
    def setpoint(self, value: float) -> None: pass

ZIEGLER_NICHOLS_FUNCTIONS = {
    "p": {
        "kp": lambda k,t: 0.5*k,
        "ki": lambda k,t: 0,
        "kd": lambda k,t: 0,
        },
    "pi": {
        "kp": lambda k,t: 0.45*k,
        "ki": lambda k,t: 0.54*k/t,
        "kd": lambda k,t: 0,
        },
    "pd": {
        "kp": lambda k,t: 0.8*k,
        "ki": lambda k,t: 0,
        "kd": lambda k,t: 0.1*k*t,
        },
    "pid": {
        "kp": lambda k,t: 0.6*k,
        "ki": lambda k,t: 1.2*k/t,
        "kd": lambda k,t: 0.075*k*t,
        },
    "pessen": {
        "kp": lambda k,t: 0.7*k,
        "ki": lambda k,t: 1.75*k/t,
        "kd": lambda k,t: 1.05*k*t,
        },
    "overshoot": {
        "kp": lambda k,t: (1/3)*k,
        "ki": lambda k,t: (2/3)*k/t,
        "kd": lambda k,t: (1/9)*k*t,
        },
    "no-overshoot": {
        "kp": lambda k,t: 0.2*k,
        "ki": lambda k,t: 0.4*k/t,
        "kd": lambda k,t: (2/30)*k*t,
        },
}

# TODO badly needs testing
def ziegler_nichols(controller: Tunable, controlType: str, setPoint: float=1, n=10000, kpMax=100):
    assert controlType in ZIEGLER_NICHOLS_FUNCTIONS
    
    def sinusoid(t, A, f, phase, offset=0, gamma=0):
        return A*np.exp(gamma*t)*np.sin(2*np.pi*f*t + phase) + offset
    
    def sineFit(ts, signal, p0=None):
        maxF = 1/np.mean(np.diff(ts))
        minF = 1/(np.max(ts) - np.min(ts))
        maxA = np.max(signal) - np.min(signal)
        
        bounds = ((
                0, 
                minF,
                0,
                min(signal),
                -maxF),
                (maxA,
                maxF,
                2*np.pi,
                max(signal),
                maxF))
        p0 = [min(max(x, lower), upper) for x, (lower, upper) in zip(p0, np.transpose(bounds))]
        popt, pcov = curve_fit(
            sinusoid, 
            ts, 
            signal,
            p0=p0,
            bounds=bounds)
        return popt, np.sqrt(np.diag(pcov))
    
    def fitScore(ts, signal, fitfn):
        # score should be between 0 and 1, higher is better
        newTs = ts
        #addedTs = np.diff(ts) + ts[:-1]
        #newTs = np.array(sorted(list(ts) + list(addedTs))) #TODO interleave more efficiently
        newSignal = interp1d(ts, signal)(newTs)
        fittedSignal = fitfn(newTs)
        maxValue = max(max(newSignal), max(fittedSignal)) - min(min(newSignal), min(fittedSignal))
        deviationSum = np.sum(np.abs((newSignal - fittedSignal)/maxValue))
        score = 1 - (deviationSum/len(newSignal))
        return score
    
    def sinusoidScore(signal):
        # returns score and period. Certainty score is between 0 (low) and 1 (high)
        if any(not np.isfinite(x) for x in signal):
            return -VALUE_LIMIT, 0
        zerodSignal = signal - np.mean(signal)
        spectrum = np.abs(np.fft.rfft(zerodSignal))/len(signal)
        freqs = np.fft.rfftfreq(len(signal), controller.T)
        maxInd = np.argmax(spectrum)
        peakF = freqs[maxInd]
        amplitude = spectrum[maxInd]
        
        ts = np.arange(len(signal)) * controller.T
        p0 = (amplitude, peakF, np.pi, np.mean(signal), 0)
        try:
            ps, (dA, df, dPhase, dOffset, dGamma) = sineFit(ts, signal, p0=p0)
        except RuntimeError as e:
            print(e)
            ps = list(p0)
            ps[4] = max(freqs)
            ps = tuple(ps)
        (A, f, phase, offset, gamma) = ps
        scoreOfFit = fitScore(ts, signal, lambda t: sinusoid(t, *ps))
        score = scoreOfFit/(np.abs(gamma) + 1e-4)
        # plt.plot(signal)
        # plt.plot(sinusoid(ts, *ps))
        # plt.show()
        return score, 1/f
    
    def getOscillationScore(kp):
        print(f"kp={kp}")
        controller.kp = kp
        controller.reset()
        inputs = [0] + [setPoint]*n
        try:
            feedbacks, _ = controller.getOutputs(inputs, abortCondition=lambda i, o, f: abs(f) > 100*abs(setPoint))
            score, t = sinusoidScore(feedbacks)
            if len(feedbacks) != len(inputs) - 1:
                score *= len(feedbacks)/len(inputs)
        except ValueError as e:
            print(e)
            return -VALUE_LIMIT, 0
        return score, t
    
    # TODO probably use a different fine-tune algorithm after the broad search
    def findUltimateGain(kpMax=kpMax, nCalls=50):
        result = gp_minimize(lambda p: -getOscillationScore(p[0])[0],
                      [(0.0, kpMax)],
                      n_calls=nCalls,
                      verbose=True)
        return result.x[0]
    
    # print(getOscillationScore(31.836)) # TODO remove
    kp = findUltimateGain(kpMax=kpMax)
    score, t = getOscillationScore(kp)
    
    znFunctions = ZIEGLER_NICHOLS_FUNCTIONS[controlType]
    result = {keyName: znFunctions[keyName](kp, t) for keyName in kKeys}
    
    plotResult(controller, result, np.array([0] + [setPoint]*n))
    return result

# TODO some sort step response tuning

#TODO increase the power for the RMS to see if it more harshly punishes overshoot
#TODO add maximum slew rate, output limits to the FPID (realistic). Skip PID for n cycles if expected output will take n cycles to change.
#TODO read literature on tuning functions

def minimizeStepDeviationsPunishingOvershoot(controller: Tunable, nCycles: int=5, cycleFrequency: float=10, ncalls: int=200, kpRange=None, kiRange=None, kdRange=None, firstSetPoint: float=0, secondSetPoint: float=1, verbose: bool=False):
    stepSamples = int(round(0.5/(cycleFrequency*controller.T)))
    inputs = np.tile([firstSetPoint]*stepSamples + [secondSetPoint]*stepSamples, nCycles)
    calculateScore = generateCalculateScore(controller=controller,
                                            inputs=inputs,
                                            scoreCalculation=lambda i, f: punishOvershoot(i, f, overshootfn=lambda x: rmp(x, 2)**(1 + max(np.abs(x)))))
    result = runGpMinimizeTuning(controller=controller,
                               calculateScore=calculateScore,
                               ncalls=ncalls,
                               kpRange=kpRange,
                               kiRange=kiRange,
                               kdRange=kdRange,
                               verbose=verbose)
    
    plotResult(controller, result, inputs)
    return result

def minimizeStepDeviations(controller: Tunable, nCycles: int=5, cycleFrequency: float=10, ncalls: int=200, kpRange=None, kiRange=None, kdRange=None, firstSetPoint: float=0, secondSetPoint: float=1, verbose: bool=False):
    stepSamples = int(round(0.5/(cycleFrequency*controller.T)))
    inputs = np.tile([firstSetPoint]*stepSamples + [secondSetPoint]*stepSamples, nCycles)
    calculateScore = generateCalculateScore(controller=controller,
                                            inputs=inputs,
                                            scoreCalculation=lambda i, f: rms(i - f))
    result = runGpMinimizeTuning(controller=controller,
                               calculateScore=calculateScore,
                               ncalls=ncalls,
                               kpRange=kpRange,
                               kiRange=kiRange,
                               kdRange=kdRange,
                               verbose=verbose)
    
    plotResult(controller, result, inputs)
    return result

def minimizeSteadystateDeviation(controller: Tunable, nsamples: int=int(1e4), ncalls: int=200, kpRange=None, kiRange=None, kdRange=None, setpoint: float=None, verbose: bool=False):
    if setpoint is None:
        setpoint = controller.setpoint
    calculateScore = generateCalculateScore(controller=controller,
                                            inputs=[setpoint]*nsamples)
    return runGpMinimizeTuning(controller=controller,
                               calculateScore=calculateScore,
                               ncalls=ncalls,
                               kpRange=kpRange,
                               kiRange=kiRange,
                               kdRange=kdRange,
                               verbose=verbose)
    
def plotResult(controller: Tunable, result: dict, inputs: list[float]):
    retuneFn = createRetuneFunction(controller, {k: k in result for k in kKeys})
    retuneFn(result.values())
    controller.reset()
    feedbacks, outputs = controller.getOutputs(inputs)
    ts = np.arange(0, len(feedbacks)*controller.T, controller.T)
    
    subTitleString = ",".join(f"{k}={v:0.3f}" for k,v in result.items())
    
    fig, (ax1, ax2) = plt.subplots(2,1, sharex=True)
    ax2.plot(ts, outputs, label="outputs", color="black")
    ax2.set_ylabel("Controller Outputs")
    ax1.plot(ts, feedbacks, label="feedbacks")
    ax1.plot(ts, inputs[:-1], label="setpoints", alpha=0.5)
    ax1.set_ylabel("Process Values")
    ax1.legend(loc="upper right")
    ax2.legend(loc="lower right")
    ax2.set_xlabel("time")
    ax1.set_title(f"Feedback Response\n{subTitleString}")
    plt.show()

def generateCalculateScore(controller: Tunable, inputs: list[float], scoreCalculation=lambda i, o: rms(i-o)):
    def calculateScore():
        try:
            controller.reset()
            outputs, _ = controller.getOutputs(inputs)
            score = scoreCalculation(outputs, inputs[:-1])
            return score
        except ValueError as e:
            print(e)
            return VALUE_LIMIT
    return calculateScore

def runGpMinimizeTuning(controller: Tunable, calculateScore: callable, ncalls: int=200, kpRange=None, kiRange=None, kdRange=None, verbose: bool=False):
    retuneFunction = createRetuneFunction(  controller, 
                                            kp=kpRange is not None,
                                            ki=kiRange is not None,
                                            kd=kdRange is not None)
    
    def getScore(*args):
        # if any(np.isnan(x) for x in args[0]):
        #     return sys.float_info.max
        if verbose:
            print(f"parameters: {args}")
        retuneFunction(*args)
        score = calculateScore()
        if not np.isfinite(score) or score > VALUE_LIMIT:
            print(f"infinite score: {score}")
            return VALUE_LIMIT
        else:
            return score
    
    limits = createLimits(kpRange=kpRange, kiRange=kiRange, kdRange=kdRange)
    
    res = gp_minimize(getScore,
                      limits,
                      n_calls=ncalls,
                      verbose=True,
                      n_points=100,
                      n_initial_points=ncalls//2)
    params = list(res.x)
    retuneFunction(params)
    
    resultsDict = {}
    if kpRange is not None:
        resultsDict["kp"] = params.pop(0)
    if kiRange is not None:
        resultsDict["ki"] = params.pop(0)
    if kdRange is not None:
        resultsDict["kd"] = params.pop(0)
    
    return resultsDict
    
def createLimits(kpRange=None, kiRange=None, kdRange=None):
    limits = []
    if kpRange is not None:
        limits.append(tuple([float(p) for p in kpRange]))
    if kiRange is not None:
        limits.append(tuple([float(p) for p in kiRange]))
    if kdRange is not None:
        limits.append(tuple([float(p) for p in kdRange]))
    
    return tuple(limits)

def createRetuneFunction(controller, kp=False, ki=False, kd=False):
    def retune(*args):
        ps = list(*args)
        if kp:
            controller.kp = ps.pop(0)
        if ki:
            controller.ki = ps.pop(0)
        if kd:
            controller.kd = ps.pop(0)
    return retune


def rmp(x: list[float], p: int) -> float:
    return np.sqrt(np.mean(x**p))

def rms(x: list[float]) -> float:
    return rmp(x, 2)

def punishOvershoot(desired, actual, overshootfn=lambda x: rmp(x,4), undershootfn=lambda x: rmp(x, 2)):
    #overshoot options: rmp(x,2)**(1 + max(x)), rmp(x,4)
    underPoints = deque()
    overPoints = deque()
    for d, a in zip(desired, actual):
        x = d - a
        if  np.abs(a) > np.abs(d):
            overPoints.append(x)
        else:
            underPoints.append(x)
    return overshootfn(np.array(overPoints)) + undershootfn(np.array(underPoints))