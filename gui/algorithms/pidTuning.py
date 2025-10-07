from abc import ABC, abstractmethod
import sys
import os
import numpy as np
from skopt import gp_minimize
from scipy import fft
import math
import matplotlib.pyplot as plt

VALUE_LIMIT = sys.float_info.max/1e200

kKeys = ["kp", "ki", "kd"]

class Tunable(ABC):
    @abstractmethod
    def getOutputs(self, inputs: list[float]) -> list[float]: pass
    
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
    
    # TODO create a fit to sinusoid method
    def sineFit(signal, guess):
        pass
    
    def sinusoidScore(signal):
        if any(not np.isfinite(x) for x in signal):
            return 0, 0
        zerodsignal = (signal - np.mean(signal))
        normsignal = zerodsignal/max(zerodsignal)
        spectrum = np.abs(np.fft.rfft(normsignal))/len(normsignal)
        freqs = np.fft.rfftfreq(len(normsignal), controller.T)
        maxInd = np.argmax(spectrum)
        peakF = freqs[maxInd]
        score = spectrum[maxInd]
        return score, 1/peakF
    
    def getOscillationScore(kp):
        controller.kp = kp
        controller.reset()
        inputs = [0] + [setPoint]*n
        outputs = controller.getOutputs(inputs)
        score, t = sinusoidScore(outputs)
        return score, t
    
    # TODO probably use a different fine-tune algorithm after the broad search
    def findUltimateGain(kpMax=kpMax, nCalls=100):
        result = gp_minimize(lambda p: -getOscillationScore(p[0])[0],
                      [(0.0, kpMax)],
                      n_calls=nCalls,
                      verbose=True)
        return result.x[0]
    
    getOscillationScore(0)
    
    kp = findUltimateGain(kpMax=kpMax)
    score, t = getOscillationScore(kp)
    
    znFunctions = ZIEGLER_NICHOLS_FUNCTIONS[controlType]
    results = {keyName: znFunctions[keyName](kp, t) for keyName in kKeys}
    return results

# TODO some sort step response tuning

#TODO increase the power for the RMS to see if it more harshly punishes overshoot
#TODO add maximum slew rate, output limits to the FPID (realistic). Skip PID for n cycles if expected output will take n cycles to change.
#TODO read literature on tuning functions

def minimizeStepDeviations(controller: Tunable, nCycles: int=5, cycleFrequency: float=10, ncalls: int=200, kpRange=None, kiRange=None, kdRange=None, firstSetPoint: float=0, secondSetPoint: float=1, verbose: bool=False):
    stepSamples = int(round(0.5/(cycleFrequency*controller.T)))
    inputs = np.tile([firstSetPoint]*stepSamples + [secondSetPoint]*stepSamples, nCycles)
    calculateScore = generateCalculateScore(controller=controller,
                                            inputs=inputs)
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
    outputs = controller.getOutputs(inputs)
    ts = np.arange(0, len(outputs)*controller.T, controller.T)
    
    plt.plot(ts, inputs[:-1], label="inputs")
    plt.plot(ts, outputs, label="feedbacks")
    plt.xlabel("time")
    plt.title(f"Feedback Response with: {result}")
    plt.show()

def rmsFromIo(outputs, inputs):
    return rms(outputs - inputs)

def generateCalculateScore(controller: Tunable, inputs: list[float], scoreCalculation=rmsFromIo):
    def calculateScore():
        try:
            controller.reset()
            outputs = controller.getOutputs(inputs)
            score = scoreCalculation(outputs, inputs[:-1])
            return score
        except ValueError as e:
            print(e)
            return sys.float_info.max/VALUE_LIMIT
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
                      verbose=True)
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