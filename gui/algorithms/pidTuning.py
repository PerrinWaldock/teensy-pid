from abc import ABC, abstractmethod
import sys
import os
import numpy as np
from skopt import gp_minimize

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

# TODO create different tuning methods
def zeigler_nichols(controller: Tunable, startingFrequency: float=1e-3):
    # TODO
    pass

# TODO some sort step response tuning

def minimizeSteadystateDeviation(controller: Tunable, nsamples: int=int(1e4), ncalls: int=200, kpRange=None, kiRange=None, kdRange=None, setpoint: float=None):
    if setpoint is None:
        setpoint = controller.setpoint
    def calculateScore():
        try:
            controller.reset() #TODO clear?
            inputs = np.array([setpoint]*nsamples)
            outputs = controller.getOutputs(inputs)
            score = rms(outputs - inputs[1:]) #TODO calculate RMS
            return score
        except ValueError as e:
            print(e)
            return sys.float_info.max
    
    return runGpMinimizeTuning(controller=controller,
                               calculateScore=calculateScore,
                               ncalls=ncalls,
                               kpRange=kpRange,
                               kiRange=kiRange,
                               kdRange=kdRange)

def runGpMinimizeTuning(controller: Tunable, calculateScore: callable, ncalls: int=200, kpRange=None, kiRange=None, kdRange=None):
    retuneFunction = createRetuneFunction(  controller, 
                                            kp=kpRange is not None,
                                            ki=kiRange is not None,
                                            kd=kdRange is not None)
    
    def getScore(*args):
        retuneFunction(*args)
        return calculateScore()
    
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
        limits.append(kpRange)
    if kiRange is not None:
        limits.append(kiRange)
    if kdRange is not None:
        limits.append(kdRange)
    
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

def rms(x):
    return np.sqrt(np.mean(x**2))