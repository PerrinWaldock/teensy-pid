from abc import ABC, abstractmethod
import numpy as np
import sys
import os
from skopt import gp_minimize
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

from controller.models import *
from algorithms.pidTuning import *

# TODO look at fourier spectrum and allan deviation of different models
# TODO create "visualize" and "generate" fourier and allan functions
# TODO create a "coloured noise" model

def main():
    testMinimizeStepDeviationFpid()
    #testMinimizeStepDeviation()
    #testZnTuning()

def testZnTuning():
    T = 1e-5
    fKnee = 1e3
    kp = 20
    ki = 1000
    kd = 0
    Tdelay = 0*T
    delaySteps = int(Tdelay/T)
    noiseDeviation = 0#1e-5
    startingValue = 0
    
    plantModel = ModelCollection([
        # RandomWalkNoise(T, noiseDeviation/1e-3, -10, 10, startingValue=startingValue),
        GaussianNoise(noiseDeviation),
        LPF(fKnee, T, startingValue=startingValue),
        Delay(delaySteps, startingValue=startingValue)
    ])
    pidModel = PID(T=T, kp=kp, ki=ki, kd=kd, model=plantModel)
    tunableModel = TunableModel(pidModel)
    
    result = ziegler_nichols(tunableModel, "pid")
    
    print(result)

def testMinimizeStepDeviation():
    T = 1e-5
    fKnee = 1e3
    kp = 20
    ki = 1000
    kd = 0
    Tdelay = 0*T
    cycleFrequency = fKnee/10
    delaySteps = int(Tdelay/T)
    startingValue = 1
    noiseDeviation = 1e-2
    maxWalkSlope = 10*cycleFrequency
    
    plantModel = ModelCollection([
        RandomWalkNoise(T, maxWalkSlope, startingValue=startingValue),
        GaussianNoise(noiseDeviation),
        LPF(fKnee, T, startingValue=startingValue),
        Delay(delaySteps, startingValue=startingValue)
    ])
    pidModel = PID(T=T, kp=kp, ki=ki, kd=kd, model=plantModel)
    tunableModel = TunableModel(pidModel)
    
    limits = {
        "kpRange": (0, 100),
        "kiRange": (0, 1000000),
        "kdRange": (0, 100)
    }
    
    result = minimizeStepDeviationsPunishingOvershoot(
        tunableModel, 
        nCycles=5,
        cycleFrequency=cycleFrequency,
        firstSetPoint=startingValue,
        secondSetPoint=0,
        **limits, 
        ncalls=200, 
        verbose=True)
    
    print(result)
    
def testMinimizeStepDeviationFpid():
    T = 1e-5
    fKnee = 1e3
    kp = 20
    ki = 1000
    kd = 0
    Tdelay = 0*T
    cycleFrequency = fKnee/10
    delaySteps = int(Tdelay/T)
    startingValue = 1
    noiseDeviation = 1e-2
    maxWalkSlope = 10*cycleFrequency
    
    minSetpoint = -5
    maxSetpoint = 5
    minOutput = -5
    maxOutput = 5
    maxRiseRate = (maxSetpoint - minSetpoint)*fKnee*2
    
    plantModel = ModelCollection([
        RandomWalkNoise(T, maxWalkSlope, startingValue=startingValue),
        GaussianNoise(noiseDeviation),
        LPF(fKnee, T, startingValue=startingValue),
        Delay(delaySteps, startingValue=startingValue)
    ])
    pidModel = FPID(T=T, 
        maxRiseRate=maxRiseRate,
        minSetpoint=minSetpoint, 
        maxSetpoint=maxSetpoint,
        minOutput=minOutput,
        maxOutput=maxOutput,
        kp=kp, ki=ki, kd=kd, model=plantModel)
    tunableModel = TunableModel(pidModel)
    
    limits = {
        "kpRange": (0, 100),
        "kiRange": (0, 1000000),
        "kdRange": (0, 100)
    }
    
    result = minimizeStepDeviationsPunishingOvershoot(
        tunableModel, 
        nCycles=5,
        cycleFrequency=cycleFrequency,
        firstSetPoint=startingValue,
        secondSetPoint=0,
        **limits, 
        ncalls=200, 
        verbose=True)
    
    print(result)

    

#TODO there should be a way of doing this with multiple inheritence (or maybe metaclasses)
class TunableModel(Tunable):
    def __init__(self, model: PidCalculatorContainer):
        self.model = model
    
    def getOutputs(self, inputs, abortCondition=None):
        outputs = deque()
        for i in inputs:
            o = self.model.next(i)
            outputs.append(o)
            if abortCondition is not None and abortCondition(i, o, self.model.feedbacks[-1]):
                break
        return list(self.model.feedbacks)[-len(inputs):-1], list(outputs)[:-1]
    
    def reset(self): 
        self.model.reset()
        self.model.clear()
    
    @property
    def kp(self): return self.model.kp
    @kp.setter
    def kp(self, value): self.model.kp = value
    
    @property
    def ki(self): return self.model.ki
    @ki.setter
    def ki(self, value): self.model.ki = value
    
    @property
    def kd(self): return self.model.kd
    @kd.setter
    def kd(self, value): self.model.kd = value
    
    @property
    def T(self): return self.model.T
    @T.setter
    def T(self, value): self.model.T = value
    
    @property
    def setpoint(self): return self.model.inputs[-1]
    @setpoint.setter #TODO maybe better implementation
    def setpoint(self, value): 
        self.model.next(value)

if __name__ == "__main__":
    main()