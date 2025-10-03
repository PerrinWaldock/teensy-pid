from abc import ABC, abstractmethod
import numpy as np
import sys
import os
from skopt import gp_minimize
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

from controller.models import *
from algorithms.pidTuning import Tunable, minimizeSteadystateDeviation


def main():
    T = 1e-5
    fKnee = 100
    kp = 20
    ki = 1000
    kd = 0
    Tdelay = 1e-5
    delaySteps = int(Tdelay/T)
    noiseDeviation = 0.5
    startingValue = 1
    
    plantModel = ModelCollection([
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
    
    result = minimizeSteadystateDeviation(tunableModel, **limits, setpoint=startingValue, ncalls=50)
    print(result)

#TODO there should be a way of doing this with multiple inheritence (or maybe metaclasses)
class TunableModel(Tunable):
    def __init__(self, model: PidCalculatorContainer):
        self.model = model
    
    def getOutputs(self, inputs):
        outputs = list(self.model.simulate(inputs))
        return self.model.feedbacks[-len(inputs):-1]
    
    def reset(self): self.model.reset()
    
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