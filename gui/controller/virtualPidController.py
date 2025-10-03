import numpy as np
from typing import List, Tuple

from .abstractPidController import AbstractPidController


MIN_VOLTAGE = 0
MAX_VOLTAGE = 5

def valueLogger(fn):
    def wrapper(*args, **kwargs):
        print(f'{fn.__name__}={args[1:]}')
        return fn(*args, **kwargs)
    return wrapper

class VirtualPidController(AbstractPidController):
    def __init__(self, **kwargs):        
        self._kp = 1
        self._ki = 1
        self._kd = 1
        self._loopFrequency = 10000
        self._pidActive = True
        self._svs = [0, 1, 2, 3]
        self._svind = 0
        self._svLimits = (MIN_VOLTAGE, MAX_VOLTAGE)
        
        for key, val in kwargs.items():
            if key[0] != "_" and key in dir(self):
                setattr(self, key, val)
                
    @property
    def ki(self):
        return self._ki
    
    @ki.setter
    @valueLogger
    def ki(self, value):
        self._ki = value

    @property
    def kp(self):
        return self._kp
    
    @kp.setter
    @valueLogger
    def kp(self, value):
        self._kp = value

    @property
    def kd(self):
        return self._kd
    
    @kd.setter
    @valueLogger
    def kd(self, value):
        self._kd = value

    @property
    def pidActive(self):
        return self._pidActive
    
    @pidActive.setter
    @valueLogger
    def pidActive(self, value):
        self._pidActive = value

    @property
    def loopFrequency(self):
        return self._loopFrequency
    
    @loopFrequency.setter
    @valueLogger
    def loopFrequency(self, value):
        self._loopFrequency = value
        
    @property
    def svs(self):
        return self._svs
    
    @svs.setter
    def svs(self, value):
        self._svs = value
            
    def load(self):
        pass
    
    def save(self):
        pass
    
    def calibrate(self):
        pass
    
    def getFeedForwardReadings(self) -> Tuple[List[float], List[float]]:
        inputs = np.linspace(MIN_VOLTAGE, MAX_VOLTAGE)
        outputs = np.linspace(MIN_VOLTAGE, MAX_VOLTAGE)
        return inputs, outputs
        
    def forceOutput(self, voltage):
        self.pidActive = False
    
    def getSetpointLimits(self) -> Tuple[float]:
        return self._svLimits
    
    #TODO start and get log
    
    #TODO log history of setpoints so a log can be reconstructed
    #TODO inject a feedforward model in the constructor (e.g. linear with lag and random term)
    #  model should be a function that takes in a history of outputs and spits out a new input

if __name__ == "__main__":
    vpid = VirtualPidController(kp=1,ki=1,kd=0)
    print(vpid.kd)
    
    #TODO set up proper tests