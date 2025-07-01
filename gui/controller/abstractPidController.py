from abc import ABC, abstractmethod

class AbstractPidController(ABC):
    
    def __init__(self):
        self._svind = 0 
    
    @property
    @abstractmethod
    def kd(self):
        pass
    
    @property
    @abstractmethod
    def ki(self):
        pass
    
    @property
    @abstractmethod
    def kp(self):
        pass
    
    @property
    @abstractmethod
    def loopFrequency(self):
        pass
    
    @property
    def sv(self):
        return self.svs[self._svind]
    
    @property
    def svind(self):
        return self._svind
    
    @property
    @abstractmethod
    def svs(self):
        pass
    
    @property
    @abstractmethod
    def pidActive(self):
        pass
    
    @abstractmethod
    def save(self):
        pass
    
    @abstractmethod
    def load(self):
        pass
    
    @abstractmethod
    def calibrate(self):
        pass