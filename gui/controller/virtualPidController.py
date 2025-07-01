from .abstractPidController import AbstractPidController

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

if __name__ == "__main__":
    vpid = VirtualPidController(kp=1,ki=1,kd=0)
    print(vpid.kd)
    
    #TODO set up proper tests