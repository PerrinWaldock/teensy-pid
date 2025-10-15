from .model import FunctionModel
import numpy as np
  
class LPF(FunctionModel):
    def __init__(self, fKnee: float, T: float, startingValue: float=0):
        self.alpha = np.exp(-2*np.pi*T*fKnee)
        def fn(inputs, outputs):
            if len(outputs) > 0:
                lastOutput = outputs[-1]
            else:
                lastOutput = startingValue
            return self.alpha*lastOutput + (1 - self.alpha)*inputs[-1]
        super().__init__(fn)

class Delay(FunctionModel):
    def __init__(self, lag: int=0, startingValue: float=0):
        self.lag = lag
        self.startingValue = startingValue
        
        def fn(inputs, _):
            if len(inputs) > self.lag:
                return self.inputs[-1 - self.lag]
            else:
                return self.startingValue
        super().__init__(fn, useLists=True)