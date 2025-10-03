"""
what architecture is best?
    fully functional?
    object-oriented (pass in parameters, ask to simulate 100 points?)
    time-based? frequency-based
    
TODO
    create method that creates a model from a complex fourier spectrum + sample frequency
"""
import math
import numpy as np
from scipy.stats import norm
from scipy.interpolate import interp1d
from abc import ABC, abstractmethod
from collections import deque
from collections.abc import Callable

class Model(ABC):
    @abstractmethod
    def next(self, input: float): pass
    
    @property
    @abstractmethod
    def inputs(self) -> list[float]: pass
    
    @property
    @abstractmethod
    def outputs(self) -> list[float]: pass
    
    @abstractmethod
    def clear(self) -> None: pass
    
    def simulate(self, inputs: list[float]):
        for input in inputs:
            yield self.next(input)
            
class FunctionModel(Model):
    #function takes full input history, full output history, generates next output
    def __init__(self, fn: Callable[[list[float], list[float]], None], useLists=False):
        self.fn = fn
        self._inputs = deque()
        self._outputs = deque()
        if useLists:
            self._inputs = list()
            self._outputs = list()
    
    def next(self, input: float) -> float:
        self._inputs.append(input)
        self._outputs.append(self.fn(self._inputs, self._outputs))
        return self._outputs[-1]
    
    def clear(self) -> None:
        self._inputs = deque()
        self._outputs = deque()

    @property
    def inputs(self) -> list[float]: return self._inputs
    
    @property
    def outputs(self) -> list[float]: return self._outputs

  
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

class GaussianNoise(FunctionModel):
    def __init__(self, deviation=0):
        self.deviation = deviation
        def fn(inputs, _):
            return norm.rvs(scale=self.deviation, loc=inputs[-1], size=1)[0]
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

class ModelCollection(Model):
    def __init__(self, models: list[Model]):
        self.models = models
    
    def next(self, input: float):
        for model in self.models:
            input = model.next(input)
        return input
    
    def clear(self) -> None:
        for model in self.models:
            model.clear()
    
    @property
    def inputs(self) -> list[float]: return self.models[0].inputs
    
    @property
    def outputs(self) -> list[float]: return self.models[-1].outputs

class PidCalculator:
    def __init__(self, kp: float, ki: float, kd: float, T=1, invertOutput=False):
        self.errors = deque()
        
        assert T > 0
        if invertOutput:
            T *= -1
        self._T = T
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self._lasterror = 0
        self._errorsum = 0
    
    @property
    def T(self): return np.abs(self._T)
    @T.setter
    def T(self, value):
        assert value > 0
        self._T = value * (self._T / np.abs(self._T))
        
    @property
    def kp(self): return self._kp
    @kp.setter
    def kp(self, value): self._kp = value
    
    @property
    def ki(self): return self._ki/self._T
    @ki.setter
    def ki(self, value): self._ki = value*self._T
    
    @property
    def kd(self): return self._kd*self._T
    @kd.setter
    def kd(self, value): self._kd = value/self._T
    
    def next(self, error):
        self.errors.append(error)
        self._errorsum += error
        output = (self._kp*error) \
                  + (self._kd*(error - self._lasterror)) \
                + (self._ki*self._errorsum)
        self._lasterror = error
        return output

    def clear(self):
        self.errors = deque()
        self.reset()
    
    def reset(self):
        self._errorsum = 0

class PidCalculatorContainer:
    def __init__(self, calculator: PidCalculator):
        self.calculator = calculator
    
    @property
    def T(self) -> float: return self.calculator.T
    @T.setter
    def T(self, value: float) -> None: self.calculator.T = value
    
    @property
    def kp(self) -> float: return self.calculator.kp
    @kp.setter
    def kp(self, value: float) -> None: self.calculator.kp = value
    
    @property
    def ki(self) -> float: return self.calculator.ki
    @ki.setter
    def ki(self, value: float) -> None: self.calculator.ki = value
    
    @property
    def kd(self) -> float: return self.calculator.kd
    @kd.setter
    def kd(self, value: float) -> None: self.calculator.ld = value
    
    @property
    def errors(self) -> list[float]: return self.calculator.errors
    
    def reset(self) -> None:
        self.calculator.reset()

class PID(Model, PidCalculatorContainer):
    # inputs are setpoints, outputs are outputs
    def __init__(self, T: float, kp: float, ki: float, kd: float, model: Model):
        self._setpoints = deque()        
        self.model = model
        super().__init__(PidCalculator(kp, ki, kd, T=T))
        
    def next(self, setpoint: float) -> float:
        self._setpoints.append(setpoint)
        if len(self.feedbacks) > 0:
            error = setpoint - self.feedbacks[-1]
        else:
            error = 0
        output = self.calculator.next(error)
        self.model.next(output)
        return output

    def clear(self):
        self._setpoints = deque()
        self.calculator.clear()
        self.model.clear()
        self.reset()
    
    @property
    def feedbacks(self) -> list[float]: return self.model.outputs
    
    @property  
    def inputs(self) -> list[float]: return self._setpoints
    
    @property
    def outputs(self) -> list[float]: return self.model.inputs

class FPID(Model, PidCalculatorContainer):
    def __init__(self, model: Model, minSetpoint: float, maxSetpoint: float, T: float, kp: float, ki: float, kd: float, resolution: int=12, setpointTolerance: float=None):
        self.setpointTolerance = setpointTolerance
        if setpointTolerance is None:
            self.setpointTolerance = (maxSetpoint - minSetpoint) / (1 << resolution)
        
        self.model = model
        self.minSetpoint = minSetpoint
        self.maxSetpoint = maxSetpoint
        self.resolution = resolution
        self.pidActive = False #TODO implement
        
        self._setpoints = deque()
        self._adjustments = deque()
        super().__init__(PidCalculator(kp, ki, kd, T)) #TODO this may crash and burn
        self.calibrate()
    
    @property
    def feedbacks(self) -> list[float]: return self.model.outputs
    
    @property  
    def inputs(self) -> list[float]: return self._setpoints
    
    @property
    def outputs(self) -> list[float]: return self.model.inputs
    
    def next(self, setpoint: float) -> float:
        output = self.feedForwardModel(setpoint)
        adjustment = 0
        if len(self._setpoints) > 0 and math.abs(self._setpoints[-1] - setpoint) <= self.setpointTolerance:
            error = setpoint - self.feedbacks[-1]
            adjustment = self.calculator.next(error)
        self._setpoints.append(setpoint)
        self._adjustments.append(adjustment)
        output += adjustment
        self.model.next(output)
        return output

    def clear(self):
        self._setpoints = deque()
        self.calculator.clear()
        self.model.clear()
        self.reset()
    
    def _generateCalibrationFunction(self, stepSize: float=None, settleTime: float=None) -> list[float]:
        if stepSize is None:
            stepSize = (self.maxSetpoint - self.minSetpoint)/(1 << self.resolution)
        
        if settleTime is None:
            settleTime = self.T
            
        repeats = int(round(settleTime/self.T))
        controllerOutputs = np.repeat(np.arange(self.minSetpoint, self.maxSetpoint, stepSize), repeats=repeats)
        controllerOutputs = np.concat((controllerOutputs, np.reverse(controllerOutputs)))
        return controllerOutputs
        
    def calibrate(self, stepSize: float=None, settleTime: float=None) -> None:
        controllerOutputs = self._generateCalibrationFunction(stepSize, settleTime)
        controllerInputs = self.model.simulate(controllerOutputs)
        
        measuredValues = dict()
        for o, i in zip(controllerOutputs, controllerInputs):
            if not o in measuredValues:
                measuredValues[o] = deque()
            measuredValues[o].append(i)

        averagedControllerOutputs = sorted(measuredValues.keys())
        averagedControllerInputs = deque()
        for o in averagedControllerOutputs:
            averagedControllerInputs.append(np.mean(measuredValues[o]))

        self.feedForwardModel = interp1d(averagedControllerInputs, averagedControllerOutputs)
        