import numpy as np
from scipy.interpolate import interp1d
from abc import abstractmethod
from collections import deque
from collections.abc import Callable

from .model import Model

class PidCalculator:
    def __init__(self, 
                 kp: float, 
                 ki: float, 
                 kd: float, 
                 T=1, 
                 invertOutput=False):
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
        self._lasterror = 0

class PidCalculatorContainer:
    def __init__(self, 
                 calculator: PidCalculator):
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
    
    @abstractmethod
    def clear(self) -> None: pass

class PID(Model, PidCalculatorContainer):
    # inputs are setpoints, outputs are outputs
    def __init__(self, 
                 T: float, 
                 kp: float, 
                 ki: float, 
                 kd: float, 
                 model: Model):
        
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
    def __init__(self, 
                 model: Model, 
                 minSetpoint: float, 
                 maxSetpoint: float, 
                 minOutput: float, 
                 maxOutput: float, 
                 maxRiseRate: float, 
                 T: float, 
                 kp: float, 
                 ki: float, 
                 kd: float, 
                 resolution: int=12, 
                 setpointTolerance: float=None):
        self._waitCycles = 0
        self.setpointTolerance = setpointTolerance
        if setpointTolerance is None:
            self.setpointTolerance = (maxSetpoint - minSetpoint) / (1 << resolution)
        
        self.model = model
        
        self.maxRiseRate = maxRiseRate*T #assume units are volts per second
        assert minSetpoint < maxSetpoint
        self.minSetpoint = minSetpoint
        self.maxSetpoint = maxSetpoint
        assert minOutput < maxOutput
        self.minOutput = minOutput
        self.maxOutput = maxOutput
        
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
    
    def _calculateNextOutput(self, setpoint: float) -> float:
        boundedSetpoint = min(max(setpoint, self.minSetpoint), self.maxSetpoint)
        setpointBounded = setpoint != boundedSetpoint
        setpoint = boundedSetpoint
        setpointChanged = len(self._setpoints) == 0 or np.abs(self._setpoints[-1] - setpoint) >= self.setpointTolerance
        if setpointChanged and len(self._setpoints) > 0:
            desiredChange = self._setpoints[-1] - setpoint
            self._waitCycles = int(round(np.abs(desiredChange)/(self.maxRiseRate))) + 1
        self._waitCycles -= 1
        
        output = self.feedForwardModel(setpoint)
        adjustment = 0
        if (self._waitCycles <= 0
            and not setpointChanged 
            and not setpointBounded):
            error = setpoint - self.feedbacks[-1]
            adjustment = self.calculator.next(error)
        self._setpoints.append(setpoint)
        self._adjustments.append(adjustment)
        output += adjustment
        return output
        
    def _setOutput(self, value: float) -> None:
        if value > self.maxOutput:
            value = self.maxOutput
        elif value < self.minOutput:
            value = self.minOutput
        self.model.next(value)
        return value
    
    def next(self, setpoint: float) -> float:
        output = self._calculateNextOutput(setpoint)
        output = self._setOutput(output)
        return output

    def clear(self):
        self._setpoints = deque()
        self.calculator.clear()
        self.model.clear()
        self.reset()
    
    def _generateCalibrationFunction(self, stepSize: float=None, settleTime: float=None) -> list[float]:
        # TODO separate settle and measure times based on max rise rate
        DEFAULT_RESOLUTION_SKIP = 4
        if stepSize is None:
            stepSize = (self.maxSetpoint - self.minSetpoint)/(1 << (self.resolution - DEFAULT_RESOLUTION_SKIP))
        
        if settleTime is None:
            settleTime = self.T*(1 << DEFAULT_RESOLUTION_SKIP)
            
        repeats = int(round(settleTime/self.T))
        controllerOutputs = np.repeat(np.arange(self.minOutput, self.maxOutput, stepSize), repeats=repeats)
        controllerOutputs = np.concat((controllerOutputs, np.flip(controllerOutputs)))
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