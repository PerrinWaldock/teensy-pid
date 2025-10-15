import numpy as np
from scipy.stats import norm
from random import random
from collections import deque

from .model import FunctionModel, Model

class GaussianNoise(FunctionModel):
    def __init__(self, deviation: float=0):
        self.deviation = deviation
        def fn(inputs, _):
            return norm.rvs(scale=self.deviation, loc=inputs[-1], size=1)[0]
        super().__init__(fn)

class SinusoidalNoise(FunctionModel):
    def __init__(self, T: float, amplitude: float, f: float):
        self.t = 0
        def fn(inputs, _):
            self.t += T
            return inputs[-1] + amplitude*np.sin(2*np.pi*f*self.t)
        super().__init__(fn)
    
    @property
    def adjustments(self) -> list[float]:
        return np.array(self.outputs) - np.array(self.inputs)
    
class RandomWalkNoise(FunctionModel):
    def __init__(self, T: float, maxSlope: float, startingValue: float=None):
        maxStep = maxSlope*T
        if startingValue is None:
            startingValue = 0
        self.offsets = deque()
        def fn(inputs, outputs):
            if len(outputs) > 1:
                lastOffset = self.offsets[-1]
            else:
                lastOffset = 0
            x = 2*random() - 1
            offset = x*maxStep + lastOffset
            self.offsets.append(offset)
            return offset + inputs[-1]
        super().__init__(fn)
    
    def clear(self):
        self.offsets = deque()
        super().clear()

class CenteredRandomWalkNoise(FunctionModel):
    def __init__(self, T: float, maxSlope: float, minValue: float, maxValue: float, startingValue: float=None):
        maxStep = maxSlope*T
        if startingValue is None:
            startingValue = (maxValue + minValue)/2
        self.offsets = deque()
        def fn(inputs, outputs):
            if len(self.offsets) > 0:
                lastValue = self.offsets[-1]
            else:
                lastValue = startingValue
            if len(outputs) > 1:
                lastOutput = outputs[-1]
            else:
                lastOutput = 0
            x = random() - 0.5
            bias = (maxValue - lastValue)/(maxValue - minValue) - 0.5
            offset = (x + bias)*maxStep + lastOutput
            self.offsets.append(offset)
            return offset + inputs[-1]
        super().__init__(fn)
    
    def clear(self):
        self.offsets = deque()
        super().clear()
        
class SpectrumNoise(FunctionModel):
    def __init__(self, spectrum: list[float], realSpectrum=False):
        if realSpectrum:
            spectrum = np.flip(spectrum[1:]) + [2*spectrum[0]] + spectrum[1:]
        
        spectrum = np.array(spectrum, dtype="complex")
        Np = (len(spectrum) - 1) // 2
        phases = np.random.rand(Np) * 2 * np.pi
        phases = np.cos(phases) + (1j * np.sin(phases))
        spectrum[1:Np+1] *= phases
        spectrum[-1:-1-Np:-1] = np.conj(spectrum[1:Np+1])
        self.noiseArray = np.fft.ifft(spectrum).real
        self.noiseArrayIndex = 0
        self.offsets = deque()
        def fn(inputs, outputs):
            if self.noiseArrayIndex >= len(self.noiseArray):
                self.noiseArrayIndex = 0
            offset = self.noiseArray[self.noiseArrayIndex]
            self.noiseArrayIndex += 1
            self.offsets.append(offset)
            return inputs[-1] + offset
        super().__init__(fn)
    
    def clear(self):
        self.offsets = deque()
        super().clear()