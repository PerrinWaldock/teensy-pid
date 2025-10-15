"""
TODO
    create method that creates a model from a complex fourier spectrum + sample frequency so that coloured noise can be created
    break up into multiple models
"""
import numpy as np
from scipy.interpolate import interp1d
from abc import ABC, abstractmethod
from collections import deque
from collections.abc import Callable
from random import random

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
    def __init__(self, 
                 fn: Callable[[list[float], list[float]], None], 
                 useLists=False):
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

class ModelCollection(Model):
    def __init__(self, 
                 models: list[Model]):
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