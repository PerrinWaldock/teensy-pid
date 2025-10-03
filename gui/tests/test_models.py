import numpy as np
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from controller import *
import math

from controller.models import PID, LPF, GaussianNoise, ModelCollection
import pytest


def test_lpf():
    T = 1e-5
    Fsignal = 100
    FKnee = 100
    ts = np.linspace(0,1,int(1/T))
    inputs = np.sin(2*np.pi*Fsignal*ts)
    
    model = LPF(FKnee, T)
    outputs = list(model.simulate(inputs))

    expected = np.std(inputs)/np.sqrt(2) 
    measured = np.std(outputs)
    
    assert math.isclose(expected, measured, rel_tol=1e-3)

def test_noise():
    T = 1e-5
    Fsignal = 10
    amplitude = 10
    ts = np.linspace(0,1,int(1/T))
    inputs = amplitude*np.sin(2*np.pi*Fsignal*ts)
    deviation = .5*amplitude
    
    model = GaussianNoise(deviation)
    outputs = list(model.simulate(inputs))
    stdev = np.std(inputs - outputs)
    
    tolerance = deviation/amplitude*1e-2
    assert math.isclose(deviation, stdev, rel_tol=tolerance)

def test_pid():
    T = 1e-5
    Fsignal = 10
    amplitude = 10
    ts = np.linspace(0,1,int(1/T))
    inputs = amplitude*np.sin(2*np.pi*Fsignal*ts)
    
    Fknee = 100
    Tlag = 1e-4
    lpf = LPF(fKnee=Fknee, T=T)
    delay = Delay(int(Tlag/T))
    plant = ModelCollection([lpf, delay])
    
    model = PID(T=T, kp=20, ki=1000, kd=0, model=plant)
    outputs = list(model.simulate(inputs))
    
    import matplotlib.pyplot as plt
    plt.plot(ts, inputs, label="setpoint")
    plt.plot(ts, outputs, label="output")
    plt.plot(ts, model.feedbacks, label="feedback")
    plt.plot(ts, model.errors, label="error")
    plt.legend()
    plt.show()
    
    #TODO add some sort of assert
    #TODO hook up a tuning algorithm to this virtual model
    
    