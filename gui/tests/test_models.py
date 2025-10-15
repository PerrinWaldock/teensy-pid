import numpy as np
import sys
import os
import math
import pytest

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from controller import *
from simulation.noise import *
from simulation.filters import *
from simulation.model import *
from simulation.feedback import *

def main():
    test_spectrum_noise()

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

def test_gaussian_noise():
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

#TODO random walk noise and biased random walk noise
def test_random_walk_noise():
    T = 1e-3
    ts = np.linspace(0,1,int(1/T))
    inputs = [0]*len(ts)
    
    model = RandomWalkNoise(T, 10)
    outputs = list(model.simulate(inputs))
    
    plt.plot(ts, outputs)
    plt.show()

def test_spectrum_noise():
    # TODO create flat spectrum of noise
    T = 1e-3
    N = int(1/T)
    spectrum = [1/(n+1) for n in range(N)]
    inputs = [0]*N
    ts = np.arange(len(inputs))/T
    model = SpectrumNoise(spectrum)
    outputs = list(model.simulate(inputs))
    
    #TODO delete .pyc files, add .pyc to gitignore
    plt.figure()
    plt.plot(ts, outputs)
    plt.xlabel("time (s)")
    plt.title("noise vs time")
    plt.figure()
    signalSpectrum = np.fft.rfft(outputs)
    plt.plot(np.fft.rfftfreq(2*N-1, T), spectrum, label="desired spectrum")
    plt.plot(np.fft.rfftfreq(len(outputs), T), np.abs(signalSpectrum), label="signal spectrum")
    # plt.yscale("log")
    plt.title("Noise Spectrum")
    plt.xlabel("Frequency (Hz)")
    plt.legend()
    plt.show()

#TODO random walk noise and biased random walk noise
def test_centered_random_walk_noise():
    T = 1e-3
    ts = np.linspace(0,1,int(1/T))
    inputs = [0]*len(ts)
    
    model = CenteredRandomWalkNoise(T, 10, -1, 1)
    outputs = list(model.simulate(inputs))
    
    plt.plot(ts, outputs)
    plt.show()

def test_delay():
    T = 1e-5
    lag = 10
    Fsignal = 10
    amplitude = 10
    ts = np.linspace(0,1,int(1/T))
    inputs = amplitude*np.sin(2*np.pi*Fsignal*ts)
    startingValue = inputs[0]
    model = Delay(lag, startingValue=startingValue)
    outputs = list(model.simulate(inputs))
    
    for ind, o in enumerate(outputs):
        if ind < lag:
            assert startingValue == o
        else:
            assert inputs[ind-lag] == o

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
    
if __name__ == "__main__":
    main()