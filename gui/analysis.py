import numpy as np
from collections import deque
import matplotlib.pyplot as plt

def spectrum(xs, T):
    amplitudes = np.abs(np.fft.fft(xs))
    amplitudes = amplitudes[:len(amplitudes)//2]
    amplitudes /= len(amplitudes)
    frequencies = np.linspace(0,1/(2*T),len(amplitudes))
    return (frequencies, amplitudes)

def allanVariance(xs, T, m):
    N = len(xs)
    return sum([(xs[n + 2*m] - 2*xs[n + m] + xs[n])**2 for n in range(N - 2*m)])/(2*(m*T)**2*(N - 2*m))
        
def allanVariances(xs, T):
    variances = deque()
    averagingTimes = deque()
    for m in range(2,len(xs)//2):
        variances.append(allanVariance(xs, T, m))
        averagingTimes.append(m*T)
    return list(averagingTimes), list(variances)

def plotTransfer(output, feedback, show=False):
    plt.figure()
    plt.plot(output, feedback)
    plt.xlabel("Output (V)")
    plt.ylabel("Feedback (V)")
    plt.title("Feedback Voltage vs Output")
    if show:
        plt.show() 

def plotWaveform(xs, T, title="Voltage vs Time", show=False):
    plotWaveforms({"": xs}, T, title=title, show=show)

def plotWaveforms(waveforms, T, title="Voltage vs Time", show=False):
    plt.figure()
    for key, values in waveforms.items():
        ts = np.arange(len(values))*T
        plt.plot(ts, values, label=key)
    plt.xlabel("Time (s)")
    plt.ylabel("Voltage (V)")
    plt.title(title)
    plt.legend()
    if show:
        plt.show()

def plotSpectrum(xs, T, show=False):
    fs, amps = spectrum(xs, T)
    plt.figure()
    plt.plot(fs, amps)
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Amplitudes (V)")
    plt.yscale("log")
    plt.title("Feedback Voltage Spectrum")
    if show:
        plt.show()

def plotAllan(xs, T, show=False):
    ats, avs = allanVariances(xs, T)
    plt.figure()
    plt.plot(ats, avs)
    plt.yscale("log")
    plt.xscale("log")
    plt.xlabel("Averageing Time (s)")
    plt.ylabel(r"$\sigma^2_{OADEV}$")
    plt.title("Allan Variance")
    if show:
        plt.show()
    

if __name__ == "__main__":
    ts = np.linspace(0,1,1000)
    noise = (np.random.rand(1000) - .5)/2
    ys = .5*np.sin(10*2*np.pi*ts) + noise
    fs, amps = spectrum(ys, 1/1000)
    
    # ys = [1]*1000 + noise
    ats, avs = allanVariances(ys, 1/1000)
    plt.figure()
    plt.plot(fs, amps)
    plt.figure()
    plt.yscale("log")
    plt.xscale("log")
    plt.plot(ats, avs)
    plt.show()