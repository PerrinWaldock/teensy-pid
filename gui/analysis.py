import numpy as np
from collections import deque
import matplotlib.pyplot as plt
from utils import *

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

def plotWaveform(xs, ts, title="Voltage vs Time", show=False):
    plotWaveforms({"": (ts, xs)}, title=title, show=show)

def plotWaveforms(waveforms, title="Voltage vs Time", show=False):
    plt.figure()
    for key, values in waveforms.items():
        ts = values[0]    
        vs = values[1]
        if not isIterable(ts):
            ts = np.arange(len(vs))*ts
        plt.plot(ts, vs, label=key)
    plt.xlabel("Time (s)")
    plt.ylabel("Voltage (V)")
    plt.title(title)
    plt.legend()
    if show:
        plt.show()

def plotSpectrum(xs, T, show=False):
    plotSpectra({"": xs}, T, show=show)

def plotSpectra(waveforms, T, show=False):
    plt.figure()
    for key, xs in waveforms.items():
        fs, amps = spectrum(xs, T)
        plt.plot(fs, amps, label=key)
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Amplitudes (V)")
    plt.yscale("log")
    plt.title("Feedback Voltage Spectrum")
    plt.legend()
    if show:
        plt.show()

def plotAllan(xs, T, show=False):
    plotAllans({"": xs}, T, show=show)

def plotAllans(waveforms, T, show=False):
    plt.figure()
    for key, xs in waveforms.items():
        ats, avs = allanVariances(xs, T)
        plt.plot(ats, avs, label=key)
    plt.yscale("log")
    plt.xscale("log")
    plt.xlabel("Averaging Time (s)")
    plt.ylabel(r"$\sigma^2_{OADEV}$")
    plt.title("Allan Variance")
    plt.legend()
    if show:
        plt.show()

def findCrossings(xs):
    mid = (min(xs) + max(xs))/2
    return np.where(np.diff(np.sign(xs - mid)))[0]

def findFirstNtrue(xs, n):
    last = 0
    for ind, val in enumerate(xs):
        if val:
            if (ind - last) > n:
                return last
        else:
            last = ind

#TODO need better criteria
def findSettledInd(xs, desired, consecutivePointsInStdevIsSettled=3):
    crossings = findCrossings(xs)
    if len(crossings) > 1:
        xs = xs[:crossings[1]]
    stepInd = crossings[0]
    settleData = xs[stepInd+1:]
    settledStdev = np.std(settleData[len(settleData)//2:])
    settledInd = findFirstNtrue((settleData < desired + settledStdev)*(settleData > desired - settledStdev), consecutivePointsInStdevIsSettled) + stepInd
    return settledInd

def expectedResponse(stepInd, desired, n):
    inds = np.arange(n)
    return (inds > stepInd)*desired

def calculateStabilityScore(xs, desired):
    xs = np.array(xs)
    return np.sqrt(np.sum((xs/desired - 1)**2))

def calculateStepResponseScore(xs, desired, consecutivePointsInStdevIsSettled=3):
    crossings = findCrossings(xs)
    if len(crossings) > 1:
        xs = xs[:crossings[1]]
    stepInd = crossings[0]
    settledInd = findSettledInd(xs, desired, consecutivePointsInStdevIsSettled=consecutivePointsInStdevIsSettled)
    expected = expectedResponse(stepInd, desired, len(xs))
    return np.sqrt(np.sum(((xs[stepInd:settledInd] - expected[stepInd:settledInd])/desired)**2))

def plotStepResponse(xs, ts, desired, show=False):
    xs = np.array(xs)
    ts = np.array(ts)
    crossings = findCrossings(xs)
    if len(crossings) > 1:
        xs = xs[:crossings[1]]
    stepInd = crossings[0]
    settledInd = findSettledInd(xs, desired)
    bufferInds = int(.1*(settledInd - stepInd))
    lastInd = settledInd + bufferInds
    startInd = stepInd - bufferInds
    expected = expectedResponse(stepInd, desired, len(xs))
    
    score = calculateStepResponseScore(xs, desired)
    plotWaveforms({"desired": (ts[startInd:lastInd], expected[startInd:lastInd]), "actual": (ts[startInd:lastInd], xs[startInd:lastInd])}, title=f"Step Response (score: {score})", show=show)
    
    #TODO find step time
    #find settle time
    #compute a score (stdev from desired)
    

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