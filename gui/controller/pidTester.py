import time
import matplotlib.pyplot as plt
import numpy as np
from tqdm import tqdm
from random import random

from .pidController import PidController
from algorithms import analysis

DEFAULT_RUNS = 0

class PidTester:
    def __init__(self, pidController: PidController):
        self.pidController = pidController
        self.pidController.kd = 0#1e-6
        self.pidController.kp = .01#.2 #.2
        self.pidController.ki = 45000#5000 #27000
        self.pidController.calibrate()
        
    def startLog(self, single=True):
        self.pidController.startLog(single=single)
        
    def getDefaultSetpoint(self):
        limits = self.pidController.getSetpointLimits()
        return np.mean(limits)
        
    def getDefaultUpperSetpoint(self):
        limits = self.pidController.getSetpointLimits()
        range = max(limits) - min(limits)
        return min(limits) + .8*range
        
    def getLog(self):
        log = self.pidController.getLog()
        times, feedbacks = log["feedback"]
        return (times, feedbacks)
    
    def getStepResponse(self, sv1=0, sv2=None):
        if sv2 is None:
            sv2 = self.getDefaultUpperSetpoint()
        self.pidController.pidActive = True
        self.pidController.sv = sv1
        time.sleep(.1)
        self.startLog(single=True)
        time.sleep(.2)
        self.pidController.sv = sv2
        log = self.getLog()
        return log
    
    def getSteadyState(self, sv=None, pidActive=True):
        if sv is None:
            sv = self.getDefaultSetpoint()
        startActive = self.pidController.pidActive
        self.pidController.sv = sv
        self.pidController.pidActive = pidActive
        time.sleep(.05)
        self.pidController.startLog(single=True)
        time.sleep(.2)
        log = self.getLog()
        self.pidController.pidActive = startActive
        return log[0][20:], log[1][20:]
    
    def plotTransfer(self, show=False):
        feedbacks, outputs = self.pidController.getFeedForwardReadings()
        analysis.plotTransfer(outputs, feedbacks, show=show)

    def plotStepResponse(self, sv=None, show=False, plot=True):
        if sv is None:
            sv = self.getDefaultUpperSetpoint()
        times, feedbacks = self.getStepResponse(sv2=sv)
        score = analysis.calculateStepResponseScore(feedbacks, self.pidController.sv)
        print("step response score:", score)
        if plot:
            analysis.plotStepResponse(feedbacks, times, sv, show=show)
        return score

    def plotStability(self, sv=None, open=True, show=False, plot=True):
        if sv is None:
            sv = self.getDefaultSetpoint()
        times, feedbacks = self.getSteadyState(sv=sv, pidActive=True)   
        T = findMedianPeriod(times)
        score = analysis.calculateStabilityScore(feedbacks, sv)
        readings = {"closed-loop": feedbacks}
        readingsWithTimes = {"closed-loop": (times, feedbacks)}
        # print(f"closed-loop normalized deviation for {sv}:", score)
        if open:
            openTimes, openFeedbacks = self.getSteadyState(sv=sv, pidActive=False)
            Topen = findMedianPeriod(openTimes)
            readings["open-loop"] = openFeedbacks
            readingsWithTimes["open-loop"] = (openTimes, openFeedbacks)
            # print(f"open-loop normalized deviation for {sv}:", analysis.calculateStabilityScore(openFeedbacks, pt.pidController.sv))
        if plot:
            print(len(times), times)
            print(len(openTimes), openTimes)
            analysis.plotWaveforms(readingsWithTimes, show=False)
            analysis.plotAllans(readingsWithTimes, show=False)
            analysis.plotSpectra({k: (t, v - sv) for k, (t,v) in readingsWithTimes.items()}, show=show)
        return score
    
def findMedianPeriod(times):
    return np.median(np.diff(times))

def calcStepResponseScore(pt, num=DEFAULT_RUNS, **kwargs):
    scores = []
    for _ in tqdm(range(num)):
        score = pt.plotStepResponse(plot=False, **kwargs)
        scores.append(score)
    return np.median(scores)

def calcStabilityScore(pt, num=DEFAULT_RUNS, **kwargs):
    scores = []
    for _ in tqdm(range(num)):
        score = pt.plotStability(sv=getRandomSetpoint(pt.pidController), open=False, plot=False, **kwargs)
        scores.append(score)
    return np.median(scores)

def getRandomSetpoint(pc: PidController):
    limits = pc.getSetpointLimits()
    setrange = (max(limits) - min(limits))
    midrange = (max(limits) + min(limits))/2
    
    return 0.75*(random() - 0.5 )*setrange + midrange

if __name__ == "__main__":
    # parser = argparse.ArgumentParser()
    # parser.add_argument("-s")
    
    pc = PidController()
    pt = PidTester(pc)
    
    pc.calibrate()
    print(f"kd={pc.kd} kp={pc.kp} ki={pc.ki}")
    print(f"Stability: {calcStabilityScore(pt)}")
    #print(f"Step Response: {calcStepResponseScore(pt)}")
        
    pt.plotTransfer(show=False)
    pt.plotStability(show=False)
    pt.plotStepResponse(show=False)
    plt.show()