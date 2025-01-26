import time
import matplotlib.pyplot as plt
import argparse
import numpy as np
from tqdm import tqdm
from random import random

from pidController import PidController
import analysis

DEFAULT_RUNS = 25

class PidTester:
    def __init__(self, pidController: PidController):
        self.pidController = pidController
        self.pidController.kp = .2
        self.pidController.ki = 27000
        
    def startLog(self, single=True):
        self.pidController.startLog(single=single)
        
    def getLog(self):
        log = self.pidController.getLog()
        times, feedbacks = log["feedback"]
        return (times, feedbacks)
    
    def getStepResponse(self, sv1=0, sv2=3):
        self.pidController.pidActive = True
        self.pidController.sv = sv1
        self.startLog(single=True)
        time.sleep(.05)
        self.pidController.sv = sv2
        return self.getLog()
    
    def getSteadyState(self, sv=3, pidActive=True):
        startActive = self.pidController.pidActive
        self.pidController.sv = sv
        self.pidController.pidActive = pidActive
        time.sleep(.01)
        self.pidController.startLog(single=True)
        time.sleep(.5)
        vals = self.getLog()
        self.pidController.pidActive = startActive
        return vals
    
def findMedianPeriod(times):
    return np.median(np.diff(times))

def plotStepResponse(pt, sv=3, show=False, plot=True):
    times, feedbacks = pt.getStepResponse(sv2=sv)
    score = analysis.calculateStepResponseScore(feedbacks, pt.pidController.sv)
    print("step response score:", score)
    if plot:
        analysis.plotStepResponse(feedbacks, times, sv, show=show)
    return score

def plotStability(pt, sv=3, open=True, show=False, plot=True):
    times, feedbacks = pt.getSteadyState(sv=sv, pidActive=True)   
    T = findMedianPeriod(times)
    score = analysis.calculateStabilityScore(feedbacks, sv)
    readings = {"closed-loop": feedbacks}
    # print(f"closed-loop normalized deviation for {sv}:", score)
    if open:
        openTimes, openFeedbacks = pt.getSteadyState(sv=sv, pidActive=False)
        Topen = findMedianPeriod(openTimes)
        readings["open-loop"] = openFeedbacks
        # print(f"open-loop normalized deviation for {sv}:", analysis.calculateStabilityScore(openFeedbacks, pt.pidController.sv))
    if plot:
       analysis.plotAllans(readings, T, show=False)
       analysis.plotSpectra(readings, Topen, show=show)
    return score

def calcStepResponseScore(pt, num=DEFAULT_RUNS, **kwargs):
    scores = []
    for _ in tqdm(range(num)):
        score = plotStepResponse(pt, plot=False, **kwargs)
        scores.append(score)
    return np.median(scores)

def calcStabilityScore(pt, num=DEFAULT_RUNS, **kwargs):
    scores = []
    for _ in tqdm(range(num)):
        score = plotStability(pt, sv=getRandomSetpoint(pt.pidController), open=False, plot=False, **kwargs)
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
        
    plotStability(pt, show=False)
    plotStepResponse(pt, show=False)
    plt.show()