import time
import matplotlib.pyplot as plt
import argparse
import numpy as np

from pidController import PidController
import analysis


class PidTester:
    def __init__(self, pidController: PidController):
        self.pidController = pidController
        self.pidController.kp = .05
        self.pidController.ki = 10000
        
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
    score = analysis.calculateStepResponseScore(feedbacks, sv)
    print("step response score:", score)
    if plot:
        analysis.plotStepResponse(feedbacks, times, sv, show=show)
    return score

def plotStability(pt, sv=3, open=True, show=False, plot=True):
    times, feedbacks = pt.getSteadyState(sv=sv, pidActive=True)   
    T = findMedianPeriod(times)
    score = analysis.calculateStabilityScore(feedbacks, sv)
    readings = {"closed-loop": feedbacks}
    print("closed-loop normalized deviation:", score)
    if open:
        openTimes, openFeedbacks = pt.getSteadyState(sv=sv, pidActive=False)
        Topen = findMedianPeriod(openTimes)
        readings["open-loop"] = openFeedbacks
        print("open-loop  normalized deviation:", analysis.calculateStabilityScore(openFeedbacks, sv))
    if plot:
       analysis.plotAllans(readings, T, show=False)
       analysis.plotSpectra(readings, Topen, show=show)
    return score

def calcStepResponseScore(pt, num=10, **kwargs):
    scores = []
    for _ in range(num):
        score = plotStepResponse(pt, plot=False, **kwargs)
        scores.append(score)
    return np.mean(scores)

def calcStabilityScore(pt, num=10, **kwargs):
    scores = []
    for _ in range(num):
        score = plotStability(pt, open=False, plot=False, **kwargs)
        scores.append(score)
    return np.mean(scores)

if __name__ == "__main__":
    # parser = argparse.ArgumentParser()
    # parser.add_argument("-s")
    
    pc = PidController()
    pt = PidTester(pc)
    
    pc.calibrate()
    print(f"kd={pc.kd} kp={pc.kp} ki={pc.ki}")
    print(f"Stability: {calcStabilityScore(pt)}")
    print(f"Step Response: {calcStepResponseScore(pt)}")
        
    plotStability(pt, show=False)
    plotStepResponse(pt, show=False)
    plt.show()