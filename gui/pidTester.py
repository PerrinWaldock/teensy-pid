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
        time.sleep(.05)
        self.startLog(single=True)
        time.sleep(.05)
        self.pidController.sv = sv2
        time.sleep(.05)
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

def plotStepResponse(pt, sv=3, show=False):
    times, feedbacks = pt.getStepResponse(sv2=sv)
    print("step response score:", analysis.calculateStepResponseScore(feedbacks, sv))
    analysis.plotStepResponse(feedbacks, times, sv, show=show)

def plotStability(pt, sv=3, show=False):
    times, feedbacks = pt.getSteadyState(sv=sv, pidActive=True)
    openTimes, openFeedbacks = pt.getSteadyState(sv=sv, pidActive=False)
    
    T = findMedianPeriod(times)
    Topen = findMedianPeriod(openTimes)
    readings = {"open-loop": openFeedbacks, "closed-loop": feedbacks}
    print("closed-loop normalized deviation:", analysis.calculateStabilityScore(feedbacks, sv))
    print("open-loop  normalized deviation:", analysis.calculateStabilityScore(openFeedbacks, sv))
    analysis.plotAllans(readings, T, show=False)
    analysis.plotSpectra(readings, Topen, show=show)

if __name__ == "__main__":
    # parser = argparse.ArgumentParser()
    # parser.add_argument("-s")
    
    pc = PidController()
    pt = PidTester(pc)
    
    pc.calibrate()
    for _ in range(5):
        print(f"kd={pc.kd} kp={pc.kp} ki={pc.ki}")
        plotStability(pt, show=False)
        plotStepResponse(pt, show=False)
    plt.show()