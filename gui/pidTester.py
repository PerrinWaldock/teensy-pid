import time
import matplotlib.pyplot as plt
import argparse
import numpy as np

from pidController import PidController
import analysis


class PidTester:
    def __init__(self, pidController: PidController):
        self.pidController = pidController
        
    def startLog(self, single=True):
        self.pidController.startLog(single=single)
        
    def getLog(self):
        log = self.pidController.getLog()
        times, feedbacks = log["feedback"]
        return (times, feedbacks)
    
    def getStepResponse(self, sv1=0, sv2=3):
        self.pidController.pidActive = True
        self.pidController.sv = sv1
        time.sleep(.1)
        self.startLog(single=True)
        time.sleep(.05)
        self.pidController.sv = sv2
        time.sleep(.05)
        return self.getLog()
    
    def getSteadyState(self, sv=3, pidActive=True):
        startActive = self.pidController.pidActive
        self.pidController.sv = sv
        time.sleep(.01)
        self.pidController.pidActive = pidActive
        time.sleep(.01)
        self.pidController.startLog(single=True)
        time.sleep(.2)
        vals = self.getLog()
        self.pidController.pidActive = startActive
        return vals

def plotStepResponse(pt, sv=3, show=False):
    times, feedbacks = pt.getStepResponse(sv2=sv)
    T = np.mean(np.diff(times))
    analysis.plotStepResponse(feedbacks, T, sv, show=show)

def plotStability(pt, sv=3, show=False):
    times, feedbacks = pt.getSteadyState(sv=sv, pidActive=True)
    openTimes, openFeedbacks = pt.getSteadyState(sv=sv, pidActive=False)
    
    T = np.mean(np.diff(times))
    Topen = np.mean(np.diff(openTimes))
    readings = {"open-loop": openFeedbacks, "closed-loop": feedbacks}
    analysis.plotAllans(readings, T, show=False)
    analysis.plotSpectra(readings, Topen, show=show)

if __name__ == "__main__":
    # parser = argparse.ArgumentParser()
    # parser.add_argument("-s")
    
    pc = PidController()
    pt = PidTester(pc)
    
    plotStability(pt, show=False)
    plotStepResponse(pt, show=True)
