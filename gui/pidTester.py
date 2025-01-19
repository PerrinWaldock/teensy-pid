import time
import matplotlib.pyplot as plt
import argparse
import numpy as np

from pidController import PidController
import analysis


class PidTester:
    def __init__(self, pidController):
        self.pidController = pidController
        
    def startLog(self):
        self.pidController.startLog(single=True)
        
    def getLog(self):
        (feedbacks, outputs) = self.pidController.getLog()
        times = np.linspace(0,len(feedbacks))*(1/self.pidController.loopFrequency)
        return (times, feedbacks)
    
    def getStepResponse(self, sv1=0, sv2=3):
        self.pidController.pidActive = True
        self.pidController.sv = sv1
        print("starting")
        self.startLog()
        time.sleep(.01)
        self.pidController.sv = sv2
        time.sleep(.01)
        return self.getLog()
    
    def getSteadyState(self, sv=3, pidActive=True):
        self.pidController.sv = sv
        time.sleep(.01)
        self.pidController.pidActive = pidActive
        time.sleep(.01)
        self.pidController.startLog(single=True)
        time.sleep(.1)
        return self.getLog()
        

def plotStepResponse(pt):
    times, feedbacks = pt.getStepResponse()
    plt.plot(times, feedbacks)
    plt.xlabel("Time (s)")
    plt.ylabel("Voltage (V)")
    plt.show()



def plotStability(pt, sv=3):
    times, feedbacks = pt.getSteadyState(sv=sv)
    T = np.mean(np.diff(times))
    analysis.plotAllan(feedbacks, T)
    analysis.plotSpectrum(feedbacks, T)
    plt.show()

if __name__ == "__main__":
    # parser = argparse.ArgumentParser()
    # parser.add_argument("-s")
    
    pc = PidController()
    pt = PidTester(pc)
    
    plotStepResponse(pt)
    plt.show()