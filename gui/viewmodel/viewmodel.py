import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from controller import PidTester, PidController

"""
two approaches:
	any time something changes, make sure to update and refresh
	set up something event-based, make upper objects subscribe to other object events
"""

class ViewModel:
    def __init__(self, pc: PidController):
        self.pc = pc
        self.pt = PidTester(pc)
    
    @property
    def kp(self):
        return self.pc.kp
    
    @kp.setter
    def kp(self, value: float):
        self.pc.kp = value
    
    @property
    def ki(self):
        return self.pc.ki
    
    @ki.setter
    def ki(self, value: float):
        self.pc.ki = value
    
    @property
    def kd(self):
        return self.pc.kd
    
    @kd.setter
    def kd(self, value: float):
        self.pc.kd = value
        
    @property
    def loopFrequency(self):
        return self.pc.loopFrequency
    
    @loopFrequency.setter
    def loopFrequency(self, value: float):
        self.pc.loopFrequency = value
        
    @property
    def setPoints(self):
        return self.pc.svs
    
    @setPoints.setter
    def setPoints(self, values: tuple[float]):
        self.pc.svs = values
        
    @property
    def pidActive(self):
        return self.pc.pidActive
    
    @pidActive.setter
    def pidActive(self, value: bool):
        self.pc.pidActive = value
        
    @property
    def activeSetPoint(self):
        return self.pc.svind
    
    def calibrate(self):
        self.pc.calibrate()
        
    def saveParameters(self):
        self.pc.save()
        
    def loadParameters(self):
        self.pc.load()
    
    def transferPlot(self):
        self.pt.plotTransfer(show=True) #TODO make nonblocking (plt.ion?)
    
    def stepResponse(self):
        #self.pt.getStepResponse(sv1=self.setPoints[0], sv2=self.setPoints[1])
        self.pt.plotStepResponse(show=True) #TODO make nonblocking
    
    def stabilityPlot(self):
        self.pt.plotStability(sv=self.pc.sv, show=True) #TODO make nonblocking