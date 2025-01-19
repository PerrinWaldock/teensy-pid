import serial
import sys
import glob
import re
from collections import deque
import time
from typing import List, Tuple

import analysis

"""
look for possible usb devices (dependant on os)
provide interface to set pid constants https://docs.python.org/3/library/functions.html#
    
commands to test:
    get/set ks
    setpoints
    get feedforward data
    write/read eeprom
    enable/disable pid
    force output voltage
    get limits
    loop frequency

higher-level functions (in different file -- pidTester)
    step response
    etc

TODO try using context manager again
TODO may be valuable to record time and value as tuple
    set an elapsedTime parameter in the log struct
"""

BAUD_RATE = 1000000
TIMEOUT = .2#.05

def int2volt(x):
    return x*5.0/(2**16 - 1)

class PidController:
    def __init__(self, port: str=None, **kwargs):
        self.portname = findPort(port)
        self.lp = 1e-5
        self._svs = None
        
        self._pidActive = True
        
    def getPort(self, timeout: float=TIMEOUT) -> str:
        s = serial.Serial()
        s.port = self.portname
        s.baudrate = BAUD_RATE
        s.bytesize = serial.EIGHTBITS
        s.parity = serial.PARITY_NONE
        s.stopbits = serial.STOPBITS_ONE
        s.timeout = timeout
        s.write_timeout = timeout
        s.open()
        return s
        
    def sendCommand(self, command: str) -> None:
        with self.getPort() as s:
            s.write((command + "\n").encode('utf_8'))
            
    def sendCommands(self, commands: List[str]) -> None:
        with self.getPort() as s:
            for command in commands:
                s.write((command + "\n").encode('utf_8'))
    
    def readLine(self, timeout: float=TIMEOUT) -> str:
        with self.getPort(timeout=timeout) as s:
            line = s.readline()
        return line.decode('utf_8')
    
    def readLines(self, timeout: float=TIMEOUT) -> List[str]: #TODO test context manager
        with self.getPort(timeout=timeout) as s:
            lines = s.readlines()
        return [l.decode('utf_8') for l in lines]
    
    def elicitResponse(self, command: str, timeout: float=TIMEOUT) -> str:
        with self.getPort(timeout=timeout) as s:
            s.write((command + "\n").encode('utf_8'))
            line = s.readline()
        return line.decode('utf_8').strip()
    
    def elicitResponses(self, command: str, timeout: float=TIMEOUT) -> List[str]:
        with self.getPort(timeout=timeout) as s:
            s.write((command + "\n").encode('utf_8'))
            lines = s.readlines()
        return [l.decode('utf_8').strip() for l in lines]
    
    def sendCommandExpectingSameResponse(self, command: str, timeout: float=TIMEOUT) -> None:
        response = self.elicitResponse(command, timeout=timeout)
        if not command in response:
            raise Exception(f'{command} != {response}')
    
    def getFloat(self, token: str) -> float:
        command = f"{token}?"
        response = self.elicitResponse(command)
        m = re.search(r'(?<=' + token + r'=)\d*.?\d*', response)
        return float(m.group())
            
    def startLog(self, single: bool=False) -> None:
        self.sendCommandExpectingSameResponse(f"lg={'s' if single else 'c'}") #TODO check for response
        
    def getLog(self):
        self.sendCommandExpectingSameResponse("lg=o")
        lines = self.elicitResponses("lg?")
        feedbacks = deque()
        feedbackTimes = deque()
        outputs = deque()
        outputTimes = deque()
        setpoints = deque()
        setpointTimes = deque()
        for line in lines:
            feedback = re.search(r'(?<=f\:\s)\d+', line)
            feedbackTime = re.search(r'(?<=ft\:\s)\d+', line)
            output = re.search(r'(?<=o\:\s)\d+', line)
            outputTime = re.search(r'(?<=ot\:\s)\d+', line)
            setpoint = re.search(r'(?<=s\:\s)\d+', line)
            setpointTime = re.search(r'(?<=st\:\s)\d+', line)
            if feedback and feedbackTime:
                feedbacks.append(int2volt(int(feedback.group())))
                feedbackTimes.append(int(feedbackTime.group()))
            if output and outputTime:
                outputs.append(int2volt(int(output.group())))
                outputTimes.append(int(outputTime.group()))
            if setpoint and setpointTime:
                setpoints.append(int2volt(int(setpoint.group())))
                setpointTimes.append(int(setpointTime.group()))
                
        retdict = {}
        if len(feedbacks) > 0:
            retdict["feedback"] = (feedbackTimes, feedbacks)
        if len(feedbacks) > 0:
            retdict["setpoint"] = (setpointTimes, setpoints)
        if len(feedbacks) > 0:
            retdict["output"] = (outputTimes, outputs)
        return retdict
    
    def calibrate(self) -> None:
        response = self.elicitResponse("cf", timeout=1)
        if response != "Calibrated!":
            raise Exception(f"Calibration response: {response}")
        
    def getFeedForwardReadings(self) -> Tuple[List[float]]:
        lines = self.elicitResponses("ff?")
        feedbacks = deque()
        outputs = deque()
        for line in lines:
            m = re.search(r'(?<=ff\s)(\d+)\:\s(\d+)', line)
            outputs.append(int2volt(float(m.group(1))))
            feedbacks.append(int2volt(float(m.group(2))))
        return feedbacks, outputs
    
    def save(self) -> None:
        self.sendCommandExpectingSameResponse("ew=w")
        
    def load(self) -> None:
        self.sendCommandExpectingSameResponse("ew=r")
        
    def forceOutput(self, voltage: float) -> None:
        self.sendCommandExpectingSameResponse(f"ov={voltage}")
    
    def getSetpointLimits(self) -> Tuple[float]:
        response = self.elicitResponse("ls?")
        matches = re.findall(r'(?<=\=)\d*\.?\d*(?=\sV)', response)
        return tuple([float(m) for m in matches])
    
    @property
    def ki(self):
        return self.getFloat("ki")
    
    @ki.setter
    def ki(self, value: float):
        command = f"ki={value}"
        self.sendCommandExpectingSameResponse(command)
    
    @property
    def kp(self):
        return self.getFloat("kp")
    
    @kp.setter
    def kp(self, value: float):
        command = f"kp={value}"
        self.sendCommandExpectingSameResponse(command)
    
    @property
    def kd(self):
        return self.getFloat("kd")
    
    @kd.setter
    def kd(self, value: float):
        command = f"kd={value}"
        self.sendCommandExpectingSameResponse(command)
    
    @property
    def loopFrequency(self):
        return self.getFloat("lf")
    
    @loopFrequency.setter
    def loopFrequency(self, value: float):
        command = f"lf={value}"
        self.sendCommandExpectingSameResponse(command)
    
    @property
    def sv(self):
        return self._svs[self._svind]
    
    @sv.setter
    def sv(self, value: float):
        responses = self.elicitResponses(f"sv={value}")
        self.refreshSetpoints(responses)
        
    @property 
    def svs(self):
        self.refreshSetpoints()
        return self._svs
    
    @svs.setter
    def svs(self, value: float):
        updateSetpoints(self, value)
    
    def refreshSetpoints(self, responses: List[str]=None):
        if responses is None:
            responses = self.elicitResponses(f"sv?")
        self._svind = getActiveSetpoint(responses)
        self._svs = updateSetpoints(self._svs, responses)
        
    def updateSetpoints(self, setpoints: List[float]):
        for ind, sv in enumerate(setpoints):
            responses = self.elicitResponses(f"sv{ind}={sv}")
        self.refreshSetpoints(responses)
        if tuple(setpoints) != tuple(self._svs):
            raise Exception(f"setpoints not set to expected: {tuple(setpoints)} != {tuple(self._svs)}")
    
    @property
    def pidActive(self):
        return int(self.getFloat("pa")) != 0
    
    @pidActive.setter
    def pidActive(self, value):
        command = f"pa={1 if value else 0}"
        self.sendCommandExpectingSameResponse(command)
        

def parseSetpoints(lines: List[str]):
    setpoints = {}
    for line in lines:
        matches = re.search(r'(?<=sv)(\d+)\=(\d*\.?\d*)', line)
        setpoints[int(matches.group(1))] = float(matches.group(2))
    return setpoints

def getActiveSetpoint(lines: List[str]):
    return [ind for ind, line in enumerate(lines) if "<-" in line][0]

def updateSetpoints(setpoints, lines: List[str]):
    newsetpoints = parseSetpoints(lines)
    if setpoints != None:
        setpoints = list(setpoints)
    else:
        setpoints = [0]*len(newsetpoints)
    for key, value in newsetpoints.items():
        setpoints[key] = value
    return tuple(setpoints)

def findPort(ports: List[str]=None):
    if ports is None:
        ports = serial_ports()
    for port in ports:
        try:
            with serial.Serial(port, BAUD_RATE, timeout=TIMEOUT, write_timeout=TIMEOUT) as s:
                testline = "nm?\n"
                expectedResponse = "nm=fpidController"
                s.write(testline.encode('utf_8'))
                response = s.readline().decode('utf_8')
                if response.strip() == expectedResponse.strip():
                    return port
        except (OSError, serial.SerialException) as e:
            print(e)
            pass

# https://stackoverflow.com/questions/12090503/listing-available-com-ports-with-python
def serial_ports():
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = [p for p in glob.glob('/dev/tty.*') if "usb" in p]
    else:
        raise EnvironmentError('Unsupported platform')

    return ports

def test(pc: PidController):
    # TODO tests
    
    pc.load()
    pc.calibrate()
    feedbacks, outputs = pc.getFeedForwardReadings() #TODO plot
    analysis.plotTransfer(outputs, feedbacks)

    low, high = pc.getSetpointLimits() #TODO check
    print("setpoint limits:", low, high)
    pc.forceOutput(3)
    print("pid constants:", pc.kd, pc.kp, pc.ki)
    print("frequency:", pc.loopFrequency)
    # TODO check setting
    setpoint = 3
    print("setpoints:", pc.svs, pc.sv == setpoint)
    pc.pidActive = True
    
    pc.startLog()
    time.sleep(.5)
    feedbacks, outputs = pc.getLog()
    T = 1/pc.loopFrequency
    analysis.plotWaveforms({"feedback": feedbacks, "output": outputs}, T, show=True)
    

if __name__ == "__main__":
    # parser = argparse.ArgumentParser()
    # parser.add_argument("-s")
    
    pc = PidController()
    test(pc)