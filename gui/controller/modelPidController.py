"""
TODO
	make this class store a history of events. Generate responses based on inputs received
	need to think about how the internal model is stored
"""
from collections import deque, namedtuple
from datetime import datetime, timedelta
from types import List, Tuple, Dict, Func

LogEntry = namedtuple('LogEntry', ('time', 'event'))

class ModelPidController:
    def __init__(self, period: timedelta, model: Func[List[float], List[float]]):
        self._eventQueue = deque()
        self._period = period
        self._inputs: Dict[datetime, float] = dict()
        self._outputs: Dict[datetime, float] = dict()
        self._setpoints: Dict[datetime, float] = dict()
        
    def _addEntry(self, event: str):
        self._eventQueue.append(LogEntry(datetime.now(), event))
        
    def _generateData(self, ):
        pass
    
    def startLog(self):
        self._addEntry("start")
    
    def getLog(self):
        self._addEntry("get")
        #TODO extraction logic
    
    