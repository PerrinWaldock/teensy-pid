#include "datalogger.h"

DataLog dataLog = DataLog{LOG_OFF, 0
	#if RECORD_FEEDBACK
		,Deque<uint16_t>(INPUT_LOG_SIZE)
		#if RECORD_TIME
			,Deque<uint16_t>(INPUT_LOG_SIZE)
		#endif
	#endif
	#if RECORD_OUTPUT
		,Deque<uint16_t>(INPUT_LOG_SIZE)
		#if RECORD_TIME
			,Deque<uint16_t>(INPUT_LOG_SIZE)
		#endif
	#endif
	#if RECORD_SETPOINT
		,Deque<uint16_t>(INPUT_LOG_SIZE)
		#if RECORD_TIME
			,Deque<uint16_t>(INPUT_LOG_SIZE)
		#endif
	#endif
};

DataLog* getDataLog()
{
	return &dataLog;
}