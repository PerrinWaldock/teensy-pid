#ifndef DATAMANGER_H
#define DATAMANGER_H

#include "deque.h"
#include "settings.h"

typedef enum{
	LOG_OFF, //don't log output
	LOG_SINGLE, //log until full
	LOG_CONTINUOUS	//continue logging, overwriting old data
} LogState;

#if TEENSY == TEENSY40
	const int32_t TOTAL_LOG_SPACE = 200000;
#elif TEENSY == TEENSY32
	const int32_t TOTAL_LOG_SPACE = 1000;
#endif

typedef struct {
	LogState state;
	elapsedMicros sinceLogStart;
	#if RECORD_FEEDBACK
		Deque<uint16_t> feedback;
		#if RECORD_TIME
			Deque<uint16_t> feedbackTime;
		#endif
	#endif
	#if RECORD_OUTPUT
		Deque<uint16_t> output;
		#if RECORD_TIME
			Deque<uint16_t> outputTime;
		#endif
	#endif
	#if RECORD_SETPOINT
		Deque<uint16_t> setpoint;
		#if RECORD_TIME
			Deque<uint16_t> setpointTime;
		#endif
	#endif
} DataLog;

const uint8_t UINT16_IN_DATALOG = (RECORD_FEEDBACK + RECORD_OUTPUT + RECORD_SETPOINT)*(RECORD_TIME + 1);
const int32_t INPUT_LOG_SIZE = TOTAL_LOG_SPACE/UINT16_IN_DATALOG/sizeof(uint16_t);

DataLog* getDataLog();

#endif