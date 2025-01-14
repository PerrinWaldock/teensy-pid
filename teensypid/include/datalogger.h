#ifndef DATAMANGER_H
#define DATAMANGER_H

//#include <CircularBuffer.hpp> //prepackaged Circular Buffer
#include "deque.h"

typedef enum{
	LOG_OFF, //don't log output
	LOG_SINGLE, //log until full
	LOG_CONTINUOUS	//continue logging, overwriting old data
} LogState;

#if TEENSY == TEENSY40
	const int32_t TOTAL_LOG_SPACE = 100000;
#elif TEENSY == TEENSY32
	const int32_t TOTAL_LOG_SPACE = 1000;
#endif
const int32_t INPUT_LOG_SIZE = TOTAL_LOG_SPACE/2/sizeof(uint16_t);

typedef struct {
	LogState state;
	Deque<uint16_t> input;
	Deque<uint16_t> output;
} DataLog;

DataLog* getDataLog();

#endif