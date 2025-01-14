#include "datalogger.h"

DataLog dataLog = DataLog{LOG_OFF, Deque<uint16_t>(INPUT_LOG_SIZE), Deque<uint16_t>(INPUT_LOG_SIZE)};

DataLog* getDataLog()
{
	return &dataLog;
}