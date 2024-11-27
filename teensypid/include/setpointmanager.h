#ifndef SETPOINTMANAGER_H
#define SETPOINTMANAGER_H

#include "settings.h"

class SetpointManager
{
	public:
		SetpointManager();
		uint16_t getSetPoint();
		uint16_t getSetPoint(uint8_t index);
		void setSetPoint(uint8_t index, uint16_t value);
		uint8_t getSetPointIndex();
		uint16_t* getSetPoints();
	
	private:
		uint16_t setPoints[NUM_SETPOINTS];
};

#endif