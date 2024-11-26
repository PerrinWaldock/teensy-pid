#include "commands.h"
#include "commandregistrar.h"
#include "utils.hpp"
#include "serialmanager.h"

#include <string.h>

#define TEMPORARY_BUFFER_LENGTH 128

#define SET_TOKEN "="
#define GET_TOKEN "?"

#define KP_TOKEN "kp"
#define KI_TOKEN "ki"
#define KD_TOKEN "kd"
#define KS_TOKEN "ks"
#define LOOP_RATE_TOKEN "lf"
#define SET_POINT_MIN_TOKEN "ls"
#define SET_POINT_MAX_TOKEN "hs"
//#define SET_POINT_LIMITS_TOKEN "sl"
#define OUTPUT_MIN_TOKEN "lo"
#define OUTPUT_MAX_TOKEN "ho"
//#define OUTPUT_LIMITS_TOKEN "ol"
#define SET_POINT_INT_TOKEN "sp"
#define SET_POINT_FLOAT_TOKEN "sv"
#define READ_AVERAGES_TOKEN "ra"

#define PID_ENABLE_TOKEN "pa"
#define PRINT_OUTPUT_TOKEN "po"
#define CALIBRATE_TOKEN "cf"
#define EEPROM_TOKEN "ew"
#define RESET_TOKEN "cl"
#define SET_OUTPUT_TOKEN "ov"

#define EEPROM_WRITE_TOKEN "w"
#define EEPROM_READ_TOKEN "r"

#define NUMBER_OF_SETPOINTS INPUT_STATES

void setKp(void*, char*);
void getKp(void*, char*);
void setKi(void*, char*);
void getKi(void*, char*);
void setKd(void*, char*);
void getKd(void*, char*);
void getKs(void*, char*);
void setLoopRate(void*, char*);
void getLoopRate(void*, char*);
void setSetPointMax(void*, char*);
void setSetPointMin(void*, char*);
void getSetPointLimits(void*, char*);
void setOutputMax(void*, char*);
void setOutputMin(void*, char*);
void getOutputLimits(void*, char*);

void setPointIntHandler(void*, char*);
void setPointFloatHandler(void*, char*);
void setSetPointInt(void*, char*); //<SP TOKEN> -> parse command
void setSetPointFloat(void*, char*);
void getSetPointsInt(void*, char*); //<SP TOKEN> <QUERY>
void getSetPointsFloat(void*, char*);

void setReadAverages(void*, char*);
void getReadAverages(void*, char*);

void setOutput(void*, char*);
void getOutput(void*, char*);
void setPidEnable(void*, char*);
void getPidEnable(void*, char*);

void setPrintOutput(void*, char*);
void calibrateFeedForward(void*, char*);
void eepromReadWrite(void*, char*);
void resetPidState(void*, char*);

void checkConfigError(FPid*);


CommandParser::CommandParser(FPid& pidController, EepromManager& eepromManager, SetpointManager& setpointManager, uint8_t& readAveragesPower, bool& printOutput)
{
	this->pidController = &pidController;
	this->eepromManager = &eepromManager;
	this->setpointManager = &setpointManager;
	this->readAveragesPower = &readAveragesPower;
	this->printOutput = &printOutput;

	addCommand(KP_TOKEN SET_TOKEN, setKp, (void*) &pidController, "sets kp (float)");
	addCommand(KP_TOKEN GET_TOKEN, getKp, (void*) &pidController, "get kp (float)");
	addCommand(KI_TOKEN SET_TOKEN, setKi, (void*) &pidController, "sets ki (float)");
	addCommand(KI_TOKEN GET_TOKEN, getKi, (void*) &pidController, "get ki (float)");
	addCommand(KD_TOKEN SET_TOKEN, setKd, (void*) &pidController, "sets kd (float)");
	addCommand(KD_TOKEN GET_TOKEN, getKd, (void*) &pidController, "get kd (float)");
	addCommand(KS_TOKEN GET_TOKEN, getKs, (void*) &pidController, "gets all PID constants");

	addCommand(LOOP_RATE_TOKEN SET_TOKEN, setLoopRate, (void*) &pidController, "sets loop rate (Hz)");
	addCommand(LOOP_RATE_TOKEN GET_TOKEN, getLoopRate, (void*) &pidController, "gets loop rate (Hz)");

	addCommand(SET_POINT_MAX_TOKEN SET_TOKEN, setSetPointMax, (void*) &pidController, "sets maximum set point (V)");
	addCommand(SET_POINT_MIN_TOKEN SET_TOKEN, setSetPointMin, (void*) &pidController, "sets minimum set point (V)");
	addCommand(SET_POINT_MAX_TOKEN GET_TOKEN, getSetPointLimits, (void*) &pidController, "gets set point limits (V)");
	addCommand(SET_POINT_MIN_TOKEN GET_TOKEN, getSetPointLimits, (void*) &pidController, "gets set point limits (V)");

	addCommand(OUTPUT_MAX_TOKEN SET_TOKEN, setOutputMax, (void*) &pidController, "sets maximum output (V)");
	addCommand(OUTPUT_MIN_TOKEN SET_TOKEN, setOutputMin, (void*) &pidController, "sets minimum output (V)");
	addCommand(OUTPUT_MAX_TOKEN GET_TOKEN, getOutputLimits, (void*) &pidController, "gets output limits (V)");
	addCommand(OUTPUT_MIN_TOKEN GET_TOKEN, getOutputLimits, (void*) &pidController, "gets output limits (V)");

	addCommand(SET_POINT_INT_TOKEN, setPointIntHandler, (void*) &setpointManager, SET_POINT_INT_TOKEN "X interacts with the Xth set point's integer value (e.g. " SET_POINT_INT_TOKEN "0" SET_TOKEN "0 sets the 1st set point to 0; " SET_POINT_INT_TOKEN "1" GET_TOKEN " gets the 2nd set point)");
	addCommand(SET_POINT_INT_TOKEN GET_TOKEN, getSetPointsInt, (void*) &setpointManager, "gets all set point values as integers");
	addCommand(SET_POINT_FLOAT_TOKEN, setPointFloatHandler, (void*) &setpointManager, SET_POINT_FLOAT_TOKEN "X interacts with the Xth set point's voltage (e.g. " SET_POINT_FLOAT_TOKEN "1" SET_TOKEN "0 sets the 1st set point to 1 V; " SET_POINT_INT_TOKEN "1" GET_TOKEN " gets the 2nd set point voltage)");
	addCommand(SET_POINT_FLOAT_TOKEN GET_TOKEN, getSetPointsFloat, (void*) &setpointManager, "gets all set point values as voltages");

	addCommand(READ_AVERAGES_TOKEN SET_TOKEN, setReadAverages, (void*) &readAveragesPower, "sets number of read averages (must be a power of 2)");
	addCommand(READ_AVERAGES_TOKEN GET_TOKEN, getReadAverages, (void*) &readAveragesPower, "gets number of read averages");

	addCommand(CALIBRATE_TOKEN, calibrateFeedForward, (void*) &pidController, "calibrates feedforward lookup table");

	addCommand(RESET_TOKEN, resetPidState, (void*) &pidController, "resets PID state");

	addCommand(EEPROM_TOKEN, eepromReadWrite, (void*) &eepromManager, EEPROM_READ_TOKEN " to read parameters from EEPROM; " EEPROM_WRITE_TOKEN " to write parameters to EEPROM");

	addCommand(PRINT_OUTPUT_TOKEN SET_TOKEN, setPrintOutput, (void*) &printOutput, "1 to print output, 0 to disable print output");
}

void setKp(void* arg, char* s)
{
	if (arg == NULL || s == NULL )
	{
		return;
	}
	PidParams params = ((FPid*)arg)->getParams();
	params.kp = atof(s);
	((FPid*)arg)->setParams(params);
	checkConfigError((FPid*)arg);
	getKp(arg, s);
}

void getKp(void* arg, char* s)
{
	char printBuffer[TEMPORARY_BUFFER_LENGTH];
	if (arg == NULL || s == NULL )
	{
		return;
	}
	PidParams params = ((FPid*)arg)->getParams();
	snprintf(printBuffer, TEMPORARY_BUFFER_LENGTH, KP_TOKEN SET_TOKEN "%f", params.kp);
	writeLine(printBuffer);
}

//todo fix i, d
void setKi(void* arg, char* s)
{
	if (arg == NULL || s == NULL )
	{
		return;
	}
	PidParams params = ((FPid*)arg)->getParams();
	params.ki = atof(s);
	((FPid*)arg)->setParams(params);
	checkConfigError((FPid*)arg);
	getKi(arg, s);
}

void getKi(void* arg, char* s)
{
	char printBuffer[TEMPORARY_BUFFER_LENGTH];
	if (arg == NULL || s == NULL )
	{
		return;
	}
	PidParams params = ((FPid*)arg)->getParams();
	snprintf(printBuffer, TEMPORARY_BUFFER_LENGTH, KI_TOKEN SET_TOKEN "%f", params.ki);
	writeLine(printBuffer);
}

void setKd(void* arg, char* s)
{
	if (arg == NULL || s == NULL )
	{
		return;
	}
	PidParams params = ((FPid*)arg)->getParams();
	params.kd = atof(s);
	((FPid*)arg)->setParams(params);
	checkConfigError((FPid*)arg);
	getKp(arg, s);
}

void getKd(void* arg, char* s)
{
	char printBuffer[TEMPORARY_BUFFER_LENGTH];
	if (arg == NULL || s == NULL )
	{
		return;
	}
	PidParams params = ((FPid*)arg)->getParams();
	snprintf(printBuffer, TEMPORARY_BUFFER_LENGTH, KD_TOKEN SET_TOKEN "%f", params.kd);
	writeLine(printBuffer);
}

void getKs(void* arg, char* s)
{
	char printBuffer[TEMPORARY_BUFFER_LENGTH];
	if (arg == NULL || s == NULL )
	{
		return;
	}
	PidParams params = ((FPid*)arg)->getParams();
	snprintf(printBuffer, TEMPORARY_BUFFER_LENGTH, KD_TOKEN SET_TOKEN "%f, " KP_TOKEN SET_TOKEN "%f, " KI_TOKEN SET_TOKEN "%f", params.kd, params.kp, params.ki);
	writeLine(printBuffer);
}

void setLoopRate(void* arg, char* s)
{
	if (arg == NULL || s == NULL )
	{
		return;
	}
	PidParams params = ((FPid*)arg)->getParams();
	params.loopPeriod_us = (uint32_t)(1000000/atof(s));
	((FPid*)arg)->setParams(params);
	checkConfigError((FPid*)arg);
	getKp(arg, s);
}

void getLoopRate(void* arg, char* s)
{
	char printBuffer[TEMPORARY_BUFFER_LENGTH];
	if (arg == NULL || s == NULL )
	{
		return;
	}
	PidParams params = ((FPid*)arg)->getParams();
	snprintf(printBuffer, TEMPORARY_BUFFER_LENGTH, LOOP_RATE_TOKEN SET_TOKEN "%f Hz", 1000000.0/params.loopPeriod_us);
	writeLine(printBuffer);
}

void setSetPointMax(void* arg, char* s)
{
	if (arg == NULL || s == NULL )
	{
		return;
	}
	PidParams params = ((FPid*)arg)->getParams();
	params.setPointLimit.max = inputVolts2int(atof(s));
	((FPid*)arg)->setParams(params);
	getSetPointLimits(arg, s);
}

void setSetPointMin(void* arg, char* s)
{
	if (arg == NULL || s == NULL )
	{
		return;
	}
	PidParams params = ((FPid*)arg)->getParams();
	params.setPointLimit.min = inputVolts2int(atof(s));
	((FPid*)arg)->setParams(params);
	getSetPointLimits(arg, s);
}

void getSetPointLimits(void* arg, char* s)
{
	char printBuffer[TEMPORARY_BUFFER_LENGTH];
	if (arg == NULL || s == NULL )
	{
		return;
	}
	PidParams params = ((FPid*)arg)->getParams();
	snprintf(printBuffer, TEMPORARY_BUFFER_LENGTH, SET_POINT_MIN_TOKEN SET_TOKEN "%f V, " SET_POINT_MAX_TOKEN SET_TOKEN "%f V", int2inputVolts(params.setPointLimit.min), int2inputVolts(params.setPointLimit.max));
	writeLine(printBuffer);
}

void setOutputMax(void* arg, char* s)
{
	if (arg == NULL || s == NULL )
	{
		return;
	}
	PidParams params = ((FPid*)arg)->getParams();
	params.outputLimit.max = inputVolts2int(atof(s));
	((FPid*)arg)->setParams(params);
	getOutputLimits(arg, s);
	}

void setOutputMin(void* arg, char* s)
{
	if (arg == NULL || s == NULL )
	{
		return;
	}
	PidParams params = ((FPid*)arg)->getParams();
	params.outputLimit.min = inputVolts2int(atof(s));
	((FPid*)arg)->setParams(params);
	getOutputLimits(arg, s);
}

void getOutputLimits(void* arg, char* s)
{
	char printBuffer[TEMPORARY_BUFFER_LENGTH];
	if (arg == NULL || s == NULL )
	{
		return;
	}
	PidParams params = ((FPid*)arg)->getParams();
	snprintf(printBuffer, TEMPORARY_BUFFER_LENGTH, OUTPUT_MIN_TOKEN SET_TOKEN "%f V, " OUTPUT_MAX_TOKEN SET_TOKEN "%f V", int2inputVolts(params.outputLimit.min), int2inputVolts(params.outputLimit.max));
	writeLine(printBuffer);
}

void setPointIntHandler(void* arg, char* s)
{
	if (arg == NULL || s == NULL )
	{
		return;
	}
	if (strstr(s, SET_TOKEN) != NULL)
	{
		setSetPointInt(arg, s);
	}
	else if (strstr(s, GET_TOKEN) != NULL)
	{
		getSetPointsInt(arg, s);
	}
}

void setPointFloatHandler(void* arg, char* s)
{
	if (arg == NULL || s == NULL )
	{
		return;
	}
	if (strstr(s, SET_TOKEN) != NULL)
	{
		setSetPointFloat(arg, s);
	}
	else if (strstr(s, GET_TOKEN) != NULL)
	{
		getSetPointsFloat(arg, s);
	}
}

void setSetPointInt(void* arg, char* s)
{
	if (arg == NULL || s == NULL )
	{
		return;
	}

	char* delimiter = strstr(s, SET_TOKEN);
	if (delimiter == NULL)
	{
		return;
	}

	*delimiter = '\0';
	uint16_t value = atoi((delimiter+1));
	uint16_t index = atoi(s);

	if (index >= 0 && index < NUM_SETPOINTS)
	{
		((SetpointManager*)(arg))->setSetPoint(index, value);
	}
	
	*delimiter = GET_TOKEN[0];
	getSetPointsInt(arg, s);
}

void setSetPointFloat(void* arg, char* s)
{
	if (arg == NULL || s == NULL )
	{
		return;
	}

	char* delimiter = strstr(s, SET_TOKEN);
	if (delimiter == NULL)
	{
		return;
	}

	*delimiter = '\0';
	float value = atof((delimiter+1));
	uint16_t index = atoi(s);

	if (index >= 0 && index < NUM_SETPOINTS)
	{
		((SetpointManager*)(arg))->setSetPoint(index, inputVolts2int(value));
	}
	
	*delimiter = GET_TOKEN[0];
	getSetPointsFloat(arg, s);
}

void getSetPointsInt(void* arg, char* s)
{
	char printBuffer[TEMPORARY_BUFFER_LENGTH];

	if (arg == NULL || s == NULL )
	{
		return;
	}
	char* delimiter = strstr(s, GET_TOKEN);
	if (delimiter != NULL)
	{
		*delimiter = '\0';
		uint8_t index = atoi(s);
		if (index >= 0 && index < NUM_SETPOINTS)
		{
			snprintf(printBuffer, TEMPORARY_BUFFER_LENGTH, SET_POINT_INT_TOKEN "%i" SET_TOKEN "%i", index, ((SetpointManager*)(arg))->getSetPoint(index));
			writeLine(printBuffer);
		}
	}
	else
	{
		for (uint8_t index = 0; index < NUM_SETPOINTS; index++)
		{
			char* isActiveSetpoint = "";
			if (index == ((SetpointManager*)(arg))->getSetPointIndex())
			{
				isActiveSetpoint = " <-";
			}

			snprintf(printBuffer, TEMPORARY_BUFFER_LENGTH, SET_POINT_INT_TOKEN "%i" SET_TOKEN "%i%s", index, ((SetpointManager*)(arg))->getSetPoint(index), isActiveSetpoint);
			writeLine(printBuffer);
		}
	}
}

void getSetPointsFloat(void* arg, char* s)
{
	char printBuffer[TEMPORARY_BUFFER_LENGTH];
	if (arg == NULL || s == NULL )
	{
		return;
	}

	char* delimiter = strstr(s, GET_TOKEN);
	if (delimiter != NULL)
	{
		*delimiter = '\0';
		uint8_t index = atoi(s);
		if (index >= 0 && index < NUM_SETPOINTS)
		{
			snprintf(printBuffer, TEMPORARY_BUFFER_LENGTH, SET_POINT_INT_TOKEN "%i" SET_TOKEN "%f V", index, int2inputVolts(((SetpointManager*)(arg))->getSetPoint(index)));
			writeLine(printBuffer);
		}
	}
	else
	{
		for (uint8_t index = 0; index < NUM_SETPOINTS; index++)
		{
			char* isActiveSetpoint = "";
			if (index == ((SetpointManager*)(arg))->getSetPointIndex())
			{
				isActiveSetpoint = " <-";
			}

			snprintf(printBuffer, TEMPORARY_BUFFER_LENGTH, SET_POINT_INT_TOKEN "%i" SET_TOKEN "%f V%s", index, int2inputVolts(((SetpointManager*)(arg))->getSetPoint(index)), isActiveSetpoint);
			writeLine(printBuffer);
		}
	}
}

void setReadAverages(void* arg, char* s)
{
	if (arg == NULL || s == NULL )
	{
		return;
	}
	float readAverages = atof(s);
	*((uint8_t*)arg)  = (uint8_t)log2(readAverages);
	getReadAverages(arg, s);
}

void getReadAverages(void* arg, char* s)
{
	char printBuffer[TEMPORARY_BUFFER_LENGTH];
	if (arg == NULL || s == NULL )
	{
		return;
	}
	snprintf(printBuffer, TEMPORARY_BUFFER_LENGTH, READ_AVERAGES_TOKEN SET_TOKEN "%i", 1 << *((uint8_t*)arg));
	writeLine(printBuffer);
}

void calibrateFeedForward(void* arg, char* s)
{
	if (arg == NULL)
	{
		return;
	}
	((FPid*)arg)->updateFeedForward();
	writeLine("Calibrated!");
}

void eepromReadWrite(void* arg, char* s)
{
	char printBuffer[TEMPORARY_BUFFER_LENGTH];
	if (arg == NULL || s == NULL )
	{
		return;
	}
	
	if (strcmp(s, EEPROM_READ_TOKEN))
	{
		((EepromManager*)(arg))->load();
	}
	else if (strcmp(s, EEPROM_WRITE_TOKEN))
	{
		((EepromManager*)(arg))->save();
	}
	snprintf(printBuffer, TEMPORARY_BUFFER_LENGTH, EEPROM_TOKEN SET_TOKEN "%s", s);
}

void resetPidState(void* arg, char* s)
{
	if (arg == NULL)
	{
		return;
	}
	((FPid*)(arg))->reset();
}

void setPidEnable(void* arg, char* s)
{
	if (arg == NULL || s == NULL )
	{
		return;
	}
	*((uint8_t*)arg) = atoi(s);
	getReadAverages(arg, s);
}

void getPidEnable(void* arg, char* s)
{
	char printBuffer[TEMPORARY_BUFFER_LENGTH];
	if (arg == NULL || s == NULL )
	{
		return;
	}
	snprintf(printBuffer, TEMPORARY_BUFFER_LENGTH, PID_ENABLE_TOKEN SET_TOKEN "%i", *((uint8_t*)arg));
	writeLine(printBuffer);
}

void setOutput(void* arg, char* s)
{
	if (arg == NULL || s == NULL )
	{
		return;
	}

	((FPid*)arg)->setOutputOpenLoop(outputVolts2int(atof(s)));
	getOutput(arg, s);
}

void getOutput(void* arg, char* s)
{
	char printBuffer[TEMPORARY_BUFFER_LENGTH];
	if (arg == NULL || s == NULL )
	{
		return;
	}

	uint16_t val = ((FPid*)arg)->getPidState().output;
	snprintf(printBuffer, TEMPORARY_BUFFER_LENGTH, SET_OUTPUT_TOKEN SET_TOKEN "%f V", int2outputVolts(val));
	writeLine(printBuffer);
}

void setPrintOutput(void* arg, char* s)
{
	if (arg == NULL || s == NULL )
	{
		return;
	}

	*((bool*)arg) = atoi(s);
}

void checkConfigError(FPid* controller)
{
	if (controller->getError())
	{
		writeLine("There was a configuration error!");
	}
}