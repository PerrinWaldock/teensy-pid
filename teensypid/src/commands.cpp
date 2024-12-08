#include "commands.h"
#include "commandregistrar.h"
#include "utils.hpp"
#include "serialmanager.h"
#include "datalogger.h"

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
#define OUTPUT_MIN_TOKEN "lo"
#define OUTPUT_MAX_TOKEN "ho"
#define SET_POINT_INT_TOKEN "sp"
#define SET_POINT_FLOAT_TOKEN "sv"
#define READ_AVERAGES_TOKEN "ra"

#define PID_ENABLE_TOKEN "pa"
#define PRINT_OUTPUT_TOKEN "po"
#define CALIBRATE_TOKEN "cf"
#define EEPROM_TOKEN "ew"
#define EEPROM_WRITE_TOKEN "w"
#define EEPROM_READ_TOKEN "r"
#define RESET_TOKEN "cl"
#define SET_OUTPUT_TOKEN "ov"

#define LOG_TOKEN "lo"
#define LOG_OFF_TOKEN "o"
#define LOG_SINGLE_TOKEN "s"
#define LOG_CONTINUOUS_TOKEN "c"
#define LOG_PRINT_TOKEN "p"

#define NUMBER_OF_SETPOINTS INPUT_STATES

#define EOL "\n"

#define createHandler(fn) [](void* o, char* s) {fn((CommandParserObjects*) o, s);}

void setKp(CommandParserObjects*, char*);
void getKp(CommandParserObjects*, char*);
void setKi(CommandParserObjects*, char*);
void getKi(CommandParserObjects*, char*);
void setKd(CommandParserObjects*, char*);
void getKd(CommandParserObjects*, char*);
void getKs(CommandParserObjects*, char*);
void setLoopRate(CommandParserObjects*, char*);
void getLoopRate(CommandParserObjects*, char*);
void setSetPointMax(CommandParserObjects*, char*);
void setSetPointMin(CommandParserObjects*, char*);
void getSetPointLimits(CommandParserObjects*, char*);
void setOutputMax(CommandParserObjects*, char*);
void setOutputMin(CommandParserObjects*, char*);
void getOutputLimits(CommandParserObjects*, char*);

void setPointIntHandler(CommandParserObjects*, char*);
void setPointFloatHandler(CommandParserObjects*, char*);
void setSetPointInt(CommandParserObjects*, char*);
void setSetPointFloat(CommandParserObjects*, char*);
void getSetPointsInt(CommandParserObjects*, char*);
void getSetPointsFloat(CommandParserObjects*, char*);

void setReadAverages(CommandParserObjects*, char*);
void getReadAverages(CommandParserObjects*, char*);

void setOutput(CommandParserObjects*, char*);
void getOutput(CommandParserObjects*, char*);
void setPidEnable(CommandParserObjects*, char*);
void getPidEnable(CommandParserObjects*, char*);

void setPrintOutput(CommandParserObjects*, char*);
void calibrateFeedForward(CommandParserObjects*, char*);
void eepromReadWrite(CommandParserObjects*, char*);
void resetPidState(CommandParserObjects*, char*);

void logSetter(CommandParserObjects*, char*);
void logGetter(CommandParserObjects*, char*);

void checkConfigError(CommandParserObjects*);


CommandParser::CommandParser(CommandParserObjects objects)
{
	this->objects = &objects;

	addCommand(KP_TOKEN SET_TOKEN, createHandler(setKp), (void*) &objects, "sets kp (float)");
	addCommand(KP_TOKEN GET_TOKEN, createHandler(getKp), (void*) &objects, "get kp (float)");
	addCommand(KI_TOKEN SET_TOKEN, createHandler(setKi), (void*) &objects, "sets ki (float)");
	addCommand(KI_TOKEN GET_TOKEN, createHandler(getKi), (void*) &objects, "get ki (float)");
	addCommand(KD_TOKEN SET_TOKEN, createHandler(setKd), (void*) &objects, "sets kd (float)");
	addCommand(KD_TOKEN GET_TOKEN, createHandler(getKd), (void*) &objects, "get kd (float)");
	addCommand(KS_TOKEN GET_TOKEN, createHandler(getKs), (void*) &objects, "gets all PID constants");

	addCommand(LOOP_RATE_TOKEN SET_TOKEN, createHandler(setLoopRate), (void*) &objects, "sets loop rate (Hz)");
	addCommand(LOOP_RATE_TOKEN GET_TOKEN, createHandler(getLoopRate), (void*) &objects, "gets loop rate (Hz)");

	addCommand(SET_POINT_MAX_TOKEN SET_TOKEN, createHandler(setSetPointMax), (void*) &objects, "sets maximum set point (V)");
	addCommand(SET_POINT_MIN_TOKEN SET_TOKEN, createHandler(setSetPointMin), (void*) &objects, "sets minimum set point (V)");
	addCommand(SET_POINT_MAX_TOKEN GET_TOKEN, createHandler(getSetPointLimits), (void*) &objects, "gets set point limits (V)");
	addCommand(SET_POINT_MIN_TOKEN GET_TOKEN, createHandler(getSetPointLimits), (void*) &objects, "gets set point limits (V)");

	addCommand(OUTPUT_MAX_TOKEN SET_TOKEN, createHandler(setOutputMax), (void*) &objects, "sets maximum output (V)");
	addCommand(OUTPUT_MIN_TOKEN SET_TOKEN, createHandler(setOutputMin), (void*) &objects, "sets minimum output (V)");
	addCommand(OUTPUT_MAX_TOKEN GET_TOKEN, createHandler(getOutputLimits), (void*) &objects, "gets output limits (V)");
	addCommand(OUTPUT_MIN_TOKEN GET_TOKEN, createHandler(getOutputLimits), (void*) &objects, "gets output limits (V)");

	addCommand(SET_POINT_INT_TOKEN, createHandler(setPointIntHandler), (void*) &objects, SET_POINT_INT_TOKEN "X interacts with the Xth set point's integer value (e.g. " SET_POINT_INT_TOKEN "0" SET_TOKEN "0 sets the 1st set point to 0; " SET_POINT_INT_TOKEN "1" GET_TOKEN " gets the 2nd set point)");
	addCommand(SET_POINT_INT_TOKEN GET_TOKEN, createHandler(getSetPointsInt), (void*) &objects, "gets all set point values as integers");
	addCommand(SET_POINT_FLOAT_TOKEN, createHandler(setPointFloatHandler), (void*) &objects, SET_POINT_FLOAT_TOKEN "X interacts with the Xth set point's voltage (e.g. " SET_POINT_FLOAT_TOKEN "1" SET_TOKEN "0 sets the 1st set point to 1 V; " SET_POINT_INT_TOKEN "1" GET_TOKEN " gets the 2nd set point voltage)");
	addCommand(SET_POINT_FLOAT_TOKEN GET_TOKEN, createHandler(getSetPointsFloat), (void*) &objects, "gets all set point values as voltages");

	addCommand(READ_AVERAGES_TOKEN SET_TOKEN, createHandler(setReadAverages), (void*) &objects, "sets number of read averages (must be a power of 2)");
	addCommand(READ_AVERAGES_TOKEN GET_TOKEN, createHandler(getReadAverages), (void*) &objects, "gets number of read averages");

	addCommand(CALIBRATE_TOKEN, createHandler(calibrateFeedForward), (void*) &objects, "calibrates feedforward lookup table");

	addCommand(RESET_TOKEN, createHandler(resetPidState), (void*) &objects, "resets PID state");

	addCommand(EEPROM_TOKEN, createHandler(eepromReadWrite), (void*) &objects, EEPROM_READ_TOKEN " to read parameters from EEPROM; " EEPROM_WRITE_TOKEN " to write parameters to EEPROM");

	addCommand(PRINT_OUTPUT_TOKEN SET_TOKEN, createHandler(setPrintOutput), (void*) &objects, "1 to print output, 0 to disable print output");

	addCommand(LOG_TOKEN SET_TOKEN, createHandler(logSetter), (void*) &objects, LOG_OFF_TOKEN " to turn logging off; " LOG_SINGLE_TOKEN " to log until buffer is full; " LOG_CONTINUOUS_TOKEN " to log continuously");
	addCommand(LOG_TOKEN GET_TOKEN, createHandler(logGetter), (void*) &objects, "to display inputs and outputs from the log");
}

bool CommandParser::parse(char* s)
{
	return processCommand(s);
}

void setKp(CommandParserObjects* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	FPid& pidController = obj->pidController;
	PidParams params = pidController.getParams();
	params.kp = atof(s);
	pidController.setParams(params);
	checkConfigError(obj);
	getKp(obj, s);
}

void getKp(CommandParserObjects* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	FPid& pidController = obj->pidController;
	PidParams params = pidController.getParams();
	obj->printer.printf(KP_TOKEN SET_TOKEN "%f" EOL, params.kp);
}

void setKi(CommandParserObjects* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	FPid& pidController = obj->pidController;
	PidParams params = pidController.getParams();
	params.ki = atof(s);
	pidController.setParams(params);
	checkConfigError(obj);
	getKi(obj, s);
}

void getKi(CommandParserObjects* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	FPid& pidController = obj->pidController;
	PidParams params = pidController.getParams();
	obj->printer.printf(KI_TOKEN SET_TOKEN "%f" EOL, params.ki);
}

void setKd(CommandParserObjects* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	FPid& pidController = obj->pidController;
	PidParams params = pidController.getParams(); //TODO fix below
	params.kd = atof(s);
	pidController.setParams(params);
	checkConfigError(obj);
	getKp(obj, s);
}

void getKd(CommandParserObjects* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	FPid& pidController = obj->pidController;
	PidParams params = pidController.getParams(); //TODO fix below
	obj->printer.printf(KD_TOKEN SET_TOKEN "%f" EOL, params.kd);
}

void getKs(CommandParserObjects* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	FPid& pidController = obj->pidController;
	PidParams params = pidController.getParams(); //TODO fix below
	obj->printer.printf(KD_TOKEN SET_TOKEN "%f, " KP_TOKEN SET_TOKEN "%f, " KI_TOKEN SET_TOKEN "%f" EOL, params.kd, params.kp, params.ki);
}

void setLoopRate(CommandParserObjects* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	FPid& pidController = obj->pidController;
	PidParams params = pidController.getParams(); //TODO fix below
	params.loopPeriod_us = (uint32_t)(1000000/atof(s));
	pidController.setParams(params);
	checkConfigError(obj);
	getKp(obj, s);
}

void getLoopRate(CommandParserObjects* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	FPid& pidController = obj->pidController;
	PidParams params = pidController.getParams(); //TODO fix below
	obj->printer.printf(LOOP_RATE_TOKEN SET_TOKEN "%f Hz" EOL, 1000000.0/params.loopPeriod_us);
}

void setSetPointMax(CommandParserObjects* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	FPid& pidController = obj->pidController;
	PidParams params = pidController.getParams(); //TODO fix below
	params.setPointLimit.max = inputVolts2int(atof(s));
	pidController.setParams(params);
	getSetPointLimits(obj, s);
}

void setSetPointMin(CommandParserObjects* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	FPid& pidController = obj->pidController;
	PidParams params = pidController.getParams(); //TODO fix below
	params.setPointLimit.min = inputVolts2int(atof(s));
	pidController.setParams(params);
	getSetPointLimits(obj, s);
}

void getSetPointLimits(CommandParserObjects* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	FPid& pidController = obj->pidController;
	PidParams params = pidController.getParams(); //TODO fix below
	obj->printer.printf(SET_POINT_MIN_TOKEN SET_TOKEN "%f V, " SET_POINT_MAX_TOKEN SET_TOKEN "%f V" EOL, int2inputVolts(params.setPointLimit.min), int2inputVolts(params.setPointLimit.max));
}

void setOutputMax(CommandParserObjects* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	FPid& pidController = obj->pidController;
	PidParams params = pidController.getParams(); //TODO fix below
	params.outputLimit.max = inputVolts2int(atof(s));
	pidController.setParams(params);
	getOutputLimits(obj, s);
	}

void setOutputMin(CommandParserObjects* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	FPid& pidController = obj->pidController;
	PidParams params = pidController.getParams(); //TODO fix below
	params.outputLimit.min = inputVolts2int(atof(s));
	pidController.setParams(params);
	getOutputLimits(obj, s);
}

void getOutputLimits(CommandParserObjects* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	FPid& pidController = obj->pidController;
	PidParams params = pidController.getParams(); //TODO fix below
	obj->printer.printf( OUTPUT_MIN_TOKEN SET_TOKEN "%f V, " OUTPUT_MAX_TOKEN SET_TOKEN "%f V" EOL, int2inputVolts(params.outputLimit.min), int2inputVolts(params.outputLimit.max));
}

//////////////////////////////TODO fix all handlers below

void setPointIntHandler(CommandParserObjects* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	if (strstr(s, SET_TOKEN) != NULL)
	{
		setSetPointInt(obj, s);
	}
	else if (strstr(s, GET_TOKEN) != NULL)
	{
		getSetPointsInt(obj, s);
	}
}

void setPointFloatHandler(CommandParserObjects* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	if (strstr(s, SET_TOKEN) != NULL)
	{
		setSetPointFloat(obj, s);
	}
	else if (strstr(s, GET_TOKEN) != NULL)
	{
		getSetPointsFloat(obj, s);
	}
}

void setSetPointInt(CommandParserObjects* obj, char* s)
{
	if (obj == NULL || s == NULL )
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
		obj->setpointManager.setSetPoint(index, value);
	}
	
	*delimiter = GET_TOKEN[0];
	getSetPointsInt(obj, s);
}

void setSetPointFloat(CommandParserObjects* obj, char* s)
{
	if (obj == NULL || s == NULL )
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
		obj->setpointManager.setSetPoint(index, inputVolts2int(value));
	}
	
	*delimiter = GET_TOKEN[0];
	getSetPointsFloat(obj, s);
}

void getSetPointsInt(CommandParserObjects* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	char* delimiter = strstr(s, GET_TOKEN);
	uint8_t minIndex = 0;
	uint8_t maxIndex = NUM_SETPOINTS;
	if (delimiter != NULL)
	{
		*delimiter = '\0';
		uint8_t index = atoi(s);
		if (index >= 0 && index < NUM_SETPOINTS)
		{
			minIndex = index;
			maxIndex = index + 1;
		}
	}
	for (uint8_t index = minIndex; index < maxIndex; index++)
	{
		const char* isActiveSetpoint = index == obj->setpointManager.getSetPointIndex() ? " <-" : "";
		obj->printer.printf(SET_POINT_INT_TOKEN "%i" SET_TOKEN "%i%s" EOL, index, obj->setpointManager.getSetPoint(index), isActiveSetpoint);
	}
}

void getSetPointsFloat(CommandParserObjects* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	char* delimiter = strstr(s, GET_TOKEN);
	uint8_t minIndex = 0;
	uint8_t maxIndex = NUM_SETPOINTS;
	if (delimiter != NULL)
	{
		*delimiter = '\0';
		uint8_t index = atoi(s);
		if (index >= 0 && index < NUM_SETPOINTS)
		{
			minIndex = index;
			maxIndex = index + 1;
		}
	}
	for (uint8_t index = minIndex; index < maxIndex; index++)
	{
		const char* isActiveSetpoint = index == obj->setpointManager.getSetPointIndex() ? " <-" : "";
		obj->printer.printf(SET_POINT_INT_TOKEN "%i" SET_TOKEN "%f V%s" EOL, index, int2inputVolts(obj->setpointManager.getSetPoint(index)), isActiveSetpoint);
	}
}

void setReadAverages(CommandParserObjects* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	float readAverages = atof(s);
	obj->readAveragesPower  = (uint8_t)log2(readAverages);
	getReadAverages(obj, s);
}

void getReadAverages(CommandParserObjects* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	obj->printer.printf(READ_AVERAGES_TOKEN SET_TOKEN "%i" EOL, 1 << obj->readAveragesPower);
}

void calibrateFeedForward(CommandParserObjects* obj, char* s)
{
	if (obj == NULL)
	{
		return;
	}
	obj->pidController.updateFeedForward();
	obj->printer.printf("Calibrated!" EOL);
}

void eepromReadWrite(CommandParserObjects* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	
	if (strcmp(s, EEPROM_READ_TOKEN))
	{
		obj->eepromManager.load();
	}
	else if (strcmp(s, EEPROM_WRITE_TOKEN))
	{
		obj->eepromManager.save();
	}
	obj->printer.printf(EEPROM_TOKEN SET_TOKEN "%s" EOL, s);
}

void resetPidState(CommandParserObjects* obj, char* s)
{
	if (obj == NULL)
	{
		return;
	}
	obj->pidController.reset();
}

void setPidEnable(CommandParserObjects* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	uint8_t value = atoi(s);
	obj->pidController.setPidActive(value);
	getPidEnable(obj, s);
}

void getPidEnable(CommandParserObjects* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	bool active = obj->pidController.getPidState().active;
	obj->printer.printf(PID_ENABLE_TOKEN SET_TOKEN "%i" EOL, active);
}

void setOutput(CommandParserObjects* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	obj->pidController.setOutputOpenLoop(outputVolts2int(atof(s)));
	getOutput(obj, s);
}

void getOutput(CommandParserObjects* obj, char* s)
{
	char printBuffer[TEMPORARY_BUFFER_LENGTH];
	if (obj == NULL || s == NULL )
	{
		return;
	}
	uint16_t val = obj->pidController.getPidState().output;
	obj->printer.printf(printBuffer, TEMPORARY_BUFFER_LENGTH, SET_OUTPUT_TOKEN SET_TOKEN "%f V" EOL, int2outputVolts(val));
}

void setPrintOutput(CommandParserObjects* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	obj->printOutput = atoi(s);
}

void checkConfigError(CommandParserObjects* objects)
{
	if (objects == NULL)
	{
		return;
	}
	if (objects->pidController.getError())
	{
		objects->printer.printf("There was a configuration error!" EOL);
	}
}

void logSetter(CommandParserObjects* obj, char* s)
{
	if (obj == NULL || s == NULL)
	{
		return;
	}
	DataLog& log = obj->log;

	if (strcmp(s, LOG_OFF_TOKEN))
	{
		log.state = LOG_OFF;
		obj->printer.printf(LOG_TOKEN SET_TOKEN LOG_OFF_TOKEN EOL);
	}
	else if(strcmp(s, LOG_SINGLE_TOKEN))
	{
		log.state = LOG_SINGLE;
		obj->printer.printf(LOG_TOKEN SET_TOKEN LOG_SINGLE_TOKEN EOL);
	}
	else if (strcmp(s, LOG_CONTINUOUS_TOKEN))
	{
		log.state = LOG_CONTINUOUS;
		obj->printer.printf(LOG_TOKEN SET_TOKEN LOG_CONTINUOUS_TOKEN EOL);
	}
	else
	{
		obj->printer.printf("%s not recognized" EOL, s);
	}
}

void logGetter(CommandParserObjects* obj, char* s)
{
	if (obj == NULL || s == NULL)
	{
		return;
	}
	DataLog& log = obj->log;

	while (!log.input.isEmpty() && !log.output.isEmpty())
	{
		obj->printer.printf("i: %i\t o: %i" EOL, log.input.shift(), log.output.shift());
	}
}