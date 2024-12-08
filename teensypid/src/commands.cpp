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

// TODO convert to handlers that take specific arguments, do the void conversion in a lambda
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
void setSetPointInt(void*, char*);
void setSetPointFloat(void*, char*);
void getSetPointsInt(void*, char*);
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

void logSetter(void*, char*);
void logGetter(void*, char*);

void checkConfigError(CommandParserObjects*);


CommandParser::CommandParser(CommandParserObjects objects)
{
	this->objects = &objects;

	addCommand(KP_TOKEN SET_TOKEN, setKp, (void*) &objects, "sets kp (float)");
	addCommand(KP_TOKEN GET_TOKEN, getKp, (void*) &objects, "get kp (float)");
	addCommand(KI_TOKEN SET_TOKEN, setKi, (void*) &objects, "sets ki (float)");
	addCommand(KI_TOKEN GET_TOKEN, getKi, (void*) &objects, "get ki (float)");
	addCommand(KD_TOKEN SET_TOKEN, setKd, (void*) &objects, "sets kd (float)");
	addCommand(KD_TOKEN GET_TOKEN, getKd, (void*) &objects, "get kd (float)");
	addCommand(KS_TOKEN GET_TOKEN, getKs, (void*) &objects, "gets all PID constants");

	addCommand(LOOP_RATE_TOKEN SET_TOKEN, setLoopRate, (void*) &objects, "sets loop rate (Hz)");
	addCommand(LOOP_RATE_TOKEN GET_TOKEN, getLoopRate, (void*) &objects, "gets loop rate (Hz)");

	addCommand(SET_POINT_MAX_TOKEN SET_TOKEN, setSetPointMax, (void*) &objects, "sets maximum set point (V)");
	addCommand(SET_POINT_MIN_TOKEN SET_TOKEN, setSetPointMin, (void*) &objects, "sets minimum set point (V)");
	addCommand(SET_POINT_MAX_TOKEN GET_TOKEN, getSetPointLimits, (void*) &objects, "gets set point limits (V)");
	addCommand(SET_POINT_MIN_TOKEN GET_TOKEN, getSetPointLimits, (void*) &objects, "gets set point limits (V)");

	addCommand(OUTPUT_MAX_TOKEN SET_TOKEN, setOutputMax, (void*) &objects, "sets maximum output (V)");
	addCommand(OUTPUT_MIN_TOKEN SET_TOKEN, setOutputMin, (void*) &objects, "sets minimum output (V)");
	addCommand(OUTPUT_MAX_TOKEN GET_TOKEN, getOutputLimits, (void*) &objects, "gets output limits (V)");
	addCommand(OUTPUT_MIN_TOKEN GET_TOKEN, getOutputLimits, (void*) &objects, "gets output limits (V)");

	addCommand(SET_POINT_INT_TOKEN, setPointIntHandler, (void*) &objects, SET_POINT_INT_TOKEN "X interacts with the Xth set point's integer value (e.g. " SET_POINT_INT_TOKEN "0" SET_TOKEN "0 sets the 1st set point to 0; " SET_POINT_INT_TOKEN "1" GET_TOKEN " gets the 2nd set point)");
	addCommand(SET_POINT_INT_TOKEN GET_TOKEN, getSetPointsInt, (void*) &objects, "gets all set point values as integers");
	addCommand(SET_POINT_FLOAT_TOKEN, setPointFloatHandler, (void*) &objects, SET_POINT_FLOAT_TOKEN "X interacts with the Xth set point's voltage (e.g. " SET_POINT_FLOAT_TOKEN "1" SET_TOKEN "0 sets the 1st set point to 1 V; " SET_POINT_INT_TOKEN "1" GET_TOKEN " gets the 2nd set point voltage)");
	addCommand(SET_POINT_FLOAT_TOKEN GET_TOKEN, getSetPointsFloat, (void*) &objects, "gets all set point values as voltages");

	addCommand(READ_AVERAGES_TOKEN SET_TOKEN, setReadAverages, (void*) &objects, "sets number of read averages (must be a power of 2)");
	addCommand(READ_AVERAGES_TOKEN GET_TOKEN, getReadAverages, (void*) &objects, "gets number of read averages");

	addCommand(CALIBRATE_TOKEN, calibrateFeedForward, (void*) &objects, "calibrates feedforward lookup table");

	addCommand(RESET_TOKEN, resetPidState, (void*) &objects, "resets PID state");

	addCommand(EEPROM_TOKEN, eepromReadWrite, (void*) &objects, EEPROM_READ_TOKEN " to read parameters from EEPROM; " EEPROM_WRITE_TOKEN " to write parameters to EEPROM");

	addCommand(PRINT_OUTPUT_TOKEN SET_TOKEN, setPrintOutput, (void*) &objects, "1 to print output, 0 to disable print output");

	addCommand(LOG_TOKEN SET_TOKEN, logSetter, (void*) &objects, LOG_OFF_TOKEN " to turn logging off; " LOG_SINGLE_TOKEN " to log until buffer is full; " LOG_CONTINUOUS_TOKEN " to log continuously");
	addCommand(LOG_TOKEN GET_TOKEN, logGetter, (void*) &objects, "to display inputs and outputs from the log");
}

bool CommandParser::parse(char* s)
{
	return processCommand(s);
}

void setKp(void* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	CommandParserObjects* objects = (CommandParserObjects*) obj;
	FPid pidController = objects->pidController;
	PidParams params = pidController.getParams();
	params.kp = atof(s);
	pidController.setParams(params);
	checkConfigError(objects);
	getKp(obj, s);
}

void getKp(void* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	CommandParserObjects* objects = (CommandParserObjects*) obj;
	FPid pidController = objects->pidController;
	PidParams params = pidController.getParams();
	objects->printer.printf(KP_TOKEN SET_TOKEN "%f" EOL, params.kp);
}

void setKi(void* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	CommandParserObjects* objects = (CommandParserObjects*) obj;
	FPid pidController = objects->pidController;
	PidParams params = pidController.getParams();
	params.ki = atof(s);
	pidController.setParams(params);
	checkConfigError(objects);
	getKi(obj, s);
}

void getKi(void* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	CommandParserObjects* objects = (CommandParserObjects*) obj;
	FPid pidController = objects->pidController;
	PidParams params = pidController.getParams();
	objects->printer.printf(KI_TOKEN SET_TOKEN "%f" EOL, params.ki);
}

void setKd(void* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	CommandParserObjects* objects = (CommandParserObjects*) obj;
	FPid pidController = objects->pidController;
	PidParams params = pidController.getParams(); //TODO fix below
	params.kd = atof(s);
	pidController.setParams(params);
	checkConfigError(objects);
	getKp(obj, s);
}

void getKd(void* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	CommandParserObjects* objects = (CommandParserObjects*) obj;
	FPid pidController = objects->pidController;
	PidParams params = pidController.getParams(); //TODO fix below
	objects->printer.printf(KD_TOKEN SET_TOKEN "%f" EOL, params.kd);
}

void getKs(void* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	CommandParserObjects* objects = (CommandParserObjects*) obj;
	FPid pidController = objects->pidController;
	PidParams params = pidController.getParams(); //TODO fix below
	objects->printer.printf(KD_TOKEN SET_TOKEN "%f, " KP_TOKEN SET_TOKEN "%f, " KI_TOKEN SET_TOKEN "%f" EOL, params.kd, params.kp, params.ki);
}

void setLoopRate(void* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	CommandParserObjects* objects = (CommandParserObjects*) obj;
	FPid pidController = objects->pidController;
	PidParams params = pidController.getParams(); //TODO fix below
	params.loopPeriod_us = (uint32_t)(1000000/atof(s));
	pidController.setParams(params);
	checkConfigError(objects);
	getKp(obj, s);
}

void getLoopRate(void* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	CommandParserObjects* objects = (CommandParserObjects*) obj;
	FPid pidController = objects->pidController;
	PidParams params = pidController.getParams(); //TODO fix below
	objects->printer.printf(LOOP_RATE_TOKEN SET_TOKEN "%f Hz" EOL, 1000000.0/params.loopPeriod_us);
}

void setSetPointMax(void* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	CommandParserObjects* objects = (CommandParserObjects*) obj;
	FPid pidController = objects->pidController;
	PidParams params = pidController.getParams(); //TODO fix below
	params.setPointLimit.max = inputVolts2int(atof(s));
	pidController.setParams(params);
	getSetPointLimits(obj, s);
}

void setSetPointMin(void* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	CommandParserObjects* objects = (CommandParserObjects*) obj;
	FPid pidController = objects->pidController;
	PidParams params = pidController.getParams(); //TODO fix below
	params.setPointLimit.min = inputVolts2int(atof(s));
	pidController.setParams(params);
	getSetPointLimits(obj, s);
}

void getSetPointLimits(void* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	CommandParserObjects* objects = (CommandParserObjects*) obj;
	FPid pidController = objects->pidController;
	PidParams params = pidController.getParams(); //TODO fix below
	objects->printer.printf(SET_POINT_MIN_TOKEN SET_TOKEN "%f V, " SET_POINT_MAX_TOKEN SET_TOKEN "%f V" EOL, int2inputVolts(params.setPointLimit.min), int2inputVolts(params.setPointLimit.max));
}

void setOutputMax(void* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	CommandParserObjects* objects = (CommandParserObjects*) obj;
	FPid pidController = objects->pidController;
	PidParams params = pidController.getParams(); //TODO fix below
	params.outputLimit.max = inputVolts2int(atof(s));
	pidController.setParams(params);
	getOutputLimits(obj, s);
	}

void setOutputMin(void* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	CommandParserObjects* objects = (CommandParserObjects*) obj;
	FPid pidController = objects->pidController;
	PidParams params = pidController.getParams(); //TODO fix below
	params.outputLimit.min = inputVolts2int(atof(s));
	pidController.setParams(params);
	getOutputLimits(obj, s);
}

void getOutputLimits(void* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	CommandParserObjects* objects = (CommandParserObjects*) obj;
	FPid pidController = objects->pidController;
	PidParams params = pidController.getParams(); //TODO fix below
	objects->printer.printf( OUTPUT_MIN_TOKEN SET_TOKEN "%f V, " OUTPUT_MAX_TOKEN SET_TOKEN "%f V" EOL, int2inputVolts(params.outputLimit.min), int2inputVolts(params.outputLimit.max));
}

//////////////////////////////TODO fix all handlers below

void setPointIntHandler(void* obj, char* s)
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

void setPointFloatHandler(void* obj, char* s)
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

void setSetPointInt(void* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	CommandParserObjects* objects = (CommandParserObjects*) obj;

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
		objects->setpointManager.setSetPoint(index, value);
	}
	
	*delimiter = GET_TOKEN[0];
	getSetPointsInt(obj, s);
}

void setSetPointFloat(void* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	CommandParserObjects* objects = (CommandParserObjects*) obj;

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
		objects->setpointManager.setSetPoint(index, inputVolts2int(value));
	}
	
	*delimiter = GET_TOKEN[0];
	getSetPointsFloat(obj, s);
}

void getSetPointsInt(void* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	CommandParserObjects* objects = (CommandParserObjects*) obj;
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
		const char* isActiveSetpoint = index == objects->setpointManager.getSetPointIndex() ? " <-" : "";
		objects->printer.printf(SET_POINT_INT_TOKEN "%i" SET_TOKEN "%i%s" EOL, index, objects->setpointManager.getSetPoint(index), isActiveSetpoint);
	}
}

void getSetPointsFloat(void* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	CommandParserObjects* objects = (CommandParserObjects*) obj;
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
		const char* isActiveSetpoint = index == objects->setpointManager.getSetPointIndex() ? " <-" : "";
		objects->printer.printf(SET_POINT_INT_TOKEN "%i" SET_TOKEN "%f V%s" EOL, index, int2inputVolts(objects->setpointManager.getSetPoint(index)), isActiveSetpoint);
	}
}

void setReadAverages(void* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	CommandParserObjects* objects = (CommandParserObjects*) obj;
	float readAverages = atof(s);
	objects->readAveragesPower  = (uint8_t)log2(readAverages);
	getReadAverages(obj, s);
}

void getReadAverages(void* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	CommandParserObjects* objects = (CommandParserObjects*) obj;
	objects->printer.printf(READ_AVERAGES_TOKEN SET_TOKEN "%i" EOL, 1 << objects->readAveragesPower);
}

void calibrateFeedForward(void* obj, char* s)
{
	if (obj == NULL)
	{
		return;
	}
	CommandParserObjects* objects = (CommandParserObjects*) obj;
	objects->pidController.updateFeedForward();
	objects->printer.printf("Calibrated!" EOL);
}

void eepromReadWrite(void* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	CommandParserObjects* objects = (CommandParserObjects*) obj;
	
	if (strcmp(s, EEPROM_READ_TOKEN))
	{
		objects->eepromManager.load();
	}
	else if (strcmp(s, EEPROM_WRITE_TOKEN))
	{
		objects->eepromManager.save();
	}
	objects->printer.printf(EEPROM_TOKEN SET_TOKEN "%s" EOL, s);
}

void resetPidState(void* obj, char* s)
{
	if (obj == NULL)
	{
		return;
	}
	CommandParserObjects* objects = (CommandParserObjects*) obj;
	objects->pidController.reset();
}

void setPidEnable(void* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	CommandParserObjects* objects = (CommandParserObjects*) obj;
	uint8_t value = atoi(s);
	objects->pidController.setPidActive(value);
	getPidEnable(obj, s);
}

void getPidEnable(void* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	CommandParserObjects* objects = (CommandParserObjects*) obj;
	bool active = objects->pidController.getPidState().active;
	objects->printer.printf(PID_ENABLE_TOKEN SET_TOKEN "%i" EOL, active);
}

void setOutput(void* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	CommandParserObjects* objects = (CommandParserObjects*) obj;
	objects->pidController.setOutputOpenLoop(outputVolts2int(atof(s)));
	getOutput(obj, s);
}

void getOutput(void* obj, char* s)
{
	char printBuffer[TEMPORARY_BUFFER_LENGTH];
	if (obj == NULL || s == NULL )
	{
		return;
	}
	CommandParserObjects* objects = (CommandParserObjects*) obj;

	uint16_t val = objects->pidController.getPidState().output;
	objects->printer.printf(printBuffer, TEMPORARY_BUFFER_LENGTH, SET_OUTPUT_TOKEN SET_TOKEN "%f V" EOL, int2outputVolts(val));
}

void setPrintOutput(void* obj, char* s)
{
	if (obj == NULL || s == NULL )
	{
		return;
	}
	CommandParserObjects* objects = (CommandParserObjects*) obj;
	objects->printOutput = atoi(s);
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

void logSetter(void* obj, char* s)
{
	if (obj == NULL || s == NULL)
	{
		return;
	}
	CommandParserObjects* objects = (CommandParserObjects*) obj;
	DataLog& log = objects->log;

	if (strcmp(s, LOG_OFF_TOKEN))
	{
		log.state = LOG_OFF;
		objects->printer.printf(LOG_TOKEN SET_TOKEN LOG_OFF_TOKEN EOL);
	}
	else if(strcmp(s, LOG_SINGLE_TOKEN))
	{
		log.state = LOG_SINGLE;
		objects->printer.printf(LOG_TOKEN SET_TOKEN LOG_SINGLE_TOKEN EOL);
	}
	else if (strcmp(s, LOG_CONTINUOUS_TOKEN))
	{
		log.state = LOG_CONTINUOUS;
		objects->printer.printf(LOG_TOKEN SET_TOKEN LOG_CONTINUOUS_TOKEN EOL);
	}
	else
	{
		objects->printer.printf("%s not recognized" EOL, s);
	}
}

void logGetter(void* obj, char* s)
{
	if (obj == NULL || s == NULL)
	{
		return;
	}
	CommandParserObjects* objects = (CommandParserObjects*) obj;
	DataLog& log = objects->log;

	while (!log.input.isEmpty() && !log.output.isEmpty())
	{
		objects->printer.printf("i: %i\t o: %i" EOL, log.input.shift(), log.output.shift());
	}
}