#include "commands.h"
#include "commandregistrar.h"
#include "utils.hpp"
#include "serialmanager.h"
#include "datalogger.h"
#include "settings.h"

#include <string.h>

#define TEMPORARY_BUFFER_LENGTH 128

#define SET_TOKEN "="
#define GET_TOKEN "?"

#define KP_TOKEN "kp"
#define KI_TOKEN "ki"
#define KD_TOKEN "kd"
#define KS_TOKEN "ks"
#define KLIMITS_TOKEN "kl"
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
#define PRINT_PERIOD_TOKEN "pp"
#define CALIBRATE_TOKEN "cf"
#define FEEDFORWARD_TOKEN "ff"
#define EEPROM_TOKEN "ew"
#define EEPROM_WRITE_TOKEN "w"
#define EEPROM_READ_TOKEN "r"
#define RESET_TOKEN "cl"
#define SET_OUTPUT_TOKEN "ov"
#define GET_FEEDBACK_TOKEN "fv"
#define NAME_TOKEN "nm"

#define LOG_TOKEN "lg"
#define LOG_OFF_TOKEN "o"
#define LOG_SINGLE_TOKEN "s"
#define LOG_CONTINUOUS_TOKEN "c"
#define LOG_PRINT_TOKEN "p"

#define NUMBER_OF_SETPOINTS INPUT_STATES

#define EOL "\n"
#define NAME "fpidController"

#define createHandler(fn) [](void* o, char* s) {fn((CommandParserObjects*) o, s);}

void setKp(CommandParserObjects*, char*);
void getKp(CommandParserObjects*, char*);
void setKi(CommandParserObjects*, char*);
void getKi(CommandParserObjects*, char*);
void setKd(CommandParserObjects*, char*);
void getKd(CommandParserObjects*, char*);
void getKs(CommandParserObjects*, char*);
void getKsLimits(CommandParserObjects*, char*);
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
void getFeedback(CommandParserObjects*, char*);
void setPidEnable(CommandParserObjects*, char*);
void getPidEnable(CommandParserObjects*, char*);

void setPrintOutput(CommandParserObjects*, char*);
void setPrintPeriod(CommandParserObjects*, char*);
void printFeedForward(CommandParserObjects*, char*);
void calibrateFeedForward(CommandParserObjects*, char*);
void eepromReadWrite(CommandParserObjects*, char*);
void resetPidState(CommandParserObjects*, char*);

void logSetter(CommandParserObjects*, char*);
void getLogState(CommandParserObjects*, char*);
void logGetter(CommandParserObjects*, char*);
void getName(CommandParserObjects*, char*);

void checkConfigError(CommandParserObjects*, char*);

bool areNullPointers(CommandParserObjects objects)
{
	return objects.pidController == nullptr
	|| objects.eepromManager == nullptr
	|| objects.setpointManager == nullptr
	|| objects.readAveragesPower == nullptr
	|| objects.printState == nullptr
	|| objects.log == nullptr
	|| objects.printer == nullptr;
}

bool handlerHasNullPointer(CommandParserObjects *obj, char *s)
{
	return obj == nullptr || s == nullptr  || areNullPointers(*obj);
}


CommandParser::CommandParser(CommandParserObjects objects)
{
	this->objects = objects;

	addCommand(NAME_TOKEN GET_TOKEN, createHandler(getName), (void*) &(this->objects), "to get name");

	addCommand(KP_TOKEN SET_TOKEN, createHandler(setKp), (void*) &(this->objects), "sets kp (float)");
	addCommand(KP_TOKEN GET_TOKEN, createHandler(getKp), (void*) &(this->objects), "get kp (float)");
	addCommand(KI_TOKEN SET_TOKEN, createHandler(setKi), (void*) &(this->objects), "sets ki (float)");
	addCommand(KI_TOKEN GET_TOKEN, createHandler(getKi), (void*) &(this->objects), "get ki (float)");
	addCommand(KD_TOKEN SET_TOKEN, createHandler(setKd), (void*) &(this->objects), "sets kd (float)");
	addCommand(KD_TOKEN GET_TOKEN, createHandler(getKd), (void*) &(this->objects), "get kd (float)");
	addCommand(KS_TOKEN GET_TOKEN, createHandler(getKs), (void*) &(this->objects), "gets all PID constants");
	addCommand(KLIMITS_TOKEN GET_TOKEN, createHandler(getKsLimits), (void*) &(this->objects), "gets limits for all PID constants");

	addCommand(LOOP_RATE_TOKEN SET_TOKEN, createHandler(setLoopRate), (void*) &(this->objects), "sets nominal loop rate (Hz)");
	addCommand(LOOP_RATE_TOKEN GET_TOKEN, createHandler(getLoopRate), (void*) &(this->objects), "gets nominal loop rate (Hz)");

	addCommand(SET_POINT_MAX_TOKEN SET_TOKEN, createHandler(setSetPointMax), (void*) &(this->objects), "sets maximum set point (V)");
	addCommand(SET_POINT_MIN_TOKEN SET_TOKEN, createHandler(setSetPointMin), (void*) &(this->objects), "sets minimum set point (V)");
	addCommand(SET_POINT_MAX_TOKEN GET_TOKEN, createHandler(getSetPointLimits), (void*) &(this->objects), "gets set point limits (V)");
	addCommand(SET_POINT_MIN_TOKEN GET_TOKEN, createHandler(getSetPointLimits), (void*) &(this->objects), "gets set point limits (V)");

	addCommand(OUTPUT_MAX_TOKEN SET_TOKEN, createHandler(setOutputMax), (void*) &(this->objects), "sets maximum output (V)");
	addCommand(OUTPUT_MIN_TOKEN SET_TOKEN, createHandler(setOutputMin), (void*) &(this->objects), "sets minimum output (V)");
	addCommand(OUTPUT_MAX_TOKEN GET_TOKEN, createHandler(getOutputLimits), (void*) &(this->objects), "gets output limits (V)");
	addCommand(OUTPUT_MIN_TOKEN GET_TOKEN, createHandler(getOutputLimits), (void*) &(this->objects), "gets output limits (V)");

	addCommand(SET_POINT_INT_TOKEN, createHandler(setPointIntHandler), (void*) &(this->objects), SET_POINT_INT_TOKEN "X interacts with the Xth set point's integer value (e.g. " SET_POINT_INT_TOKEN "0" SET_TOKEN "0 sets the 1st set point to 0; " SET_POINT_INT_TOKEN "1" GET_TOKEN " gets the 2nd set point)");
	addCommand(SET_POINT_INT_TOKEN GET_TOKEN, createHandler(getSetPointsInt), (void*) &(this->objects), "gets all set point values as integers");
	addCommand(SET_POINT_FLOAT_TOKEN, createHandler(setPointFloatHandler), (void*) &(this->objects), SET_POINT_FLOAT_TOKEN "X interacts with the Xth set point's voltage (e.g. " SET_POINT_FLOAT_TOKEN "1" SET_TOKEN "0 sets the 1st set point to 1 V; " SET_POINT_INT_TOKEN "1" GET_TOKEN " gets the 2nd set point voltage)");
	addCommand(SET_POINT_FLOAT_TOKEN GET_TOKEN, createHandler(getSetPointsFloat), (void*) &(this->objects), "gets all set point values as voltages");

	addCommand(READ_AVERAGES_TOKEN SET_TOKEN, createHandler(setReadAverages), (void*) &(this->objects), "sets number of read averages (must be a power of 2)");
	addCommand(READ_AVERAGES_TOKEN GET_TOKEN, createHandler(getReadAverages), (void*) &(this->objects), "gets number of read averages");

	addCommand(FEEDFORWARD_TOKEN GET_TOKEN, createHandler(printFeedForward), (void*) &(this->objects), "prints the feedforward lookup table readings");	
	addCommand(CALIBRATE_TOKEN, createHandler(calibrateFeedForward), (void*) &(this->objects), "calibrates feedforward lookup table");
	addCommand(RESET_TOKEN, createHandler(resetPidState), (void*) &(this->objects), "resets PID state");
	addCommand(EEPROM_TOKEN SET_TOKEN, createHandler(eepromReadWrite), (void*) &(this->objects), EEPROM_READ_TOKEN " to read parameters from EEPROM; " EEPROM_WRITE_TOKEN " to write parameters to EEPROM");
	addCommand(PRINT_OUTPUT_TOKEN SET_TOKEN, createHandler(setPrintOutput), (void*) &(this->objects), "1 to print output, 0 to disable print output");
	addCommand(PRINT_PERIOD_TOKEN SET_TOKEN, createHandler(setPrintPeriod), (void*) &(this->objects), "set print output period (ms)");

	addCommand(PID_ENABLE_TOKEN SET_TOKEN, createHandler(setPidEnable), (void*) &(this->objects), "0 to disable pid, 1 to enable");
	addCommand(PID_ENABLE_TOKEN GET_TOKEN, createHandler(getPidEnable), (void*) &(this->objects), "gets whether pid is enabled (0 off; 1 on)");
	addCommand(SET_OUTPUT_TOKEN SET_TOKEN, createHandler(setOutput), (void*) &(this->objects), "to set output voltage");
	addCommand(SET_OUTPUT_TOKEN GET_TOKEN, createHandler(getOutput), (void*) &(this->objects), "to get output voltage");
	addCommand(GET_FEEDBACK_TOKEN GET_TOKEN, createHandler(getFeedback), (void*) &(this->objects), "to get feedback voltage");

	addCommand(LOG_TOKEN SET_TOKEN, createHandler(logSetter), (void*) &(this->objects), LOG_OFF_TOKEN " to turn logging off; " LOG_SINGLE_TOKEN " to log until buffer is full; " LOG_CONTINUOUS_TOKEN " to log continuously");
	addCommand(LOG_TOKEN GET_TOKEN, createHandler(logGetter), (void*) &(this->objects), "to display inputs and outputs from the log");
}

bool CommandParser::parse(char* s)
{
	return processCommand(s);
}

void setKp(CommandParserObjects* obj, char* s)
{
	if (handlerHasNullPointer(obj, s))
	{
		return;
	}
	FPid& pidController = *(obj->pidController);
	PidParams params = pidController.getParams();
	params.kp = atof(s);
	pidController.setParams(params);
	checkConfigError(obj, s);
	getKp(obj, s);
}

void getKp(CommandParserObjects* obj, char* s)
{
	if (handlerHasNullPointer(obj, s))
	{
		return;
	}
	FPid& pidController = *(obj->pidController);
	PidParams params = pidController.getParams();
	obj->printer->printf(KP_TOKEN SET_TOKEN "%f" EOL, params.kp);
}

void setKi(CommandParserObjects* obj, char* s)
{
	if (handlerHasNullPointer(obj, s))
	{
		return;
	}
	FPid& pidController = *(obj->pidController);
	PidParams params = pidController.getParams();
	params.ki = atof(s);
	pidController.setParams(params);
	checkConfigError(obj, s);
	getKi(obj, s);
}

void getKi(CommandParserObjects* obj, char* s)
{
	if (handlerHasNullPointer(obj, s))
	{
		return;
	}
	FPid& pidController = *(obj->pidController);
	PidParams params = pidController.getParams();
	obj->printer->printf(KI_TOKEN SET_TOKEN "%f" EOL, params.ki);
}

void setKd(CommandParserObjects* obj, char* s)
{
	if (handlerHasNullPointer(obj, s))
	{
		return;
	}
	FPid& pidController = *(obj->pidController);
	PidParams params = pidController.getParams();
	params.kd = atof(s);
	pidController.setParams(params);
	checkConfigError(obj, s);
	getKd(obj, s);
}

void getKd(CommandParserObjects* obj, char* s)
{
	if (handlerHasNullPointer(obj, s))
	{
		return;
	}
	FPid& pidController = *(obj->pidController);
	PidParams params = pidController.getParams();
	obj->printer->printf(KD_TOKEN SET_TOKEN "%f" EOL, params.kd);
}

void getKs(CommandParserObjects* obj, char* s)
{
	if (handlerHasNullPointer(obj, s))
	{
		return;
	}
	FPid& pidController = *(obj->pidController);
	PidParams params = pidController.getParams();
	obj->printer->printf(KD_TOKEN SET_TOKEN "%f, " KP_TOKEN SET_TOKEN "%f, " KI_TOKEN SET_TOKEN "%f" EOL, params.kd, params.kp, params.ki);
}

void getKsLimits(CommandParserObjects* obj, char* s)
{
	if (handlerHasNullPointer(obj, s))
	{
		return;
	}
	FPid& pidController = *(obj->pidController);
	PidParams params = pidController.getParams();
	float loopRate = (float)US_TO_S/params.loopPeriod_us;
	obj->printer->printf("%f < " KD_TOKEN " < %f" EOL, KD_MIN(loopRate), KD_MAX(loopRate));
	obj->printer->printf("%f < " KP_TOKEN " < %f" EOL, KP_MIN(loopRate), KP_MAX(loopRate));
	obj->printer->printf("%f < " KI_TOKEN " < %f" EOL, KI_MIN(loopRate), KI_MAX(loopRate));

}

void setLoopRate(CommandParserObjects* obj, char* s)
{
	if (handlerHasNullPointer(obj, s))
	{
		return;
	}
	FPid& pidController = *(obj->pidController);
	PidParams params = pidController.getParams(); //TODO fix below
	params.loopPeriod_us = (uint32_t)(US_TO_S/atof(s));
	pidController.setParams(params);
	checkConfigError(obj, s);
	getLoopRate(obj, s);
}

void getLoopRate(CommandParserObjects* obj, char* s)
{
	if (handlerHasNullPointer(obj, s))
	{
		return;
	}
	FPid& pidController = *(obj->pidController);
	PidParams params = pidController.getParams(); //TODO fix below
	obj->printer->printf(LOOP_RATE_TOKEN SET_TOKEN "%f Hz" EOL, (float)US_TO_S/params.loopPeriod_us);
}

void setSetPointMax(CommandParserObjects* obj, char* s)
{
	if (handlerHasNullPointer(obj, s))
	{
		return;
	}
	FPid& pidController = *(obj->pidController);
	PidParams params = pidController.getParams(); //TODO fix below
	params.setPointLimit.max = inputVolts2int(atof(s));
	pidController.setParams(params);
	getSetPointLimits(obj, s);
}

void setSetPointMin(CommandParserObjects* obj, char* s)
{
	if (handlerHasNullPointer(obj, s))
	{
		return;
	}
	FPid& pidController = *(obj->pidController);
	PidParams params = pidController.getParams(); //TODO fix below
	params.setPointLimit.min = inputVolts2int(atof(s));
	pidController.setParams(params);
	getSetPointLimits(obj, s);
}

void getSetPointLimits(CommandParserObjects* obj, char* s)
{
	if (handlerHasNullPointer(obj, s))
	{
		return;
	}
	FPid& pidController = *(obj->pidController);
	PidParams params = pidController.getParams(); //TODO fix below
	obj->printer->printf(SET_POINT_MIN_TOKEN SET_TOKEN "%f V, " SET_POINT_MAX_TOKEN SET_TOKEN "%f V" EOL, int2inputVolts(params.setPointLimit.min), int2inputVolts(params.setPointLimit.max));
}

void setOutputMax(CommandParserObjects* obj, char* s)
{
	if (handlerHasNullPointer(obj, s))
	{
		return;
	}
	FPid& pidController = *(obj->pidController);
	PidParams params = pidController.getParams(); //TODO fix below
	params.outputLimit.max = inputVolts2int(atof(s));
	pidController.setParams(params);
	getOutputLimits(obj, s);
	}

void setOutputMin(CommandParserObjects* obj, char* s)
{
	if (handlerHasNullPointer(obj, s))
	{
		return;
	}
	FPid& pidController = *(obj->pidController);
	PidParams params = pidController.getParams(); //TODO fix below
	params.outputLimit.min = inputVolts2int(atof(s));
	pidController.setParams(params);
	getOutputLimits(obj, s);
}

void getOutputLimits(CommandParserObjects* obj, char* s)
{
	if (handlerHasNullPointer(obj, s))
	{
		return;
	}
	FPid& pidController = *(obj->pidController);
	PidParams params = pidController.getParams(); //TODO fix below
	obj->printer->printf( OUTPUT_MIN_TOKEN SET_TOKEN "%f V, " OUTPUT_MAX_TOKEN SET_TOKEN "%f V" EOL, int2inputVolts(params.outputLimit.min), int2inputVolts(params.outputLimit.max));
}

void setPointIntHandler(CommandParserObjects* obj, char* s)
{
	if (handlerHasNullPointer(obj, s))
	{
		return;
	}
	if (strstr(s, SET_TOKEN) != nullptr)
	{
		setSetPointInt(obj, s);
	}
	else if (strstr(s, GET_TOKEN) != nullptr)
	{
		getSetPointsInt(obj, s);
	}
}

void setPointFloatHandler(CommandParserObjects* obj, char* s)
{
	if (handlerHasNullPointer(obj, s))
	{
		return;
	}
	if (strstr(s, SET_TOKEN) != nullptr)
	{
		setSetPointFloat(obj, s);
	}
	else if (strstr(s, GET_TOKEN) != nullptr)
	{
		getSetPointsFloat(obj, s);
	}
}

void setSetPointInt(CommandParserObjects* obj, char* s)
{
	if (handlerHasNullPointer(obj, s))
	{
		return;
	}
	char* delimiter = strstr(s, SET_TOKEN);
	if (delimiter == nullptr)
	{
		return;
	}

	*delimiter = '\0';
	uint16_t value = atoi((delimiter+1));
	uint16_t index = atoi(s);

	if (index >= 0 && index < NUM_SETPOINTS)
	{
		obj->setpointManager->setSetPoint(index, value);
	}
	
	*delimiter = GET_TOKEN[0];
	getSetPointsInt(obj, s);
}

void setSetPointFloat(CommandParserObjects* obj, char* s)
{
	if (handlerHasNullPointer(obj, s))
	{
		return;
	}

	char* delimiter = strstr(s, SET_TOKEN);
	if (delimiter == nullptr)
	{
		return;
	}

	*delimiter = '\0';
	float value = atof((delimiter+1));
	uint16_t index = atoi(s);

	if (index >= 0 && index < NUM_SETPOINTS)
	{
		obj->setpointManager->setSetPoint(index, inputVolts2int(value));
	}
	
	*delimiter = GET_TOKEN[0];
	getSetPointsFloat(obj, s);
}

void getSetPointsInt(CommandParserObjects* obj, char* s)
{
	if (handlerHasNullPointer(obj, s))
	{
		return;
	}
	char* delimiter = strstr(s, GET_TOKEN);
	uint8_t minIndex = 0;
	uint8_t maxIndex = NUM_SETPOINTS;
	if (delimiter != nullptr && delimiter - s != 0)
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
		const char* isActiveSetpoint = index == obj->setpointManager->getSetPointIndex() ? " <-" : "";
		obj->printer->printf(SET_POINT_INT_TOKEN "%i" SET_TOKEN "%i%s" EOL, index, obj->setpointManager->getSetPoint(index), isActiveSetpoint);
	}
}

void getSetPointsFloat(CommandParserObjects* obj, char* s)
{
	if (handlerHasNullPointer(obj, s))
	{
		return;
	}
	char* delimiter = strstr(s, GET_TOKEN);
	uint8_t minIndex = 0;
	uint8_t maxIndex = NUM_SETPOINTS;
	if (delimiter != nullptr && delimiter - s != 0)
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
		const char* isActiveSetpoint = index == obj->setpointManager->getSetPointIndex() ? " <-" : "";
		obj->printer->printf(SET_POINT_FLOAT_TOKEN "%i" SET_TOKEN "%f V%s" EOL, index, int2inputVolts(obj->setpointManager->getSetPoint(index)), isActiveSetpoint);
	}
}

void setReadAverages(CommandParserObjects* obj, char* s)
{
	if (handlerHasNullPointer(obj, s))
	{
		return;
	}
	float readAverages = atof(s);
	*(obj->readAveragesPower) = (uint8_t)log2(readAverages);
	getReadAverages(obj, s);
}

void getReadAverages(CommandParserObjects* obj, char* s)
{
	if (handlerHasNullPointer(obj, s))
	{
		return;
	}
	obj->printer->printf(READ_AVERAGES_TOKEN SET_TOKEN "%i" EOL, 1 << *(obj->readAveragesPower));
}

void printFeedForward(CommandParserObjects* obj, char* s)
{
	if (handlerHasNullPointer(obj, s))
	{
		return;
	}
	int32_t length;
	uint16_t* values = obj->pidController->getFeedForwardReadings(length);
	for (int32_t i = 0; i < length; i++)
	{
		obj->printer->printf(FEEDFORWARD_TOKEN " %i: %i\n", i*length, values[i]);
	}
}

void calibrateFeedForward(CommandParserObjects* obj, char* s)
{
	if (handlerHasNullPointer(obj, s))
	{
		return;
	}
	obj->pidController->updateFeedForward();
	obj->printer->printf("Calibrated!" EOL);
}

void eepromReadWrite(CommandParserObjects* obj, char* s)
{
	if (handlerHasNullPointer(obj, s))
	{
		return;
	}
	
	if (strcmp(s, EEPROM_READ_TOKEN) == 0)
	{
		obj->eepromManager->load();
	}
	else if (strcmp(s, EEPROM_WRITE_TOKEN) == 0)
	{
		obj->eepromManager->save();
	}
	else
	{
		obj->printer->printf("%s not recognized" EOL, s);
		return;
	}
	obj->printer->printf(EEPROM_TOKEN SET_TOKEN "%s" EOL, s);
}

void resetPidState(CommandParserObjects* obj, char* s)
{
	if (handlerHasNullPointer(obj, s))
	{
		return;
	}
	obj->pidController->reset();
}

void setPidEnable(CommandParserObjects* obj, char* s)
{
	if (handlerHasNullPointer(obj, s))
	{
		return;
	}
	uint8_t value = atoi(s);
	obj->pidController->setPidActive(value);
	getPidEnable(obj, s);
}

void getPidEnable(CommandParserObjects* obj, char* s)
{
	if (handlerHasNullPointer(obj, s))
	{
		return;
	}
	bool active = obj->pidController->getPidState().active;
	obj->printer->printf(PID_ENABLE_TOKEN SET_TOKEN "%i" EOL, active);
}

void setOutput(CommandParserObjects* obj, char* s)
{
	if (handlerHasNullPointer(obj, s))
	{
		return;
	}
	obj->pidController->setOutputOpenLoop(outputVolts2int(atof(s)));
	getOutput(obj, s);
}

void getOutput(CommandParserObjects* obj, char* s)
{
	if (handlerHasNullPointer(obj, s))
	{
		return;
	}
	uint16_t val = obj->pidController->getPidState().output;
	obj->printer->printf(SET_OUTPUT_TOKEN SET_TOKEN "%f V" EOL, int2outputVolts(val));
}

void getFeedback(CommandParserObjects* obj, char* s)
{
	if (handlerHasNullPointer(obj, s))
	{
		return;
	}
	uint16_t val = obj->pidController->getFeedbackValue();
	obj->printer->printf(GET_FEEDBACK_TOKEN SET_TOKEN "%f V" EOL, int2outputVolts(val));
}

void setPrintOutput(CommandParserObjects* obj, char* s)
{
	if (handlerHasNullPointer(obj, s))
	{
		return;
	}
	obj->printState->printOutput = atoi(s);
}

void setPrintPeriod(CommandParserObjects* obj, char* s)
{
	if (handlerHasNullPointer(obj, s))
	{
		return;
	}
	obj->printState->printPeriod_ms = atoi(s);
	obj->printer->printf(PRINT_PERIOD_TOKEN SET_TOKEN "%i ms" EOL, obj->printState->printPeriod_ms);
}

void checkConfigError(CommandParserObjects* obj, char *s)
{
	if (handlerHasNullPointer(obj, s))
	{
		return;
	}
	if (obj->pidController->getError())
	{
		obj->printer->printf("There was a configuration error!" EOL);
	}
}

void getLogState(CommandParserObjects* obj, char* s)
{
	if (handlerHasNullPointer(obj, s))
	{
		return;
	}
	DataLog& log = *(obj->log);

	if (log.state == LOG_OFF)
	{
		obj->printer->printf(LOG_TOKEN SET_TOKEN LOG_OFF_TOKEN EOL);
	}
	else if(log.state == LOG_SINGLE)
	{
		obj->printer->printf(LOG_TOKEN SET_TOKEN LOG_SINGLE_TOKEN EOL);
	}
	else if (log.state == LOG_CONTINUOUS)
	{
		obj->printer->printf(LOG_TOKEN SET_TOKEN LOG_CONTINUOUS_TOKEN EOL);
	}
}

void logSetter(CommandParserObjects* obj, char* s)
{
	if (handlerHasNullPointer(obj, s))
	{
		return;
	}
	DataLog& log = *(obj->log);

	if (strcmp(s, LOG_OFF_TOKEN) == 0)
	{
		log.state = LOG_OFF;
	}
	else if(strcmp(s, LOG_SINGLE_TOKEN) == 0)
	{
		log.state = LOG_SINGLE;
		log.sinceLogStart = 0;
	}
	else if (strcmp(s, LOG_CONTINUOUS_TOKEN) == 0)
	{
		log.state = LOG_CONTINUOUS;
		log.sinceLogStart = 0;
	}
	else
	{
		obj->printer->printf("%s not recognized" EOL, s);
		return;
	}

	getLogState(obj, s);
}

void logGetter(CommandParserObjects* obj, char* s)
{
	if (handlerHasNullPointer(obj, s))
	{
		return;
	}
	DataLog& log = *(obj->log);
	getLogState(obj, s);
	#if RECORD_SETPOINT
		obj->printer->printf("setpoint %i\t", log.setpoint.count());
		uint32_t setpointTimeSum = 0;
		uint16_t lastSetpointTime = 0;
	#endif
	#if RECORD_FEEDBACK
		obj->printer->printf("feedback %i\t", log.feedback.count());
		uint32_t feedbackTimeSum = 0;
		uint16_t lastFeedbackTime = 0;
	#endif
	#if RECORD_OUTPUT
		obj->printer->printf("output %i\t", log.output.count());
		uint32_t outputTimeSum = 0;
		uint16_t lastOutputTime = 0;
	#endif
	obj->printer->printf("\n");
	// TODO this needs a large rewrite
	// create a function that prints one of these all at once, then the next instead of jamming all onto the same line

	while (
	#if RECORD_SETPOINT
		(log.setpoint.count() > 0) ||
	#endif
	#if RECORD_FEEDBACK
		(log.feedback.count() > 0) ||
	#endif
	#if RECORD_OUTPUT
		(log.output.count() > 0) ||
	#endif
	false )
	{
		obj->printer->printf(LOG_TOKEN);
		#if RECORD_SETPOINT
			if (log.setpoint.count() > 0 && log.setpointTime.count() > 0)
			{
				uint16_t time = log.setpointTime.pop_back();
				if (time < lastSetpointTime)
				{
					setpointTimeSum += 1 << 16;
				}
				lastSetpointTime = time;
				obj->printer->printf("\t" SETPOINT_TOKEN TIME_TOKEN ": %i\t" SETPOINT_TOKEN ": %i", setpointTimeSum + time, log.setpoint.pop_back());
			}
		#endif
		#if RECORD_FEEDBACK
			if (log.feedback.count() > 0 && log.feedbackTime.count() > 0)
			{
				uint16_t time = log.feedbackTime.pop_back();
				if (time < lastFeedbackTime)
				{
					feedbackTimeSum += 1 << 16;
				}
				lastFeedbackTime = time;
				obj->printer->printf("\t" FEEDBACK_TOKEN TIME_TOKEN ": %i\t" FEEDBACK_TOKEN ": %i", feedbackTimeSum + time, log.feedback.pop_back());
			}
		#endif
		#if RECORD_OUTPUT
			if (log.output.count() > 0 && log.outputTime.count() > 0)
			{
				uint16_t time = log.outputTime.pop_back();
				if (time < lastOutputTime)
				{
					outputTimeSum += 1 << 16;
				}
				lastOutputTime = time;
				obj->printer->printf("\t" OUTPUT_TOKEN TIME_TOKEN ": %i\t" OUTPUT_TOKEN ": %i", outputTimeSum + time, log.output.pop_back());
			}
		#endif
			obj->printer->printf("\n");
	}

	log.sinceLogStart = 0;
}

void getName(CommandParserObjects* obj, char* s)
{
	if (handlerHasNullPointer(obj, s))
	{
		return;
	}
	obj->printer->printf(NAME_TOKEN SET_TOKEN NAME EOL);
}