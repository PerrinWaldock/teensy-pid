#include "commands.h"
#include "commandregistrar.h"

// TODO scrap the macros, just use the standard function calls because we need to deal with getters and setters
// TODO add void pointer to function handler so instances can be handled

#define addVoltToIntGetter(key, value, helpString) {\
    addCommand(key GET_TOKEN, [](char* s) { \
    char printBuffer[64]; \
    sprintf("%s%s", printBuffer, key SET_TOKEN, typeToString<type>(value)); \
    sprint(printBuffer); \
    }, helpString); \
}

CommandParser::CommandParser(FPid& pidController, EepromManager& eepromManager)
{
	this->pidController = &pidController;
	this->eepromManager = &eepromManager;
}
