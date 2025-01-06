#ifndef COMMANDS_H
#define COMMANDS_H

#include "fpid.h"
#include "eeprom-manager.h"
#include "setpointmanager.h"
#include "datalogger.h"
#include "printstate.h"

struct CommandParserObjects {
    FPid* pidController;
    EepromManager* eepromManager;
    SetpointManager* setpointManager;
    uint8_t* readAveragesPower;
    PrintState* printState;
    DataLog* log;
    Print* printer;
};

class CommandParser
{
    public:
        CommandParser(CommandParserObjects objects);
        bool parse(char* string);
    
    private:
        CommandParserObjects objects;
};

#endif