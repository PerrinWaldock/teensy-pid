#ifndef COMMANDS_H
#define COMMANDS_H

#include "fpid.h"
#include "eeprom-manager.h"
#include "setpointmanager.h"
#include "datalogger.h"

/*
TODO
    create a CommandParserObjects struct that includes a print function
    pass that struct to the event handlers
*/

struct CommandParserObjects {
    FPid* pidController;
    EepromManager* eepromManager;
    SetpointManager* setpointManager;
    uint8_t* readAveragesPower;
    bool* printOutput;
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