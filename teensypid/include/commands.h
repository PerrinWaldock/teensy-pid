#ifndef COMMANDS_H
#define COMMANDS_H

#include "fpid.h"
#include "eeprom-manager.h"
#include "setpointmanager.h"

class CommandParser
{
    public:
        CommandParser(FPid& pidController, EepromManager& eepromManager, SetpointManager& setpointManager, uint8_t& readAveragesPower, bool& printOutput);
        bool parse(char* string);
    
    private:
        EepromManager* eepromManager;
        FPid* pidController;
        SetpointManager* setpointManager;
        uint8_t* readAveragesPower;
        bool* printOutput;
};

#endif