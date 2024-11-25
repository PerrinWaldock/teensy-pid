#ifndef COMMANDS_H
#define COMMANDS_H

#include "fpid.h"
#include "eeprom-manager.h"

class CommandParser
{
    public:
        CommandParser(FPid& pidController, EepromManager& eepromManager, uint16_t setPoints[], uint8_t& readAveragesPower, bool& printOutput);
        void parse(char* string);
    
    private:
        EepromManager* eepromManager;
        FPid* pidController;
        uint16_t* setPoints;
        uint8_t* readAveragesPower;
        bool* printOutput;
};

#endif