#ifndef COMMANDS_H
#define COMMANDS_H

#include "fpid.h"
#include "eeprom-manager.h"

/*
TODO replace <float>V in string with bitshifted integer ?
    alternate:
        separate commands for voltage and integer
        add 
*/

class CommandParser
{
    public:
        CommandParser::CommandParser(FPid& pidController, EepromManager& eepromManager);
        void CommandParser::parse(char* string);
    
    private:
        EepromManager* eepromManager;
        FPid* pidController;

};

#endif