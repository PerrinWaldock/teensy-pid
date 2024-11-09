#include "commandregistrar.h"

#define TEMPORARY_BUFFER_LENGTH 128

void (*callbacks[MAX_COMMAND_NUMBER])(char*);
const char* keys[MAX_COMMAND_NUMBER];
const char* helpStrings[MAX_COMMAND_NUMBER];
uint8_t numberOfCommands = 0;

bool processCommand(char* commandString)
{
    char tempBuffer[TEMPORARY_BUFFER_LENGTH];
    for (uint8_t i = 0; i < numberOfCommands; i++)
    {
        if (strncmp(commandString, keys[i], strlen(keys[i])) == 0)
        {
            char* argument = commandString + strlen(keys[i]);
            callbacks[i](argument);
            return true;
        }
    }
    snprintf(tempBuffer, TEMPORARY_BUFFER_LENGTH, "%s not recognized", commandString);
    sprint(tempBuffer);
    
    return false;
}

bool addCommand(const char* key, void (*callback)(char*), const char * helpString)
{
    if (numberOfCommands >= MAX_COMMAND_NUMBER)
    {
        return false;
    }

    keys[numberOfCommands] = key;
    helpStrings[numberOfCommands] = helpString;
    callbacks[numberOfCommands] = callback;
    numberOfCommands++;
    return true;
}

void printHelp()
{
    char tempBuffer[TEMPORARY_BUFFER_LENGTH];
    for (uint8_t i = 0; i < numberOfCommands; i++)
    {
        snprintf(tempBuffer, TEMPORARY_BUFFER_LENGTH, "%s : %s", keys[i], helpStrings[i]);
        sprint(tempBuffer);
    }
}
