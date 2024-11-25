#include "commandregistrar.h"
#include "serialmanager.h"
#include <Arduino.h>
#include <stdio.h>

#define TEMPORARY_BUFFER_LENGTH 128

void (*callbacks[MAX_COMMAND_NUMBER])(void*, char*);
void* arguments[MAX_COMMAND_NUMBER];
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
            void* arg1 = arguments[i];
            char* arg2 = commandString + strlen(keys[i]);
            callbacks[i](arg1, arg2);
            return true;
        }
    }
    snprintf(tempBuffer, TEMPORARY_BUFFER_LENGTH, "%s not recognized", commandString);
    writeLine(tempBuffer);
    
    return false;
}

bool addCommand(const char* key, void (*callback)(void*, char*), void* firstArgument, const char * helpString)
{
    if (numberOfCommands >= MAX_COMMAND_NUMBER || callback == NULL || firstArgument == NULL)
    {
        return false;
    }

    keys[numberOfCommands] = key;
    helpStrings[numberOfCommands] = helpString;
    callbacks[numberOfCommands] = callback;
    arguments[numberOfCommands] = firstArgument;
    numberOfCommands++;
    return true;
}

void printHelp()
{
    char tempBuffer[TEMPORARY_BUFFER_LENGTH];
    for (uint8_t i = 0; i < numberOfCommands; i++)
    {
        snprintf(tempBuffer, TEMPORARY_BUFFER_LENGTH, "%s : %s", keys[i], helpStrings[i]);
        writeLine(tempBuffer);
    }
}
