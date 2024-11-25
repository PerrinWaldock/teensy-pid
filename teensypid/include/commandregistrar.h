#ifndef COMMAND_REGISTRAR_H
#define COMMAND_REGISTRAR_H

#include <Arduino.h>
#include "utils.hpp"

#define MAX_COMMAND_NUMBER 64
/*
NOTE: this could be implemented using a dictionary structure to get faster performance
TODO alternatively, remove the fancy addGetter and addSetter implementation, implement all in functions as before
*/

bool processCommand(char* commandString);
bool addCommand(const char * key, void (*callback)(void*, char*), void* firstArgument, const char * helpString);

void printHelp();

#endif