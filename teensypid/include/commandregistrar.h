#ifndef COMMAND_REGISTRAR_H
#define COMMAND_REGISTRAR_H

#include <Arduino.h>
#include "utils.h"

#define MAX_COMMAND_NUMBER 64
#define SET_TOKEN "="
#define GET_TOKEN "?"
/*
NOTE: this could be implemented using a dictionary structure to get faster performance
TODO alternatively, remove the fancy addGetter and addSetter implementation, implement all in functions as before
*/

bool processCommand(char* commandString);
bool addCommand(const char * key, void (*callback)(char*), const char * helpString);

#define addGetter(type, key, value, helpString) {\
    addCommand(key GET_TOKEN, [](char* s) { \
    char printBuffer[64]; \
    sprintf("%s%s", printBuffer, key SET_TOKEN, typeToString<type>(value)); \
    sprint(printBuffer); \
    }, helpString); \
}

#define addSetter(type, key, value, helpString) {\
    addCommand(key SET_TOKEN, [](char* s) { \
    value = stringToType<type>(s); \
    char printBuffer[64]; \
    sprintf("%s%s", printBuffer, key SET_TOKEN, typeToString<type>(value)); \
    sprint(printBuffer); \
    }, helpString); \
}

#define addGetterAndSetter(type, key, value, helpString) {\
    addGetter(type, key, value, "gets " helpString);\
    addSetter(type, key, value, "sets " helpString);\
}

void printHelp();

#endif