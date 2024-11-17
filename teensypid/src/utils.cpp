#include "utils.hpp"

template<typename T>
T stringToType(char* str)
{
    switch(typeid(T))
    {
        case typeid(char*):
            return str;
        case typeid(int):
            return atoi(str);
        case typeid(float):
            return atof(str);
        default:
            return (T)(str); // TODO better handling
    }
}

template<typename T>
char* typeToString(T var)
{
    char[64] buffer;
    switch(typeid(T))
    {
        case typeid(char*):
            return str;
        case typeid(int):
            sprintf("%i", buffer, var);
            return buffer;
        case typeid(float):
            sprintf("%f", buffer, var);
            return buffer;
        default:
            return var.toString(); // TODO better handling
    }
}