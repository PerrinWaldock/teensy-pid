#ifndef EEPROM_MANAGER
#define EEPROM_MANAGER

#include "fpid.h"

class EepromManager
{
    public:
        EepromManager(FPid& feedbackController, uint16_t setPoints[], uint8_t& readAveragesPower);
        void save();
        void load();
    private:
        FPid* feedbackController;
        uint16_t* setPoints;
        uint8_t* readAveragesPower;
};

#endif