#ifndef EEPROM_MANAGER
#define EEPROM_MANAGER

#include "fpid.h"

class EepromManager
{
    public:
        EepromManager::EepromManager(FPid& feedbackController);
        void EepromManager::save();
        void EepromManager::load();
    private:
        FPid* feedbackController;
};

#endif