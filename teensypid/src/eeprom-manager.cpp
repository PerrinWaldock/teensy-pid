#include "eeprom-manager.h"
#include <EEPROM.h>

#ifndef FEED_FORWARD
#define FEED_FORWARD false
#endif

const uint16_t EEPROM_PARAMS_ADDRESS = 0;
const uint16_t EEPROM_FEED_FORWARD_ADDRESS = 512;


EepromManager::EepromManager(FPid& feedbackController)
{
    this->feedbackController = &feedbackController;
}

void EepromManager::save()
{
    PidParams params = feedbackController->getParams();
    EEPROM.put(EEPROM_PARAMS_ADDRESS, params);
    
    #if FEED_FORWARD
        int32_t readingsLength;
        uint16_t** readings;
        feedbackController->getFeedForwardReadings(readings, readingsLength);
        for(uint16_t j = 0; j < readingsLength; j++)
        {
            EEPROM.put(EEPROM_FEED_FORWARD_ADDRESS+(j*sizeof(**readings)), (*readings)[j]);
        }
    #endif
}

void EepromManager::load()
{
    PidParams params;
    EEPROM.get(EEPROM_PARAMS_ADDRESS, params);
    feedbackController->setParams(params);
    
    #if FEED_FORWARD
        int32_t readingsLength;
        uint16_t** readings;
        feedbackController->getFeedForwardReadings(readings, readingsLength);
        for(uint16_t j = 0; j < readingsLength; j++)
        {
            EEPROM.get(EEPROM_FEED_FORWARD_ADDRESS+(j*sizeof(**readings)), (*readings)[j]);
        }
        feedbackController->updateFeedForward(*readings, readingsLength);
    #endif
}