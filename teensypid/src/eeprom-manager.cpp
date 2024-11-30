#include "eeprom-manager.h"
#include <EEPROM.h>

#ifndef FEED_FORWARD
#define FEED_FORWARD false
#endif

const uint16_t READ_AVERAGES_ADDRESS = 0;
const uint16_t EEPROM_PARAMS_ADDRESS = 4;
const uint16_t EEPROM_FEED_FORWARD_ADDRESS = 512;


EepromManager::EepromManager(FPid& feedbackController, uint16_t setPoints[], uint8_t& readAveragesPower)
{
    this->feedbackController = &feedbackController;
	this->setPoints = setPoints;
	this->readAveragesPower = &readAveragesPower;
}

void EepromManager::save()
{
    PidParams params = feedbackController->getParams();
    EEPROM.put(EEPROM_PARAMS_ADDRESS, params);
    EEPROM.put(READ_AVERAGES_ADDRESS, *readAveragesPower);
    
    #if FEED_FORWARD
        int32_t readingsLength;
        uint16_t* readings = feedbackController->getFeedForwardReadings(readingsLength);
        for(uint16_t j = 0; j < readingsLength; j++)
        {
            EEPROM.put(EEPROM_FEED_FORWARD_ADDRESS+(j*sizeof(*readings)), readings[j]);
        }
    #endif
}

void EepromManager::load()
{
    PidParams params;
    EEPROM.get(EEPROM_PARAMS_ADDRESS, params);
    feedbackController->setParams(params);
    EEPROM.get(READ_AVERAGES_ADDRESS, *readAveragesPower);
    
    #if FEED_FORWARD
        int32_t readingsLength;
        uint16_t* readings = feedbackController->getFeedForwardReadings(readingsLength);
        for(uint16_t j = 0; j < readingsLength; j++)
        {
            EEPROM.get(EEPROM_FEED_FORWARD_ADDRESS+(j*sizeof(*readings)), readings[j]);
        }
        feedbackController->updateFeedForward(readings, readingsLength);
    #endif
}