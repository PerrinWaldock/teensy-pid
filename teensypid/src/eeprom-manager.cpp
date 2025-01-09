#include "eeprom-manager.h"
#include <EEPROM.h>

#ifndef FEED_FORWARD
#define FEED_FORWARD false
#endif

const uint16_t EEPROM_READ_AVERAGES_ADDRESS = 0;
const uint16_t EEPROM_SET_POINTS_ADDRESS = 8;
const uint16_t EEPROM_PARAMS_ADDRESS = 64;
const uint16_t EEPROM_FEED_FORWARD_ADDRESS = 128;


EepromManager::EepromManager(FPid& feedbackController, uint16_t setPoints[], uint8_t& readAveragesPower)
{
    this->feedbackController = &feedbackController;
	this->setPoints = setPoints;
	this->readAveragesPower = &readAveragesPower;
}

void EepromManager::save()
{
    EEPROM.put(EEPROM_READ_AVERAGES_ADDRESS, *readAveragesPower);

    for (uint8_t i = 0; i < NUM_SETPOINTS; i++)
    {
        uint16_t address = EEPROM_SET_POINTS_ADDRESS+(i*sizeof(*setPoints));
        EEPROM.put(address, setPoints[i]);
    }

    PidParams params = feedbackController->getParams();
    EEPROM.put(EEPROM_PARAMS_ADDRESS, params);
    
    #if FEED_FORWARD
        int32_t readingsLength;
        uint16_t* readings = feedbackController->getFeedForwardReadings(readingsLength);
        for(uint16_t j = 0; j < readingsLength; j++)
        {
            uint16_t address = EEPROM_FEED_FORWARD_ADDRESS+(j*sizeof(*readings)); 
            EEPROM.put(address, readings[j]);
        }
    #endif
}

void EepromManager::load()
{
    PidParams params = feedbackController->getParams();
    EEPROM.get(EEPROM_PARAMS_ADDRESS, params);
    feedbackController->setParams(params);
    EEPROM.get(EEPROM_READ_AVERAGES_ADDRESS, *readAveragesPower);


    for (uint8_t i = 0; i < NUM_SETPOINTS; i++)
    {
        uint16_t address = EEPROM_SET_POINTS_ADDRESS+(i*sizeof(*setPoints));
        EEPROM.get(address, setPoints[i]);
    }
    
    #if FEED_FORWARD
        int32_t readingsLength;
        uint16_t* readings = feedbackController->getFeedForwardReadings(readingsLength);
        for(uint16_t j = 0; j < readingsLength; j++)
        {
            uint16_t address = EEPROM_FEED_FORWARD_ADDRESS+(j*sizeof(*readings));
            EEPROM.get(address, readings[j]);
        }
        feedbackController->updateFeedForward(readings, readingsLength);
    #endif

}