#ifndef FEED_FORWARD_H
#define FEED_FORWARD_H

#include "settings.h"
#include "utils.hpp"

#define MAX_FEED_FORWARD_BITS 16

#ifndef CALIBRATION_AVERAGES
    #define CALIBRATION_AVERAGES 16
#endif
#ifndef CALIBRATION_SETTLE_DELAY_US
	#define CALIBRATION_SETTLE_DELAY_US 10
#endif
#ifndef CALIBRATION_PRE_SETTLE_DELAY_US
	#define CALIBRATION_PRE_SETTLE_DELAY_US 10000
#endif

const uint32_t MAX_ARRAY_SIZE = MAX_INPUT;

class FeedForwardHelper
{
    public:
        FeedForwardHelper();
        FeedForwardHelper(uint8_t calibrationBits, uint8_t feedForwardBits, uint8_t numberOfAverages, uint16_t (*getFeedback)(void), void (*setOutput)(uint16_t));

        void setReadings(uint16_t* readings, int32_t length);
        uint16_t* getReadings(int32_t& length);
        
        uint16_t* getArray(int32_t& length);
        uint16_t* getArray();

        void measure();
        Extrema calculateArrayAndSetpointLimits(Extrema outputLimits);
        Extrema getExtrema();


    private:
        uint16_t readings[MAX_ARRAY_SIZE];
        uint16_t array[MAX_ARRAY_SIZE];
        uint32_t readingsSize = MAX_ARRAY_SIZE;
        uint32_t arraySize = MAX_ARRAY_SIZE;
        uint8_t adcReadings;

        uint16_t (*getFeedback)(void);
        void (*setOutput)(uint16_t);
        uint16_t feedForwardCalibrationOutput(uint16_t);
};

#endif