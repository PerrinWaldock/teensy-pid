#ifndef FEED_FORWARD_H
#define FEED_FORWARD_H

#include "settings.h"
#include "utils.h"

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
#ifndef MAX_INPUT
    const uint32_t MAX_INPUT = 1 << MAX_FEED_FORWARD_BITS;
#endif

const uint32_t MAX_ARRAY_SIZE = MAX_INPUT;

class FeedForwardHelper
{
    public:
        FeedForwardHelper::FeedForwardHelper();
        FeedForwardHelper::FeedForwardHelper(uint8_t calibrationBits, uint8_t feedForwardBits, uint8_t numberOfAverages, uint16_t (*getFeedback)(void), void (*setOutput)(uint16_t));

        void FeedForwardHelper::setReadings(uint16_t* readings, int32_t length);
        uint16_t* FeedForwardHelper::getReadings(int32_t& length);
        
        uint16_t* FeedForwardHelper::getArray(int32_t& length);
        uint16_t* FeedForwardHelper::getArray();

        void FeedForwardHelper::measure();
        Extrema FeedForwardHelper::calculateArrayAndSetpointLimits(Extrema outputLimits);
        Extrema FeedForwardHelper::getExtrema();


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