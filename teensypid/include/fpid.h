#ifndef FPID_H
#define FPID_H

#include "pidparams.h"
#include "feedforward.h"
#include "settings.h"
#include <FastPID.h>

/*TODO create separate file that handles feedforward. 
has settings like FF_CALIBRATION_ARRAY_LENGTH, CALIBRATION_PRE_SETTLE_DELAY_US, etc
has getFeedForwardReadings, calibrateFeedForward, calibrationExtrema

also need separate EEPROM writer
*/

#define OUTPUT_BITS DAC_BITS

typedef struct
{
    uint16_t setPoint;
    uint16_t feedBack;
    uint16_t output;
    bool active;
    bool railed;
} PidState;

class FPid
{
    public:
        FPid::FPid(PidParams params, uint16_t (*getSetPoint)(void), uint16_t (*getFeedback)(void), void (*setOutput)(uint16_t));
        void FPid::iterate();

        void FPid::setParams(PidParams params);
        PidParams FPid::getParams();
        
        void FPid::setPidActive(bool active);
        PidState FPid::getPidState();

        void FPid::updateFeedForward(); //via measurement
        void FPid::updateFeedForward(uint16_t* readings, int32_t readingsLength); //from array
        void FPid::getFeedForwardReadings(uint16_t** readings, int32_t& readingsLength);
        
    private:
        uint16_t lastSetPoint = 0;
        uint16_t lastFeedBack = 0;
        uint16_t lastOutput = 0;
        bool pidActive = false;
        bool inputRailed = false;
        elapsedMicros timeSinceLastRun;
        PidParams params;
        FastPID pidController;
        FeedForwardHelper feedForward;

        uint16_t (*getSetPoint)(void);
        uint16_t (*getFeedback)(void);
        void (*setOutput)(uint16_t);
        void setOutputWithLimits(uint32_t outPut);
};

#endif