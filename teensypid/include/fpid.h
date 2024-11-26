#ifndef FPID_H
#define FPID_H

#include "pidparams.h"
#include "feedforward.h"
#include "settings.h"
#include "FastPID.h"

/*TODO create separate file that handles feedforward. 
has settings like FF_CALIBRATION_ARRAY_LENGTH, CALIBRATION_PRE_SETTLE_DELAY_US, etc
has getFeedForwardReadings, calibrateFeedForward, calibrationExtrema

also need separate EEPROM writer
*/

#define OUTPUT_BITS DAC_BITS

// TODO limits checking in commands
#define KP_MIN(f) (1/PARAM_MULT)
#define KP_MAX(f) (PARAM_MAX)
#define KI_MIN(f) (KP_MIN(f)*f)
#define KI_MAX(f) (KP_MAX(f)*f)
#define KD_MIN(f) (KD_MIN(f)/f)
#define KD_MAX(f) (KD_MAX(f)/f)


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
        FPid(PidParams params, uint16_t (*getSetPoint)(void), uint16_t (*getFeedback)(void), void (*setOutput)(uint16_t));
        void iterate();
        void reset();

        void setParams(PidParams params);
        PidParams getParams();
        
        void setPidActive(bool active);
        PidState getPidState();

        bool getError();

        void updateFeedForward(); //via measurement
        void updateFeedForward(uint16_t* readings, int32_t readingsLength); //from array
        void getFeedForwardReadings(uint16_t** readings, int32_t& readingsLength);
        uint16_t getFeedForwardValue();

        void setOutputOpenLoop(uint16_t value);
        
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