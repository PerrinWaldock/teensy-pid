#include "fpid.h"

FPid::FPid(PidParams params, uint16_t (*getSetPoint)(void), uint16_t (*getFeedback)(void), void (*setOutput)(uint16_t))
{
    this->getSetPoint = getSetPoint;
    this->getFeedback = getFeedback;
    this->setOutput = setOutput;

    this->pidController = FastPID(params.kp, params.ki, params.kd, params.loopRate, OUTPUT_BITS, true);
    this->feedForward = FeedForwardHelper(FF_CALIBRATION_ARRAY_BITS, FEED_FORWARD_ARRAY_BITS, CALIBRATION_AVERAGES, getFeedback, setOutput);
    setParams(params);
}

// TODO add timing externally
/*
#if TIME_FEEDBACK_LOOP
    elapsedMicros timeSinceLoopStart;
#endif
*/

void FPid::iterate()
{
    // wait until correct time to run TODO better implementation
    while (!inputRailed && timeSinceLastRun < params.loopRate);
    timeSinceLastRun -= timeSinceLastRun; // reset time

    if (!pidActive)
    {
        // TODO consider still recording feedback value here
        return;
    }

    uint16_t setPoint = (*getSetPoint)();

    bool setPointChanged = setPoint != lastSetPoint;
    lastSetPoint = setPoint;
    
    if (setPointChanged)
    {
        if (setPoint < params.setPointLimit.min)
        {
            #if NEGATIVE_OUTPUT 
                setOutputWithLimits(params.outputLimit.max);
            #else
                setOutputWithLimits(params.outputLimit.min);
            #endif
            inputRailed = true;
        }
        else if (setPoint > params.setPointLimit.max)
        {
            #if NEGATIVE_OUTPUT 
                setOutputWithLimits(params.outputLimit.min);
            #else
                setOutputWithLimits(params.outputLimit.max);
            #endif
            inputRailed = true;
        }

        if (inputRailed)
        {
            #if ! FEED_FORWARD && CLEAR_INTEGRAL_WHEN_RAILED
                pidController.clear();
            #endif
            return;
        }
    }

    bool performPidCalculation = true;
    #if INPUT_MODE == DIGITAL_INPUT && FEED_FORWARD
        if (setPointChanged)
        {
            performPidCalculation = false;
        }
    #endif

    #if FEED_FORWARD
        int32_t output = feedForward.getArray()[setPoint];
    #else
        int32_t output = HALF_MAX_OUTPUT;
    #endif

    if (performPidCalculation)
    {
        uint16_t feedback = (*getFeedback)();
        lastFeedBack = feedback;
        // TODO record input
        int16_t step = pidController.step(setPoint - HALF_MAX_INPUT, feedback - HALF_MAX_INPUT);
        
        #if NEGATIVE_OUTPUT
            step *= -1;
        #endif

        output += step;
    }

    setOutputWithLimits(output);
}


void FPid::setOutputWithLimits(uint32_t outPut)
{
    if (outPut > params.outputLimit.max)
    {
        outPut = params.outputLimit.max;
    }
    else if(outPut < params.outputLimit.min)
    {
        outPut = params.outputLimit.min;
    }

    lastOutput = outPut;
    (*setOutput)(outPut);
}

void FPid::setParams(PidParams params)
{
    this->params = params;
    pidController.configure(params.kp, params.ki, params.kd, params.loopRate, OUTPUT_BITS, true);
    pidController.clear();
}

void FPid::setPidActive(bool active)
{
    pidActive = active;
}

PidState FPid::getPidState()
{
    return PidState
    {
        lastSetPoint,
        lastFeedBack,
        lastOutput,
        pidActive,
        inputRailed
    };
}


void FPid::updateFeedForward()
{
    bool previousActiveState = pidActive;
    pidActive = false;
    feedForward.measure();
    params.setPointLimit = feedForward.calculateArrayAndSetpointLimits(params.outputLimit);

    pidActive = previousActiveState;
}

void FPid::updateFeedForward(uint16_t* readings, int32_t readingsLength)
{
    feedForward.setReadings(readings, readingsLength);
    
    bool previousActiveState = pidActive;
    pidActive = false;
    params.setPointLimit = feedForward.calculateArrayAndSetpointLimits(params.outputLimit);

    pidActive = previousActiveState;
}

void FPid::getFeedForwardReadings(uint16_t** readings, int32_t& readingsLength)
{
    *readings = feedForward.getReadings(readingsLength);
}
