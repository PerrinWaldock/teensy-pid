#include "fpid.h"
#include <Arduino.h> //TODO remove

const uint16_t SAME_SETPOINT_TOLERANCE = inputVolts2int(.05);

#ifndef PRECISE_LOOP_TIMING
#define PRECISE_LOOP_TIMING false
#endif

FPid::FPid(PidParams params, uint16_t (*getSetPoint)(void), uint16_t (*getFeedback)(void), void (*setOutput)(uint16_t))
{
    this->getSetPoint = getSetPoint;
    this->getFeedback = getFeedback;
    this->setOutput = setOutput;

    this->pidController = FastPID(params.kp, params.ki, params.kd, (float)US_TO_S/params.loopPeriod_us, OUTPUT_BITS, true);
    this->feedForward = FeedForwardHelper(FF_CALIBRATION_ARRAY_BITS, FEED_FORWARD_ARRAY_BITS, CALIBRATION_AVERAGES, getFeedback, setOutput);

    setParams(params);
}

void FPid::iterate()
{
    #if PRECISE_LOOP_TIMING
        while (!inputRailed && timeSinceLastRun < params.loopPeriod_us);
    #endif
    timeSinceLastRun = 0; // reset time
    performCalc = true;

    if (!pidActive)
    {
        lastIterationTime = 0;
        #if RECORD_FEEDBACK_ALL_ITERATIONS
            lastFeedBack = getFeedback();
            setOutput(lastOutput);
        #endif
        return;
    }

    uint16_t setPoint = (*getSetPoint)();
    int16_t setPointChange = setPoint - lastSetPoint;
    bool setPointChanged = setPointChange >= SAME_SETPOINT_TOLERANCE || setPointChange <= -1*SAME_SETPOINT_TOLERANCE;
    lastSetPoint = setPoint;
    
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
    else
    {
        inputRailed = false;
    }

    if (inputRailed)
    {
        #if ! FEED_FORWARD && CLEAR_INTEGRAL_WHEN_RAILED
            pidController.clear();
        #endif
        #if RECORD_FEEDBACK_ALL_ITERATIONS
            lastFeedBack = getFeedback();
        #endif
        lastIterationTime = timeSinceLastRun;
        return;
    }
    #if INPUT_MODE == DIGITAL_INPUT && FEED_FORWARD
        if (setPointChanged)
        {
            performCalc = false;
        }
    #endif

    #if FEED_FORWARD
        int32_t output = feedForward.getArray()[setPoint];
    #else
        int32_t output = HALF_MAX_OUTPUT;
    #endif

    if (performCalc)
    {
        lastFeedBack = (*getFeedback)();;
        lastPid = pidController.step(setPoint - HALF_MAX_INPUT, lastFeedBack - HALF_MAX_INPUT);
        
        #if NEGATIVE_OUTPUT
            lastPid *= -1;
        #endif

        output += lastPid;
    }
    else
    {
        #if RECORD_FEEDBACK_ALL_ITERATIONS
            lastFeedBack = getFeedback();
        #endif
    }

    setOutputWithLimits(output);
    lastIterationTime = timeSinceLastRun;
}


void FPid::setOutputWithLimits(uint32_t output)
{
    if (output > params.outputLimit.max)
    {
        output = params.outputLimit.max;
    }
    else if(output < params.outputLimit.min)
    {
        output = params.outputLimit.min;
    }

    lastOutput = output;
    (*setOutput)(output);
}

void FPid::setParams(PidParams ps)
{
    params = ps;
    pidController.configure(params.kp, params.ki, params.kd, (float)US_TO_S/params.loopPeriod_us, OUTPUT_BITS, true);
    reset();
}

bool FPid::getError()
{
    return pidController.err();
}

void FPid::reset()
{
    pidController.clear();
}

PidParams FPid::getParams()
{
    return params;
}

void FPid::setPidActive(bool active)
{
    pidActive = active;
    if (!pidActive)
    {
        reset();
    }
}

PidState FPid::getPidState()
{
    return PidState
    {
        lastSetPoint,
        lastFeedBack,
        lastOutput,
        lastPid,
        lastIterationTime,
        pidActive,
        performCalc,
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

uint16_t* FPid::getFeedForwardReadings(int32_t& readingsLength)
{
    return feedForward.getReadings(readingsLength);
}

uint16_t FPid::getFeedForwardValue()
{
    #if FEED_FORWARD
        return feedForward.getArray()[lastSetPoint];
    #else
        return 0;
    #endif
}

void FPid::setOutputOpenLoop(uint16_t value)
{
    pidActive = false;
    reset();
    setOutputWithLimits(value);
}

uint16_t FPid::getFeedbackValue()
{
    return (*getFeedback)();
}
