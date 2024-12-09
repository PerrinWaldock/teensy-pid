#include "fpid.h"

const uint16_t SAME_SETPOINT_TOLERANCE = inputVolts2int(.05);

#ifndef PRECISE_LOOP_TIMING
#define PRECISE_LOOP_TIMING false
#endif

FPid::FPid(PidParams params, uint16_t (*getSetPoint)(void), uint16_t (*getFeedback)(void), void (*setOutput)(uint16_t))
{
    this->getSetPoint = getSetPoint;
    this->getFeedback = getFeedback;
    this->setOutput = setOutput;

    this->pidController = FastPID(params.kp, params.ki, params.kd, 1000000/params.loopPeriod_us, OUTPUT_BITS, true);
    this->feedForward = FeedForwardHelper(FF_CALIBRATION_ARRAY_BITS, FEED_FORWARD_ARRAY_BITS, CALIBRATION_AVERAGES, getFeedback, setOutput);

    Serial.printf("FPid(): kd:%f\n", params.kd);
    setParams(params);
    Serial.printf("FPid() post set: kd:%f\n", this->params.kd);
    Serial.printf("FPid() post set accesor: kd:%f\n", getParams().kd);
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
    #if PRECISE_LOOP_TIMING
        while (!inputRailed && timeSinceLastRun < params.loopPeriod_us);
    #endif
    timeSinceLastRun = 0; // reset time

    if (!pidActive)
    {
        // TODO consider still recording feedback value here
        return;
    }

    uint16_t setPoint = (*getSetPoint)();

    //TODO check setpoint changed across a range
    int16_t setPointChange = setPoint - lastSetPoint;
    bool setPointChanged = setPointChange <= SAME_SETPOINT_TOLERANCE && setPointChange >= -1*SAME_SETPOINT_TOLERANCE;
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
    lastIterationTime = timeSinceLastRun;
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

void FPid::setParams(PidParams ps)
{
    params = ps;
    pidController.configure(params.kp, params.ki, params.kd, 1000000/params.loopPeriod_us, OUTPUT_BITS, true);
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
    Serial.printf("getting params\n");
    Serial.printf("params kd -> %f\n", params.kd);
    return params;
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
        lastIterationTime,
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