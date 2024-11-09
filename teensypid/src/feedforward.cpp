#include "feedforward.h"
#include "utils.h"

uint16_t readMultiple(uint16_t (*readData)(void), int8_t averages)
{
    if (averages <= 0)
    {
        return 0;
    }

    uint32_t sum = 0;
    for (uint8_t i = 0; i < averages; i++)
    {
        sum += readData();
    }
    return sum/averages;
}

uint16_t FeedForwardHelper::feedForwardCalibrationOutput(uint16_t x)
{
    return (uint16_t)((x*((1 << arraySize) - 1))/((1 << readingsSize) - 1));
}

FeedForwardHelper::FeedForwardHelper()
{
    readingsSize = 0;
    arraySize = 0;
}

FeedForwardHelper::FeedForwardHelper(uint8_t calibrationBits, uint8_t feedForwardBits, uint8_t numberOfAverages, uint16_t (*getFeedback)(void), void (*setOutput)(uint16_t))
{
    readingsSize = min(1 << calibrationBits, MAX_ARRAY_SIZE);
    arraySize = min(feedForwardBits, MAX_ARRAY_SIZE);
    adcReadings = numberOfAverages;
    this->getFeedback = getFeedback;
    this->setOutput = setOutput;
}

void FeedForwardHelper::setReadings(uint16_t* readings, int32_t length)
{
    readingsSize = min(length, MAX_ARRAY_SIZE);
    for (uint32_t i = 0; i < readingsSize; i++)
    {
        this->readings[i] = readings[i];
    }
}

uint16_t* FeedForwardHelper::getReadings(int32_t& length)
{
    length = readingsSize;
    return readings;
}

uint16_t* FeedForwardHelper::getArray(int32_t& length)
{
    length = arraySize;
    return array;
}

uint16_t* FeedForwardHelper::getArray()
{
    return array;
}

void FeedForwardHelper::measure()
{
    setOutput(0);
	delayMicroseconds(CALIBRATION_PRE_SETTLE_DELAY_US);
    for(int32_t i = 0; i < readingsSize; i++)
    {
        uint16_t outputValue = feedForwardCalibrationOutput(i);
        setOutput(outputValue);
        delayMicroseconds(CALIBRATION_SETTLE_DELAY_US); //wait for output to stabilize
        readings[i] = readMultiple(getFeedback, CALIBRATION_AVERAGES);
    }

    //repeat and average readings (this time from opposite direction)
	delayMicroseconds(CALIBRATION_PRE_SETTLE_DELAY_US);
    for(int32_t i = readingsSize-1; i >= 0; i--)
    {
        uint16_t outputValue = feedForwardCalibrationOutput(i);
        setOutput(outputValue);
        delayMicroseconds(CALIBRATION_SETTLE_DELAY_US); //wait for output to stabilize
        readings[i] = (readMultiple(getFeedback, CALIBRATION_AVERAGES) + readings[i])/2;
    }
}

Extrema FeedForwardHelper::calculateArrayAndSetpointLimits(Extrema outputLimits)
{
    Extrema extrema = getExtrema();
    uint16_t& minIndex = extrema.min;
    uint16_t& maxIndex = extrema.max;
    uint16_t& maxValue = readings[maxIndex];
    uint16_t& minValue = readings[minIndex];

    uint16_t minSetPoint = minValue;
    uint16_t maxSetPoint = maxValue;

    for(uint32_t desired = 0; desired < arraySize; desired++)  //iterates through calibration array
    {
        bool valueFound = false;
        if(desired > maxValue) //if desired value is out of range, linearly interpolate
        {
            array[desired] = feedForwardCalibrationOutput(maxIndex);
            valueFound = true;
        }
        else if(desired < minValue)
        {
            array[desired] = feedForwardCalibrationOutput(minIndex);
            valueFound = true;
        }
        else //search through calibration array for desired to be between two adjacent points
        {
            for(uint16_t j = 0; j < readingsSize-1 && !valueFound; j++)
            {
                uint16_t jp = j + 1;
                if(desired == readings[j])
                {
                    array[desired] = feedForwardCalibrationOutput(j);
                    valueFound = true;
                }
                //readings[j] must be at least one smaller than desired and readings[jp] must be equal or larger so never will divide by zero
                else if((desired > readings[j]) && (desired <= readings[jp])) //by the IVT, there must be some set of subsequent rising points with the desired value between
                {
                    array[desired] = feedForwardCalibrationOutput(j) + ((int32_t)((desired - readings[j])*(feedForwardCalibrationOutput(jp) - feedForwardCalibrationOutput(j))))/(readings[jp] - readings[j]); //linear interpolate
                    valueFound = true;
                }
                else if (desired < readings[j] && desired >= readings[jp])
                {
                    array[desired] = feedForwardCalibrationOutput(jp) + ((int32_t)(readings[j] - desired)*(feedForwardCalibrationOutput(jp) - feedForwardCalibrationOutput(j)))/(readings[j] - readings[jp]); //linear interpolate
                    valueFound = true;
                }
                if (valueFound)
                    break;
            }
        }
        if (!valueFound) //must be too low
        {
            Serial.println("Can't find");
            Serial.println(desired);
            //feedForward[desired] = ffCalibrationOutput(0) + (desired - ffCalibration[0])*(ffCalibrationOutput(FF_CALIBRATION_ARRAY_LENGTH-1) - ffCalibrationOutput(0))/(ffCalibration[FF_CALIBRATION_ARRAY_LENGTH-1] - ffCalibration[0]); //linear interpolate beyond ends
        }

        minSetPoint = 2*minValue + 1;
        maxSetPoint = .98*maxValue - 1;
        array[desired] = bound(array[desired], min(outputLimits.min, outputLimits.max), max(outputLimits.min, outputLimits.max));
    }

    return Extrema 
    {
        minSetPoint,
        maxSetPoint
    };
}

Extrema FeedForwardHelper::getExtrema()
{
    uint16_t maxIndex = 0;
    uint16_t minIndex = 0;
    for(uint16_t i = 0; i < readingsSize; i++)
    {
        if (readings[i] > readings[maxIndex])
        {
            maxIndex = i;
        }
        if (readings[i] < readings[minIndex])
        {
            minIndex = i;
        }
    }

    return Extrema
    {
        minIndex,
        maxIndex
    };
}