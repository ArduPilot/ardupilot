#include "AP_Math.h"


double mapValue(double inputValue, double inputMin, double inputMax, double outputMin, double outputMax) {
    // Calculate the percentage of the input value in the input range
    double inputRange = inputMax - inputMin;
    double outputRange = outputMax - outputMin;
    double normalizedValue = (inputValue - inputMin) / inputRange;

    // Map the normalized value to the output range
    double outputValue = outputMin + (normalizedValue * outputRange);

    return outputValue;
}
