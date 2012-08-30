#ifndef AP_ADC_HIL_H
#define AP_ADC_HIL_H

/*
 *       AP_ADC_HIL.h
 *       Author: James Goppert
 *
 *       License:
 *       This library is free software; you can redistribute it and/or
 *       modify it under the terms of the GNU Lesser General Public
 *       License as published by the Free Software Foundation; either
 *       version 2.1 of the License, or (at your option) any later version.
 */

#include <inttypes.h>
#include "AP_ADC.h"

///
// A hardware in the loop model of the ADS7844 analog to digital converter
// @author James Goppert DIYDrones.com
class AP_ADC_HIL : public AP_ADC
{
public:

    ///
    // Constructor
    AP_ADC_HIL();      // Constructor

    ///
    // Initializes sensor, part of public AP_ADC interface
    void        Init(AP_PeriodicProcess*);

    ///
    // Read the sensor, part of public AP_ADC interface
    float        Ch(unsigned char ch_num);

    ///
    // Read 6 sensors at once
    uint32_t        Ch6(const uint8_t *channel_numbers, float *result);

    // see if Ch6 would block
    bool            new_data_available(const uint8_t *channel_numbers);

    // Get minimum number of samples read from the sensors
    uint16_t        num_samples_available(const uint8_t *channel_numbers);

    ///
    // Set the adc raw values given the current rotations rates,
    // temps, accels, and pressures
    void        setHIL(int16_t p, int16_t q, int16_t r, int16_t gyroTemp,
                       int16_t aX, int16_t aY, int16_t aZ, int16_t diffPress);

private:

    ///
    // The raw adc array
    uint16_t        adcValue[8];

    // the time in milliseconds when we last got a HIL update
    uint32_t        last_hil_time;

    ///
    // sensor constants
    // constants declared in cpp file
    // @see AP_ADC_HIL.cpp
    static const uint8_t        sensors[6];
    static const float          gyroBias[3];
    static const float          gyroScale[3];
    static const float          accelBias[3];
    static const float          accelScale[3];
    static const int8_t         sensorSign[6];

    ///
    // gyro set function
    // @param val the value of the gyro in milli rad/s
    // @param index the axis for the gyro(0-x,1-y,2-z)
    inline void        setGyro(uint8_t index, int16_t val) {
        int16_t        temp = val * gyroScale[index] / 1000 + gyroBias[index];
        adcValue[sensors[index]] = (sensorSign[index] < 0) ? -temp : temp;
    }

    ///
    // accel set function
    // @param val the value of the accel in milli g's
    // @param index the axis for the accelerometer(0-x,1-y,2-z)
    inline void        setAccel(uint8_t index, int16_t val) {
        int16_t        temp = val * accelScale[index] / 1000 + accelBias[index];
        adcValue[sensors[index+3]] = (sensorSign[index+3] < 0) ? -temp : temp;
    }

    ///
    // Sets the differential pressure adc channel
    // TODO: implement
    void        setPressure(int16_t val) {
    }

    ///
    // Sets the gyro temp adc channel
    // TODO: implement
    void        setGyroTemp(int16_t val) {
    }
};

#endif
