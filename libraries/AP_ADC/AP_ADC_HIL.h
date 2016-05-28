#ifndef AP_ADC_HIL_H
#define AP_ADC_HIL_H
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       AP_ADC_HIL.h
 *       Author: James Goppert
 *
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
    void        Init();

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

private:

    ///
    // The raw adc array
    uint16_t        adcValue[8];

    // the time in microseconds when Ch6 was last requested
    uint32_t        _last_ch6_time;

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
    
    // read function that pretends to capture new data
    void read(void) {
       _count++;
    }

    uint16_t  _count;                    // number of samples captured
};

#endif
