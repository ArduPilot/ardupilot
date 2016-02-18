#pragma once

#include <AP_Common/AP_Common.h>
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
 *       AP_ADC.cpp - Analog Digital Converter Base Class for Ardupilot Mega
 *       Code by James Goppert. DIYDrones.com
 *
 *       Methods:
 *               Init() : Initialization of ADC. (interrupts etc)
 *               Ch(ch_num) : Return the ADC channel value
 *               Ch6(channel_numbers, result) : Return 6 ADC channel values
 */

class AP_ADC
{
public:
    AP_ADC() {
    };                // Constructor
    virtual void            Init() = 0;

    /* read one channel value */
    virtual float           Ch(uint8_t ch_num) = 0;

    /* read 6 channels values as a set, used by IMU for 3 gyros
     *  and 3 accelerometeres.
     *
     *  Pass in an array of 6 channel numbers and results are
     *  returned in result[]
     *
     *  The function returns the amount of time (in microseconds)
     *  since the last call to Ch6().
     */
    virtual uint32_t        Ch6(const uint8_t *channel_numbers, float *result) = 0;

    // check if Ch6() can return new data
    virtual bool            new_data_available(const uint8_t *channel_numbers) = 0;

    virtual uint16_t        num_samples_available(const uint8_t *channel_numbers) = 0;

private:
};

#include "AP_ADC_ADS7844.h"
#include "AP_ADC_ADS1115.h"
