// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
 *       AP_RangeFinder_LR4.cpp - Arduino Library for Porcupine Electronics LR4 + Fluke 414D
 *       Code by Dominic Chen
 *
 *       datasheet: http://porcupineelectronics.com/uploads/LR4_Data_Sheet.pdf
 *                  http://support.fluke.com/find-sales/Download/Asset/3028711_0000_ENG_E_W.PDF
 *
 *       Sensor should be connected to UART2
 *
 *       Variables:
 *               bool measuring : indicates whether last communication with sensor was successful
 *
 *       Methods:
 *               take_reading(): ask the sonar to take a new distance measurement
 *               read() : read last distance measured (in cm)
 *
 */

// AVR LibC Includes
#include "AP_RangeFinder_LR4.h"
#include <AP_HAL.h>

extern const AP_HAL::HAL& hal;

// Constructor //////////////////////////////////////////////////////////////

AP_RangeFinder_LR4::AP_RangeFinder_LR4(AP_HAL::UARTDriver *uart, FilterInt16 *filter) :
    RangeFinder(NULL, filter),
    _port(uart),
    reading(false)
{
    min_distance = AP_RANGE_FINDER_LR4_MIN_DISTANCE;
    max_distance = AP_RANGE_FINDER_LR4_MAX_DISTANCE;
}

// Public Methods //////////////////////////////////////////////////////////////

// take_reading - ask sensor to start making readings
bool AP_RangeFinder_LR4::start_reading() {
    char tmp;

    // send 'g' to start reading
    _port->print_P(PSTR("g"));

    // loop until valid response received
    while (true) {
        if (_port->available()) {
            tmp = _port->read();

            // receiving 'ok'
            if (tmp == 'o') {
                return true;
            }
            // receiving 'badcmd'
            else if (tmp == 'b') {
                return false;
            }
            // otherwise receiving a leftover measurement
        }
    }

    return false;
}

// stop_reading - ask sensor to stop making readings
bool AP_RangeFinder_LR4::stop_reading() {
    char tmp;

    // send 's' to stop reading
    _port->print_P(PSTR("s"));

    // loop until valid response received
    while (true) {
        if (_port->available()) {
            tmp = _port->read();

            // receiving 'ok'
            if (tmp == 'o') {
                return true;
            }
            // receiving 'badcmd'
            else if (tmp == 'b') {
                return false;
            }
            // otherwise receiving a leftover measurement
        }
    }

    return false;
}

// read - return last value measured by sensor
int AP_RangeFinder_LR4::read() {
    uint8_t pos = 0;
    char buf[5], tmp;
    int val = 0;
    
    // attempt to start reading if not reading
    if (!reading) {
        reading = start_reading();
    }
    
    // currently reading
    if (reading) {
        // flush previous results
        _port->flush();

        // loop until we obtain a valid 5-digit measurement
        while (pos != 5) {
            if (_port->available()) {
                tmp = _port->read();

                // if it is a digit
                if (tmp >= '0' && tmp <= '9') {
                    buf[pos++] = tmp;
                }
            }
        }

        // convert result from string to integer
        val = atoi(buf);

        // ensure distance is within min and max
        val = constrain_float(val, min_distance, max_distance);

        // apply mode (median?) filter?
        // val = _mode_filter->apply(val);
        
        // reading = !stop_reading();

        return val;
    }

    // failed to start reading, return constant
    return -1;
}
