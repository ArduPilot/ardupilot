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

//
//  GPS_406.cpp - 406 GPS library for Arduino
//  Code by Michael Smith, Jason Short, Jordi Muñoz and Jose Julio. DIYDrones.com
//  This code works with boards based on ATMega168/328 ATMega1280 (Serial port 1)
//
#include <AP_HAL.h>

#include "AP_GPS_406.h"

extern const AP_HAL::HAL& hal;

static const char init_str[] = "$PSRF100,0,57600,8,1,0*37";

// Public Methods ////////////////////////////////////////////////////////////////////
void AP_GPS_406::init(AP_HAL::UARTDriver *s, enum GPS_Engine_Setting nav_setting)
{
    _change_to_sirf_protocol();         // Changes to SIRF protocol and sets baud rate
    _configure_gps();                           // Function to configure GPS, to output only the desired msg's

    AP_GPS_SIRF::init(s, nav_setting);                     // let the superclass do anything it might need here
}

// Private Methods //////////////////////////////////////////////////////////////

void
AP_GPS_406::_configure_gps(void)
{
    const uint8_t gps_header[]          = {0xA0, 0xA2, 0x00, 0x08, 0xA6, 0x00};
    const uint8_t gps_payload[]         = {0x02, 0x04, 0x07, 0x09, 0x1B};
    const uint8_t gps_checksum[]        = {0xA8, 0xAA, 0xAD, 0xAF, 0xC1};
    const uint8_t gps_ender[]           = {0xB0, 0xB3};

    for(int16_t z = 0; z < 2; z++) {
        for(int16_t x = 0; x < 5; x++) {
            _port->write(gps_header, sizeof(gps_header));       // Prints the msg header, is the same header for all msg..
            _port->write(gps_payload[x]);                                       // Prints the payload, is not the same for every msg
            for(int16_t y = 0; y < 6; y++)                                              // Prints 6 zeros
                _port->write((uint8_t)0);
            _port->write(gps_checksum[x]);                                      // Print the Checksum
            _port->write(gps_ender[0]);                                         // Print the Ender of the string, is same on all msg's.
            _port->write(gps_ender[1]);                                         // ender
        }
    }
}

// The EM406 defalts to NMEA at 4800bps.  We want to switch it to SiRF binary
// mode at a higher rate.
//
// The change is sticky, but only for as long as the internal supercap holds
// settings (usually less than a week).
//
void
AP_GPS_406::_change_to_sirf_protocol(void)
{
    // this is a bit grody...
    AP_HAL::UARTDriver *fs = (AP_HAL::UARTDriver*)_port;

    fs->begin(4800);
    hal.scheduler->delay(300);
    _port->print(init_str);
    hal.scheduler->delay(300);

    fs->begin(9600);
    hal.scheduler->delay(300);
    _port->print(init_str);
    hal.scheduler->delay(300);

    fs->begin(GPS_406_BITRATE);
    hal.scheduler->delay(300);
    _port->print(init_str);
    hal.scheduler->delay(300);
}

