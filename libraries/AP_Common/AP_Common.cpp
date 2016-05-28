/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
 *   AP_Common.cpp - common utility functions
 */


#include <AP_HAL.h>
#include <AP_Common.h>

extern const AP_HAL::HAL& hal;

/*
 *  map from a 16 bit EEPROM baud rate to a real baud rate
 * The practical rates for APM1/APM2 are up to 500kbaud, although
 * beyond 115200 isn't recommended. For PX4 we can do 1.5MBit,
 * although 921600 is more reliable
 */
uint32_t map_baudrate(int16_t rate)
{
    if (rate <= 0) {
        rate = 57;
    }
    switch (rate) {
    case 1:    return 1200;
    case 2:    return 2400;
    case 4:    return 4800;
    case 9:    return 9600;
    case 19:   return 19200;
    case 38:   return 38400;
    case 57:   return 57600;
    case 111:  return 111100;
    case 115:  return 115200;
    case 230:  return 230400;
    case 460:  return 460800;
    case 500:  return 500000;
    case 921:  return 921600;
    case 1500:  return 1500000;
    }
    // allow any other kbaud rate
    return rate*1000;
}

