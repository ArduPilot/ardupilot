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

#include <AP_HAL/AP_HAL.h>
#include "AP_RangeFinder_uLanding.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>

extern const AP_HAL::HAL& hal;

/*
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_uLanding::AP_RangeFinder_uLanding(RangeFinder &_ranger, uint8_t instance,
                                                             RangeFinder::RangeFinder_State &_state,
                                                             AP_SerialManager &serial_manager) :
    AP_RangeFinder_Backend(_ranger, instance, _state)
{
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Aerotenna_uLanding, 0);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Aerotenna_uLanding, 0));
    }
}

/*
   detect if a uLanding rangefinder is connected. We'll detect by
   trying to take a reading on Serial. If we get a result the sensor is
   there.
*/
bool AP_RangeFinder_uLanding::detect(RangeFinder &_ranger, uint8_t instance, AP_SerialManager &serial_manager)
{
    return serial_manager.find_serial(AP_SerialManager::SerialProtocol_Aerotenna_uLanding, 0) != nullptr;
}

// read - return last value measured by sensor
bool AP_RangeFinder_uLanding::get_reading(uint16_t &reading_cm)
{
    if (uart == nullptr) {
        return false;
    }

    // read any available lines from the uLanding
    float sum = 0;
    uint16_t count = 0;
    uint8_t  index = 0;

    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
        uint8_t c = uart->read();
        // ok, we have located start byte
        if (c == 72 && index == 0) {
            linebuf_len = 0;
            index       = 1;
        }
        // now it is ready to decode index information
        if (index == 1) {
            linebuf[linebuf_len] = c;
            linebuf_len ++;
            if (linebuf_len == 3) {
                index = 0;
                sum += (linebuf[2]&0x7F) *128 + (linebuf[1]&0x7F);
                linebuf_len = 0;
                count ++;
            }
        }
    }

    if (count == 0) {
        return false;
    }

    reading_cm = 2.5f * sum / count;
    return true;
}

/*
   update the state of the sensor
*/
void AP_RangeFinder_uLanding::update(void)
{
    if (get_reading(state.distance_cm)) {
        // update range_valid state based on distance measured
        last_reading_ms = AP_HAL::millis();
        update_status();
    } else if (AP_HAL::millis() - last_reading_ms > 200) {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}
