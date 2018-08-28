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
#include "AP_RangeFinder_Benewake.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL& hal;

#define BENEWAKE_FRAME_HEADER 0x59
#define BENEWAKE_FRAME_LENGTH 9

// format of serial packets received from benewake lidar
//
// Data Bit             Definition      Description
// ------------------------------------------------
// byte 0               Frame header    0x59
// byte 1               Frame header    0x59
// byte 2               DIST_L          Distance (in cm) low 8 bits
// byte 3               DIST_H          Distance (in cm) high 8 bits
// byte 4               STRENGTH_L      Strength low 8 bits
// byte 5               STRENGTH_H      Strength high 8 bits
// byte 6 (TF02)        SIG             Reliability in 8 levels, 7 & 8 means reliable
// byte 6 (TFmini)      Distance Mode   0x02 for short distance (mm), 0x07 for long distance (cm)
// byte 7 (TF02 only)   TIME            Exposure time in two levels 0x03 and 0x06
// byte 8               Checksum        Checksum byte, sum of bytes 0 to bytes 7

/* 
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_Benewake::AP_RangeFinder_Benewake(RangeFinder::RangeFinder_State &_state,
                                                             AP_SerialManager &serial_manager,
                                                             uint8_t serial_instance,
                                                             benewake_model_type model) :
    AP_RangeFinder_Backend(_state),
    model_type(model)
{
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance));
    }
}

/* 
   detect if a Benewake rangefinder is connected. We'll detect by
   trying to take a reading on Serial. If we get a result the sensor is
   there.
*/
bool AP_RangeFinder_Benewake::detect(AP_SerialManager &serial_manager, uint8_t serial_instance)
{
    return serial_manager.find_serial(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance) != nullptr;
}

// distance returned in reading_cm, signal_ok is set to true if sensor reports a strong signal
bool AP_RangeFinder_Benewake::get_reading(uint16_t &reading_cm, bool &signal_ok)
{
    if (uart == nullptr) {
        return false;
    }

    float sum_cm = 0;
    uint16_t count = 0;
    bool dist_reliable = false;

    // read any available lines from the lidar
    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
        char c = uart->read();
        // if buffer is empty and this byte is 0x59, add to buffer
        if (linebuf_len == 0) {
            if (c == BENEWAKE_FRAME_HEADER) {
                linebuf[linebuf_len++] = c;
            }
        } else if (linebuf_len == 1) {
            // if buffer has 1 element and this byte is 0x59, add it to buffer
            // if not clear the buffer
            if (c == BENEWAKE_FRAME_HEADER) {
                linebuf[linebuf_len++] = c;
            } else {
                linebuf_len = 0;
                dist_reliable = false;
            }
        } else {
            // add character to buffer
            linebuf[linebuf_len++] = c;
            // if buffer now has 9 items try to decode it
            if (linebuf_len == BENEWAKE_FRAME_LENGTH) {
                // calculate checksum
                uint16_t checksum = 0;
                for (uint8_t i=0; i<BENEWAKE_FRAME_LENGTH-1; i++) {
                    checksum += linebuf[i];
                }
                // if checksum matches extract contents
                if ((uint8_t)(checksum & 0xFF) == linebuf[BENEWAKE_FRAME_LENGTH-1]) {
                    // tell caller we are receiving packets
                    signal_ok = true;
                    // calculate distance and add to sum
                    uint16_t dist = ((uint16_t)linebuf[3] << 8) | linebuf[2];
                    if (dist != 0xFFFF) {
                        // TFmini has short distance mode (mm)
                        if (model_type == BENEWAKE_TFmini) {
                            if (linebuf[6] == 0x02) {
                                dist *= 0.1f;
                            }
                            // no signal byte from TFmini
                            dist_reliable = true;
                        } else {
                            // TF02 provides signal reliability (good = 7 or 8)
                            dist_reliable = (linebuf[6] >= 7);
                        }
                        if (dist_reliable) {
                            sum_cm += dist;
                            count++;
                        }
                    }
                }
                // clear buffer
                linebuf_len = 0;
                dist_reliable = false;
            }
        }
    }

    if (count == 0) {
        return false;
    }
    reading_cm = sum_cm / count;
    return true;
}

/* 
   update the state of the sensor
*/
void AP_RangeFinder_Benewake::update(void)
{
    bool signal_ok;
    if (get_reading(state.distance_cm, signal_ok)) {
        // update range_valid state based on distance measured
        state.last_reading_ms = AP_HAL::millis();
        if (signal_ok) {
            update_status();
        } else {
            // if signal is weak set status to out-of-range
            set_status(RangeFinder::RangeFinder_OutOfRangeHigh);
        }
    } else if (AP_HAL::millis() - state.last_reading_ms > 200) {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}
