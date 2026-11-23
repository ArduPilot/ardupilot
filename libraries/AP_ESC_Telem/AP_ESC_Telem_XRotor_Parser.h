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
  Parser for the serial telemetry frames sent by HobbyWing XRotor ESCs
  and by T-Motor Alpha ESCs, which use the same format: 24 bytes at
  19200 baud, one ESC per serial line.
 */

#pragma once

#include "AP_ESC_Telem_config.h"

#if AP_ESC_TELEM_XROTOR_PARSER_ENABLED

#include <stdint.h>

class AP_ESC_Telem_XRotor_Parser {
public:
    // the ESCs send at this fixed rate
    static constexpr uint32_t BAUDRATE = 19200;
    // bytes in one frame
    static constexpr uint8_t FRAME_LEN = 24;

    // fields of a frame, in host byte order and the ESC's own units
    struct Frame {
        uint32_t counter;           // 0x01 0x02 then a 16-bit frame count on a T-Motor Alpha
        uint16_t throttle_req;      // throttle the ESC was asked for
        uint16_t throttle;          // throttle the ESC applied
        uint16_t rpm;               // electrical RPM / 10
        uint16_t voltage;           // bus voltage in units of 0.1 V
        int16_t current;            // bus current; sources disagree on its scale
        int16_t phase_current;      // phase current; sources disagree on its scale
        uint8_t mos_temperature;    // raw MOSFET NTC reading, see temperature_decode()
        uint8_t cap_temperature;    // raw capacitor NTC reading, see temperature_decode()
        uint16_t status;            // non-zero on a fault
    };

    // consume one received byte; true when it completes a frame with a good checksum
    bool parse_byte(uint8_t b);

    // the last frame received with a good checksum
    const Frame &get_frame() const { return frame; }

    // convert a raw NTC reading to degrees C
    static uint8_t temperature_decode(uint8_t temp_raw);

private:
    uint8_t buf[FRAME_LEN];         // the frame being received
    uint8_t len;                    // bytes of it received so far
    Frame frame;                    // the last good frame

    // restart from the next header in a frame that failed its checksum
    void resync();
};

#endif  // AP_ESC_TELEM_XROTOR_PARSER_ENABLED
