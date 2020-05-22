/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */

#pragma once

#include "AP_RCProtocol.h"
#include "SoftSerial.h"

#define FPORT_CONTROL_FRAME_SIZE 29

struct FPort_Frame;

class AP_RCProtocol_FPort : public AP_RCProtocol_Backend {
public:
    AP_RCProtocol_FPort(AP_RCProtocol &_frontend, bool inverted);
    void process_pulse(uint32_t width_s0, uint32_t width_s1) override;
    void process_byte(uint8_t byte, uint32_t baudrate) override;

private:
    void decode_control(const FPort_Frame &frame);
    void decode_downlink(const FPort_Frame &frame);
    bool check_checksum(void);

    void _process_byte(uint32_t timestamp_us, uint8_t byte);
    SoftSerial ss{115200, SoftSerial::SERIAL_CONFIG_8N1};
    uint32_t saved_width;

    struct {
        uint8_t buf[FPORT_CONTROL_FRAME_SIZE];
        uint8_t ofs;
        uint32_t last_byte_us;
        bool got_DLE;
    } byte_input;

    const bool inverted;

    struct {
        bool available = false;
        uint32_t data;
        uint16_t appid;
        uint8_t frame;
    } telem_data;

    // receiver sends 0x10 when ready to receive telemetry frames (R-XSR)
    bool rx_driven_frame_rate = false;

    // if the receiver is not controlling frame rate apply a constraint on consecutive frames
    uint8_t consecutive_telemetry_frame_count;
};
