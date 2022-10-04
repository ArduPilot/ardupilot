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

#include <AP_Frsky_Telem/AP_Frsky_SPort.h>

#include "AP_RCProtocol.h"
#include "SoftSerial.h"

#define FPORT2_CONTROL_FRAME_SIZE 38

struct FPort2_Frame;

class AP_RCProtocol_FPort2 : public AP_RCProtocol_Backend {
public:
    AP_RCProtocol_FPort2(AP_RCProtocol &_frontend, bool inverted);
    void process_pulse(const uint32_t &width_s0, const uint32_t &width_s1, const uint8_t &pulse_id) override;
    void process_byte(uint8_t byte, uint32_t baudrate) override;

private:
    void decode_control(const FPort2_Frame &frame);
    void decode_downlink(const FPort2_Frame &frame);
    bool check_checksum(void);

    void _process_byte(uint32_t timestamp_us, uint8_t byte);
    uint32_t saved_width;

    struct {
        uint8_t buf[FPORT2_CONTROL_FRAME_SIZE];
        uint8_t ofs;
        uint32_t last_byte_us;
        uint8_t control_len;
        bool is_downlink;
    } byte_input;

    uint8_t chan_count;

    const bool inverted;

    struct {
        bool available;
        AP_Frsky_SPort::sport_packet_t packet;
    } telem_data;
};
