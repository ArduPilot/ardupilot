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

#define FPORT_CONTROL_FRAME_SIZE 29

struct FPort_Frame;

class AP_RCProtocol_FPort : public AP_RCProtocol_Backend {
public:
    AP_RCProtocol_FPort(AP_RCProtocol &_frontend, bool inverted);
    void process_pulse(const uint32_t &width_s0, const uint32_t &width_s1, const uint8_t &pulse_id) override;
    void process_byte(uint8_t byte, uint32_t baudrate, uint8_t byte_id) override;
    size_t get_frame_size() const override { return FPORT_CONTROL_FRAME_SIZE; }

private:
    struct PACKED FPort_Frame {
        uint8_t header; // 0x7E
        uint8_t len;    // 0x19 for control, 0x08 for downlink
        uint8_t type;
        union {
            struct PACKED {
                uint8_t data[22]; // 16 11-bit channels
                uint8_t flags;
                uint8_t rssi;
                uint8_t crc;
                uint8_t end;
            } control;
            struct PACKED {
                uint8_t prim;
                uint16_t appid;
                uint8_t data[4];
                uint8_t crc;
                uint8_t end;
            } downlink;
        };
    };
    static_assert(sizeof(FPort_Frame) == FPORT_CONTROL_FRAME_SIZE, "FPort_Frame incorrect size");

    void decode_control(const FPort_Frame &frame);
    void decode_downlink(const FPort_Frame &frame);
    bool check_checksum(void);

    void _process_byte(uint32_t timestamp_us, uint8_t byte, uint8_t byte_id, bool shared_buffer);
    uint32_t saved_width;

    struct {
        uint8_t *buf;
        uint8_t ofs;
        uint32_t last_byte_us;
        bool got_DLE;
    } byte_input;

    const bool inverted;

    struct {
        bool available = false;
        AP_Frsky_SPort::sport_packet_t packet;
    } telem_data;

    // receiver sends 0x10 when ready to receive telemetry frames (R-XSR)
    bool rx_driven_frame_rate = false;

    // if the receiver is not controlling frame rate apply a constraint on consecutive frames
    uint8_t consecutive_telemetry_frame_count;

    const FPort_Frame *fport_frame;
};
