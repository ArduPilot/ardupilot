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
 * ArduPilot device driver for Camsense X1 LiDAR,  Amicro Lidar-X1 etc
 *
 * Model: Camsense X1 (commonly found in various vacuum robots and as a standalone module)
 * Type: 360-degree triangulation laser rangefinder
 *
 * References:
 * - Pinout and technical details: https://github.com/Vidicon/camsense-X1
 *
 * Protocol Specification:
 * - Baud rate: 115200 bps
 * - Packet size: 36 bytes
 * - Header: 0x55 0xAA
 * - Payload: 8 samples per packet (14-bit distance + intensity byte)
 * - Angle format: Sent with 0xA000 (40960) offset, scaled by 64
 *
 * Implementation based on the LD06 LiDAR driver structure.
 * Modified for Camsense X1 protocol by Alexey Kozin
 */


#include "AP_Proximity_CamsenseX1.h"

#if AP_PROXIMITY_CAMSENSE_X1_ENABLED
#include <AP_HAL/AP_HAL.h>

#define PROXIMITY_CAMSENSE_TIMEOUT_MS 500


void AP_Proximity_CamsenseX1::update(void)
{
    if (_uart == nullptr) {
        return;
    }

    get_readings();

    // check timeout same as LD06
    if (_last_distance_received_ms == 0 || (AP_HAL::millis() - _last_distance_received_ms > PROXIMITY_CAMSENSE_TIMEOUT_MS)) {
        set_status(AP_Proximity::Status::NoData);
    } else {
        set_status(AP_Proximity::Status::Good);
    }
}


void AP_Proximity_CamsenseX1::get_readings()
{
    uint32_t nbytes = MIN((uint16_t) 4000, _uart->available());

    while (nbytes-- > 0) {
        uint8_t b;
        if (!_uart->read(b)) {
            break;
        }

        switch (_step) {
            case State::WAIT_SYNC1:
                if (b == MSG_HEADER1) {
                    _step = State::WAIT_SYNC2;
                }
                break;

            case State::WAIT_SYNC2:
                if (b == MSG_HEADER2) {
                    _step = State::RECEIVING;
                    _byte_count = 2;
                    _buffer[0] = MSG_HEADER1;
                    _buffer[1] = MSG_HEADER2;
                } else {
                    _step = State::WAIT_SYNC1;
                }
                break;

            case State::RECEIVING:
                _buffer[_byte_count++] = b;
                if (_byte_count >= sizeof(_buffer)) {
                    _last_distance_received_ms = AP_HAL::millis();
                    decode_packet();
                    _step = State::WAIT_SYNC1;
                    _byte_count = 0;
                }
                break;
        }
    }
}


void AP_Proximity_CamsenseX1::decode_packet()
{
    // check header (0x03 0x08)
    if (_buffer[2] != 0x03 || _buffer[3] != 0x08) {
        return;
    }

    const float start_angle = (float(uint16_t(_buffer[6]) | (uint16_t(_buffer[7]) << 8)) - 40960.0f) / 64.0f;
    const float stop_angle_raw = (float(uint16_t(_buffer[32]) | (uint16_t(_buffer[33]) << 8)) - 40960.0f) / 64.0f;
    
    float end_angle = stop_angle_raw;
    if (end_angle < start_angle) {
        end_angle += 360.0f;
    }

    const float angle_step = (end_angle - start_angle) / 7.0f;

    for (uint8_t i = 0; i < 8; i++) {
        const uint8_t base = 8 + (i * 3);
        const uint16_t raw_dist = uint16_t(_buffer[base]) | (uint16_t(_buffer[base + 1]) << 8);
        
        // angle correction
        const float angle_deg = correct_angle_for_orientation(start_angle + angle_step * i);
        const float distance_m = (raw_dist & 0x3FFF) * 0.001f;

        
        if (distance_m < distance_min_m() || distance_m > distance_max_m() || (raw_dist & 0x4000)) {
            continue;
        }

        if (ignore_reading(angle_deg, distance_m)) {
            continue;
        }


        
        
        const AP_Proximity_Boundary_3D::Face face = frontend.boundary.get_face(angle_deg);
        if (face != _last_face) {
            if (_last_distance_valid) {
                frontend.boundary.set_face_attributes(_last_face, _last_angle_deg, _last_distance_m, state.instance);
            } else {
                frontend.boundary.reset_face(face, state.instance);
            }
            _last_face = face;
            _last_distance_valid = false;
        }

        // min distance
        if (!_last_distance_valid || (distance_m < _last_distance_m)) {
            _last_distance_m = distance_m;
            _last_distance_valid = true;
            _last_angle_deg = angle_deg;
        }

        
        database_push(angle_deg, distance_m);
    }
}
#endif
