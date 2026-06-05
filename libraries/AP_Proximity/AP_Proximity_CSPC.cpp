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
 * ArduPilot device driver for Guoke Optics COIN-D6 / CSPC M1CT_TOF 360 degree LiDAR
 *
 * Protocol reference: COIN-D6 LiDAR Data Format Standard Specification V1.0
 *
 * Packet structure (all little-endian):
 *   [0xAA 0x55][M&T][LSN][FSAL FSAH][LSAL LSAH][CSL CSH][S1_L S1_2nd S1_H][S2 ...]
 *
 *   M&T   : Mode (bit 7) + Type (bit 0). Type 0 = new revolution, 1 = mid-revolution.
 *   LSN   : Number of samples in this packet (variable, typically 8-32).
 *   FSA   : First sample angle, units of 1/64 degree after stripping check bit (LSB).
 *   LSA   : Last sample angle, same units.
 *   CS    : XOR checksum of all 16-bit words in the packet (currently not validated;
 *           checksum algorithm is not fully documented in the public spec, so we
 *           rely on range-checking distances instead).
 *   Si    : 3 bytes per sample:
 *             Distance(mm)   = Si_H * 64 + (Si_2nd >> 2)
 *             Intensity(8b)  = (Si_2nd & 0x03) * 64 + (Si_L >> 2)
 *
 * Angle interpolation between FSA and LSA:
 *   angle_fsa_deg = (FSA >> 1) / 64.0
 *   angle_lsa_deg = (LSA >> 1) / 64.0
 *   angle_i_deg   = angle_fsa + (angle_lsa - angle_fsa) / (LSN - 1) * i   (i = 0..LSN-1)
 *
 * Author: Andrew Dewar
 * Based on AP_Proximity_LD06 by Adithya Patil (Georgia Tech)
 */

#include "AP_Proximity_config.h"

#if AP_PROXIMITY_CSPC_ENABLED
#include "AP_Proximity_CSPC.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

// Confidence threshold below which a sample is ignored (0-255 scale)
#define CSPC_CONFIDENCE_THRESHOLD       5

// Update sensor state - called at high rate from main loop
void AP_Proximity_CSPC::update(void)
{
    if (_uart == nullptr) {
        return;
    }

    // Send motor-start command once (idempotent if LiDAR already spinning)
    if (!_motor_started) {
        send_motor_start();
        _motor_started = true;
    }

    // Pull bytes from UART and run the state machine
    get_readings();

    // Health: have we received a valid packet recently?
    const uint32_t now_ms = AP_HAL::millis();
    if (_last_distance_received_ms == 0 ||
        (now_ms - _last_distance_received_ms) > CSPC_TIMEOUT_MS) {
        set_status(AP_Proximity::Status::NoData);
    } else {
        set_status(AP_Proximity::Status::Good);
    }
}

// Send motor-start command bytes once
void AP_Proximity_CSPC::send_motor_start()
{
    if (_uart == nullptr) {
        return;
    }
    static const uint8_t start_cmd[CSPC_CMD_MOTOR_START_LEN] = CSPC_CMD_MOTOR_START_BYTES;
    _uart->write(start_cmd, sizeof(start_cmd));
}

// Drain UART bytes and feed the parser state machine
void AP_Proximity_CSPC::get_readings()
{
    if (_uart == nullptr) {
        return;
    }

    // cap bytes per call to avoid hogging CPU
    uint32_t nbytes = MIN((uint16_t)2048, _uart->available());

    while (nbytes-- > 0) {
        uint8_t c;
        if (!_uart->read(c)) {
            break;
        }

        switch (_parse_state) {
        case ParseState::WAIT_HEAD_1:
            if (c == CSPC_HEADER_BYTE_1) {
                _buffer[0] = c;
                _byte_count = 1;
                _parse_state = ParseState::WAIT_HEAD_2;
            }
            break;

        case ParseState::WAIT_HEAD_2:
            if (c == CSPC_HEADER_BYTE_2) {
                _buffer[1] = c;
                _byte_count = 2;
                _parse_state = ParseState::READ_HEADER;
            } else if (c == CSPC_HEADER_BYTE_1) {
                // sequential 0xAA: stay armed for HEAD_2
                _buffer[0] = c;
                _byte_count = 1;
            } else {
                _parse_state = ParseState::WAIT_HEAD_1;
                _byte_count = 0;
            }
            break;

        case ParseState::READ_HEADER:
            _buffer[_byte_count++] = c;
            if (_byte_count >= CSPC_HEADER_LENGTH) {
                // LSN is byte 3 (0xAA 0x55 M&T LSN ...)
                _expected_samples = _buffer[3];
                if (_expected_samples == 0 || _expected_samples > CSPC_MAX_SAMPLES) {
                    // bogus sample count - resync
                    _parse_state = ParseState::WAIT_HEAD_1;
                    _byte_count = 0;
                    break;
                }
                _expected_packet_bytes = CSPC_HEADER_LENGTH +
                                         (uint16_t)_expected_samples * CSPC_SAMPLE_BYTES;
                if (_expected_packet_bytes > CSPC_MAX_PACKET_LENGTH) {
                    _parse_state = ParseState::WAIT_HEAD_1;
                    _byte_count = 0;
                    break;
                }
                _parse_state = ParseState::READ_SAMPLES;
            }
            break;

        case ParseState::READ_SAMPLES:
            _buffer[_byte_count++] = c;
            if (_byte_count >= _expected_packet_bytes) {
                parse_packet();
                _parse_state = ParseState::WAIT_HEAD_1;
                _byte_count = 0;
            }
            break;
        }
    }
}

// Decode a fully-received CSPC packet and push readings into boundary
void AP_Proximity_CSPC::parse_packet()
{
    // Buffer layout (LSN samples):
    //   [0]=0xAA [1]=0x55 [2]=M&T [3]=LSN
    //   [4..5]=FSA (LE) [6..7]=LSA (LE) [8..9]=CS (LE)
    //   [10..]=samples, 3 bytes each
    const uint8_t lsn       = _buffer[3];
    const uint16_t fsa_raw  = (uint16_t)_buffer[4] | ((uint16_t)_buffer[5] << 8);
    const uint16_t lsa_raw  = (uint16_t)_buffer[6] | ((uint16_t)_buffer[7] << 8);

    // Per spec: angle_deg = (raw >> 1) / 64.0  (strip the check bit, then scale)
    const float angle_fsa_deg = (float)(fsa_raw >> 1) * (1.0f / 64.0f);
    float angle_lsa_deg       = (float)(lsa_raw >> 1) * (1.0f / 64.0f);

    // Handle wrap-around (LSA < FSA across the 360 boundary)
    if (angle_lsa_deg < angle_fsa_deg) {
        angle_lsa_deg += 360.0f;
    }

    const float angle_step_deg = (lsn > 1) ?
                                 (angle_lsa_deg - angle_fsa_deg) / (float)(lsn - 1) :
                                 0.0f;

    // Mark that we've received a valid-looking packet
    _last_distance_received_ms = AP_HAL::millis();

    // Walk samples
    for (uint8_t i = 0; i < lsn; i++) {
        const uint16_t sample_offset = CSPC_HEADER_LENGTH + (uint16_t)i * CSPC_SAMPLE_BYTES;
        const uint8_t s_l   = _buffer[sample_offset + 0];
        const uint8_t s_2nd = _buffer[sample_offset + 1];
        const uint8_t s_h   = _buffer[sample_offset + 2];

        // Distance: 14-bit value in mm
        const uint16_t distance_mm = ((uint16_t)s_h * 64u) + ((uint16_t)s_2nd >> 2);
        // Intensity: 8-bit
        const uint8_t intensity = (uint8_t)(((s_2nd & 0x03u) * 64u) + ((uint16_t)s_l >> 2));

        // Compute this sample's angle (degrees, 0-360)
        float angle_deg = angle_fsa_deg + angle_step_deg * (float)i;
        if (angle_deg >= 360.0f) {
            angle_deg -= 360.0f;
        }
        if (angle_deg < 0.0f) {
            angle_deg += 360.0f;
        }

        const float distance_m = (float)distance_mm * 0.001f;

        // Reject out-of-range or low-confidence samples
        if (distance_m < distance_min_m() ||
            distance_m > distance_max_m() ||
            intensity < CSPC_CONFIDENCE_THRESHOLD) {
            continue;
        }

        // Correct for mount orientation (yaw correction)
        const float corrected_angle = correct_angle_for_orientation(angle_deg);

        // User-configured ignore zones
        if (ignore_reading(corrected_angle, distance_m)) {
            continue;
        }

        // Push into temp boundary; will be committed at end-of-revolution
        const AP_Proximity_Boundary_3D::Face face =
            frontend.boundary.get_face(corrected_angle);
        _temp_boundary.add_distance(face, corrected_angle, distance_m);

        // Also push to obstacle-avoidance database
        database_push(corrected_angle, distance_m);
    }

    // End-of-revolution = M&T type bit (bit 0) == 0 (new revolution coming next)
    // We commit accumulated faces to the boundary and reset.
    const uint8_t mt = _buffer[2];
    const bool start_of_new_rev = ((mt & 0x01u) == 0u);
    if (start_of_new_rev) {
        _temp_boundary.update_3D_boundary(state.instance, frontend.boundary);
        _temp_boundary.reset();
    }
}

#endif // AP_PROXIMITY_CSPC_ENABLED
