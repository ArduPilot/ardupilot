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

#include "AP_Proximity_config.h"

#if AP_PROXIMITY_HLK_LD2451_ENABLED

#include "AP_Proximity_HLK_LD2451.h"
#include <AP_HAL/AP_HAL.h>

// Protocol constants (Serial Communication Protocol v1.03)
static const uint8_t FRAME_HEADER[4] = { 0xF4, 0xF3, 0xF2, 0xF1 };
static const uint8_t FRAME_TAIL[4]   = { 0xF8, 0xF7, 0xF6, 0xF5 };

void AP_Proximity_HLK_LD2451::update()
{
    if (_uart == nullptr) {
        return;
    }

    read_sensor_data();

    const uint32_t now_ms = AP_HAL::millis();
    if (_last_packet_ms == 0 || (now_ms - _last_packet_ms > HLK_LD2451_TIMEOUT_MS)) {
        set_status(AP_Proximity::Status::NoData);
    } else {
        set_status(AP_Proximity::Status::Good);
    }
}

void AP_Proximity_HLK_LD2451::read_sensor_data()
{
    uint32_t nbytes = MIN((uint32_t)1024, (uint32_t)_uart->available());

    while (nbytes-- > 0) {
        uint8_t c;
        if (!_uart->read(c)) {
            break;
        }

        switch (_state) {

        case ParseState::HEADER:
            // Slide a 4-byte window looking for FRAME_HEADER
            if (_buf_len < HLK_LD2451_HEADER_LEN) {
                _buf[_buf_len++] = c;
            } else {
                memmove(&_buf[0], &_buf[1], HLK_LD2451_HEADER_LEN - 1);
                _buf[HLK_LD2451_HEADER_LEN - 1] = c;
                _buf_len = HLK_LD2451_HEADER_LEN;
            }
            if (_buf_len == HLK_LD2451_HEADER_LEN &&
                memcmp(_buf, FRAME_HEADER, HLK_LD2451_HEADER_LEN) == 0) {
                _state = ParseState::LENGTH;
            }
            break;

        case ParseState::LENGTH:
            _buf[_buf_len++] = c;
            if (_buf_len == HLK_LD2451_HEADER_LEN + HLK_LD2451_LEN_FIELD_SIZE) {
                const uint16_t payload_len =
                    (uint16_t)_buf[HLK_LD2451_HEADER_LEN] |
                    ((uint16_t)_buf[HLK_LD2451_HEADER_LEN + 1] << 8);

                // Validate: must be at least overhead bytes, and a whole
                // number of targets beyond that, not exceeding maximum.
                const uint16_t target_bytes = payload_len - HLK_LD2451_PAYLOAD_OVERHEAD;
                if (payload_len < HLK_LD2451_PAYLOAD_OVERHEAD ||
                    target_bytes % HLK_LD2451_BYTES_PER_TARGET != 0 ||
                    target_bytes / HLK_LD2451_BYTES_PER_TARGET > HLK_LD2451_MAX_TARGETS) {
                    _buf_len = 0;
                    _state = ParseState::HEADER;
                    break;
                }
                _payload_bytes_expected = payload_len;
                _state = ParseState::PAYLOAD;
            }
            break;

        case ParseState::PAYLOAD: {
            _buf[_buf_len++] = c;

            const uint16_t full_len = HLK_LD2451_HEADER_LEN +
                                      HLK_LD2451_LEN_FIELD_SIZE +
                                      _payload_bytes_expected +
                                      HLK_LD2451_TAIL_LEN;

            if (_buf_len == full_len) {
                // Verify tail
                const uint8_t *tail_ptr = &_buf[full_len - HLK_LD2451_TAIL_LEN];
                if (memcmp(tail_ptr, FRAME_TAIL, HLK_LD2451_TAIL_LEN) == 0) {
                    parse_frame();
                    _last_packet_ms = AP_HAL::millis();
                }
                _buf_len = 0;
                _state = ParseState::HEADER;
            }
            break;
        }

        } // switch
    }
}

/*
  Payload layout (after header + length field):
    [0]  count   – number of detected targets (0–3)
    [1]  alarm   – 0x01 if any target is approaching
    For each target i (5 bytes at offset 2 + i*5):
      [0] angle_raw  – actual_degrees = raw - 0x80  (-128..+127°)
      [1] dist_m     – unsigned, metres
      [2] direction  – 0x00 = moving away, 0x01 = approaching
      [3] speed_kmh  – unsigned, km/h
      [4] snr        – signal-to-noise ratio 0–100
*/
void AP_Proximity_HLK_LD2451::parse_frame()
{
    const uint8_t *payload = &_buf[HLK_LD2451_HEADER_LEN + HLK_LD2451_LEN_FIELD_SIZE];
    const uint8_t  count   = payload[0];

    if (count == 0) {
        return;
    }

    for (uint8_t i = 0; i < count; i++) {
        const uint8_t *t = payload + HLK_LD2451_PAYLOAD_OVERHEAD +
                           i * HLK_LD2451_BYTES_PER_TARGET;

        // Angle: 0x80 = 0°, values < 0x80 are negative (left), > 0x80 are positive (right)
        const float angle_deg  = (float)((int16_t)t[0] - 0x80);
        const float distance_m = (float)t[1];

        if (distance_m < distance_min_m() || distance_m > distance_max_m()) {
            continue;
        }

        // correct_angle_for_orientation applies YAW_CORR and ORIENT parameters
        const float vehicle_angle = correct_angle_for_orientation(angle_deg);

        if (ignore_reading(vehicle_angle, distance_m)) {
            continue;
        }

        database_push(vehicle_angle, distance_m);
    }
}

#endif  // AP_PROXIMITY_HLK_LD2451_ENABLED
