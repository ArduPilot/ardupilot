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

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_BENEWAKE_TFA1500_ENABLED

#include "AP_RangeFinder_Benewake_TFA1500.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL &hal;

static constexpr uint8_t TFA1500_FRAME_HEADER = 0x5C;
static constexpr uint8_t TFA1500_FRAME_LENGTH = 5;
static constexpr uint32_t TFA1500_DIST_MAX_CM = 130000;

static const uint8_t TFA1500_CMD_START[] = {
    0x55U, 0xAAU, 0xCBU, 0xCCU, 0xCCU, 0xCCU, 0xCCU, 0xFBU};

void AP_RangeFinder_Benewake_TFA1500::find_signature_in_buffer(uint8_t start)
{
    for (uint8_t i = start; i < tf_frame_len; i++) {
        if (tf_frame.bytes[i] == TFA1500_FRAME_HEADER) {
            memmove(&tf_frame.bytes[0], &tf_frame.bytes[i], tf_frame_len - i);
            tf_frame_len -= i;
            return;
        }
    }
    tf_frame_len = 0;
}

bool AP_RangeFinder_Benewake_TFA1500::get_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    const uint32_t now = AP_HAL::millis();
    if (last_init_ms == 0 ||
        ((now - last_init_ms > 1000) && (now - state.last_reading_ms > 1000))) {
        last_init_ms = now;
        uart->write(TFA1500_CMD_START, sizeof(TFA1500_CMD_START));
    }

    float sum_cm = 0.0f;
    uint16_t count = 0;
    uint16_t count_out_of_range = 0;

    // read limit to prevent consuming too much CPU
    for (auto j = 0U; j < 8192; j++) {
        const auto num_read = uart->read(&tf_frame.bytes[tf_frame_len], sizeof(tf_frame) - tf_frame_len);
        tf_frame_len += num_read;
        
        if (tf_frame_len == 0) {
            break;
        }

        if (tf_frame.packet.header != TFA1500_FRAME_HEADER) {
            find_signature_in_buffer(1);
            continue;
        }

        if (tf_frame_len < TFA1500_FRAME_LENGTH) {
            break;
        }

        const uint8_t expected_checksum = (uint8_t)~(tf_frame.packet.dist_low +
                                                tf_frame.packet.dist_mid +
                                                tf_frame.packet.dist_high);
        if (expected_checksum == tf_frame.packet.checksum_of_bytes) {
            const uint32_t dist_cm = (tf_frame.packet.dist_high << 16) |
                                     (tf_frame.packet.dist_mid << 8) |
                                     tf_frame.packet.dist_low;
            tf_frame_len = 0;

            if ((dist_cm >= TFA1500_DIST_MAX_CM) || (dist_cm == 0x3FFFFF)) {
                count_out_of_range++;
            } else {
                sum_cm += dist_cm;
                count++;
            }
        } else {
            find_signature_in_buffer(1);
        }
    }

    if (count > 0) {
        reading_m = (sum_cm * 0.01f) / count;
        return true;
    } 
    if (count_out_of_range > 0) {
        reading_m = MAX(model_dist_max_cm() * 0.01f, max_distance());
        return true;
    }

    return false;
}

#endif
