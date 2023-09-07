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

#if AP_RANGEFINDER_AINSTEIN_LR_D1_ENABLED

#include "AP_RangeFinder_Ainstein_LR_D1.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/utility/sparse-endian.h>

static const uint8_t PACKET_HEADER_MSB = 0xEB;
static const uint8_t PACKET_HEADER_LSB = 0x90;

// make sure we know what size the packet object is:
// assert_storage_size<AP_RangeFinder_Ainstein_LR_D1::LRD1Union::LRD1Packet, 32> _assert_storage_lrd1_packet;

bool AP_RangeFinder_Ainstein_LR_D1::move_signature_in_buffer(uint8_t start)
{
    for (uint8_t i=start; i<buffer_used; i++) {
        if (u.buffer[i] == PACKET_HEADER_MSB) {
            memmove(&u.buffer[0], &u.buffer[i], buffer_used-i);
            buffer_used -= i;
            return true;
        }
    }
    // header byte not in buffer
    buffer_used = 0;
    return false;
}

uint8_t AP_RangeFinder_Ainstein_LR_D1::LRD1Union::calculate_checksum() const
{
    uint32_t sum = 0;
    for (uint8_t i=3; i<31; i++) {
        sum += buffer[i];
    }
    return sum & 0xff;
}

// get_reading - read a value from the sensor
bool AP_RangeFinder_Ainstein_LR_D1::get_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    const uint8_t num_read = uart->read(&u.buffer[buffer_used], ARRAY_SIZE(u.buffer)-buffer_used);
    buffer_used += num_read;

    if (buffer_used == 0) {
        return false;
    }

    if (u.packet.header_msb != PACKET_HEADER_MSB &&
        !move_signature_in_buffer(1)) {
        return false;
    }

    if (buffer_used < sizeof(u.packet)) {
        return false;
    }

    // sanity checks; see data sheet on these fixed values.
    if (u.packet.header_lsb != PACKET_HEADER_LSB ||
        u.packet.device_id != 0x00 ||
        u.packet.length != 28 ||
        //u.packet.objects_number != 1 ||
        u.calculate_checksum() != u.packet.checksum) {
        // sanity checks failed - discard and try again next time we're called:
        move_signature_in_buffer(1);
        return false;
    }

    reading_m = be16toh(u.packet.object1_alt) * 0.01;

    // consume this packet:
    move_signature_in_buffer(sizeof(u.packet));

    /*  From datasheet:
      Altitude data from radar may have unexpected
      or incorrect readings if aircraft pitch and/or
      roll are beyond the radar’s detection angle
      (Azimuth 43°，Elevation 30°). The ideal case is
      keeping radar perpendicular to ground.
    */
    const auto &ahrs = AP::ahrs();
    if (fabs(ahrs.roll) > radians(43)) {
        return false;
    }
    if (fabs(ahrs.pitch) > radians(30)) {
        return false;
    }

    snr = u.packet.object1_snr;

    if (snr == 0) {
        // out of range according to datasheet
        // max valid altitude from device is 655 metres
        reading_m = MAX(656, max_distance_cm() * 0.01 + 1);
        return true;
    }

    return true;
}

bool AP_RangeFinder_Ainstein_LR_D1::get_signal_quality_pct(uint8_t &quality_pct) const
{
    if (snr == 255) {
        return false;
    }
    return snr;
}

#endif  // AP_RANGEFINDER_AINSTEIN_LR_D1_ENABLED
