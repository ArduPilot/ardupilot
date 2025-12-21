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

#ifndef AP_RANGEFINDER_AINSTEIN_LR_D1_SHOW_MALFUNCTIONS
#define AP_RANGEFINDER_AINSTEIN_LR_D1_SHOW_MALFUNCTIONS 1
#endif

#include "AP_RangeFinder_Ainstein_LR_D1.h"
#include <GCS_MAVLink/GCS.h>

#include <AP_HAL/utility/sparse-endian.h>

static const uint8_t PACKET_HEADER_MSB = 0xEB;
static const uint8_t PACKET_HEADER_LSB = 0x90;

// make sure we know what size the packet object is:
// assert_storage_size<AP_RangeFinder_Ainstein_LR_D1::LRD1Union::LRD1Packet, 32> _assert_storage_lrd1_packet;

// ensures that there is a packet starting at offset 0 in the buffer.
// If that's not the case this returns false.  Search starts at offset
// start in the buffer - if a packet header is found at a non-zero
// offset then the data is moved to the start of the buffer.
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
    // the -4 here is 3 bytes of header and 1 byte of checksum
    return crc_sum_of_bytes(&buffer[3], sizeof(u.packet)-4);
}

// get_reading - read all samples, return last.  The device sends at
// 40Hz, so there should only ever be one sample available.  Assuming
// that something's gone wrong with scheduling, we will simply return
// the last.
bool AP_RangeFinder_Ainstein_LR_D1::get_reading(float &reading_m)
{
    bool ret = false;
    for (uint8_t i=0; i<10; i++) {
        float tmp_reading = 0;
        if (!get_one_reading(tmp_reading)) {
            break;
        }
        ret = true;
        reading_m = tmp_reading;
    }
    return ret;
}

// get_one_reading - read a value from the sensor
bool AP_RangeFinder_Ainstein_LR_D1::get_one_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    const uint8_t num_read = uart->read(&u.buffer[buffer_used], ARRAY_SIZE(u.buffer)-buffer_used);
    buffer_used += num_read;

    if (buffer_used == 0) {
        return false;
    }

    if (!move_signature_in_buffer(0)) {
        return false;
    }

    if (buffer_used < sizeof(u.packet)) {
        return false;
    }

    // sanity checks; see data sheet on these fixed values.
    if (u.packet.header_lsb != PACKET_HEADER_LSB ||
        u.packet.device_id != 0x00 ||
        u.packet.length != 28 ||
        u.calculate_checksum() != u.packet.checksum) {
        // sanity checks failed - discard and try again next time we're called:
        move_signature_in_buffer(1);
        return false;
    }

    // distinguish between old and new packet format by inspecting
    // "byte 12" (offset 11); in the old format this will always be
    // 0xff.  This was reserved for "offset count", now is altitude-valid.
    const bool is_v19000 = (u.packet_v19000.altitude_valid != 0xff);

    // same for both packet formats:
    reading_m = be16toh(u.packet.object1_alt) * 0.01;

#if AP_RANGEFINDER_AINSTEIN_LR_D1_SHOW_MALFUNCTIONS
        const uint32_t now_ms = AP_HAL::millis();
        uint16_t malfunction_alert;
        if (is_v19000) {
            // new packet format allows for 16 bits:
            malfunction_alert = be16toh(u.packet_v19000.malfunction_alert);
        } else {
            malfunction_alert = u.packet.malfunction_alert;
        }
        if (malfunction_alert_prev != malfunction_alert && now_ms - malfunction_alert_last_send_ms >= 1000) {
            if (is_v19000) {
                report_malfunction_v19000(malfunction_alert, malfunction_alert_prev);
            } else {
                report_malfunction(malfunction_alert, malfunction_alert_prev);
            }
            malfunction_alert_prev = malfunction_alert;
            malfunction_alert_last_send_ms = now_ms;
        }
#endif

    const uint8_t snr = u.packet.object1_snr;

    signal_quality_pct = linear_interpolate(
        RangeFinder::SIGNAL_QUALITY_MIN, RangeFinder::SIGNAL_QUALITY_MAX,
        snr,
        0, 255
    );

    if (is_v19000 && (!u.packet_v19000.altitude_valid ||
                      (malfunction_alert & uint16_t(MalfunctionAlert_v19000::AltitudeReadingOverflow)))) {
        // the device has specifically told us that the reading is not
        // valid, we can't interepret it further.  It's still a
        // reading, so we still return true from this function, but we
        // can't say anything about our altitude so we mark ourselves
        // as out-of-range-high:
        reading_m = max_distance() + 1;
    } else {
        // try to infer out-of-range from the SNR field:

        /* From datasheet:
            Altitude measurements associated with a SNR value 
            of 13dB or lower are considered erroneous. 

            SNR values of 0 are considered out of maximum range (655 metres)

            The altitude measurements should not in any circumstances be used as true
            measurements independently of the corresponding SNR values. 
        */

        if (snr <= 13) {
            if (snr == 0) {
                // out-of-range high:
                reading_m = max_distance() + 1;
            }
        }
    }

    // consume this packet:
    move_signature_in_buffer(sizeof(u.packet));

    return true;
}

#if AP_RANGEFINDER_AINSTEIN_LR_D1_SHOW_MALFUNCTIONS
void AP_RangeFinder_Ainstein_LR_D1::report_malfunction(const uint16_t _malfunction_alert_, const uint16_t _malfunction_alert_prev_)
{
    static const struct {
        MalfunctionAlert bit;
        const char *name;
    } alerts[] {
        { MalfunctionAlert::Temperature, "Temperature" },
        { MalfunctionAlert::Voltage, "Voltage" },
        { MalfunctionAlert::IFSignalSaturation, "IF signal saturation" },
        { MalfunctionAlert::AltitudeReading, "Altitude reading overflow" },
    };

    for (const auto &alert : alerts) {
        if ((_malfunction_alert_ & uint8_t(alert.bit)) == 0) {
            // alert not current
            continue;
        }
        if ((_malfunction_alert_prev_ & uint8_t(alert.bit)) != 0) {
            // alert is not new
            continue;
        }
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RangeFinder: %s alert", alert.name);
    }
}
void AP_RangeFinder_Ainstein_LR_D1::report_malfunction_v19000(const uint16_t malfunction_alert, const uint16_t malfunction_alert_prev)
{
    static const struct {
        MalfunctionAlert_v19000 bit;
        const char *name;
    } alerts[] {
        { MalfunctionAlert_v19000::MCUTemperature, "MCU Temperature" },
        { MalfunctionAlert_v19000::MCUVoltage, "MCU Voltage" },
        { MalfunctionAlert_v19000::IFTemperature, "IF Temperature" },
        { MalfunctionAlert_v19000::IFSignalSaturation, "IF Signal Saturation" },
        { MalfunctionAlert_v19000::Software, "Software" },
        { MalfunctionAlert_v19000::AltitudeReadingOverflow, "Altitude Reading Overflow" },
        { MalfunctionAlert_v19000::Voltage, "Voltage" },
    };

    for (const auto &alert : alerts) {
        if ((malfunction_alert & uint16_t(alert.bit)) == 0) {
            // alert not current
            continue;
        }
        if ((malfunction_alert_prev & uint16_t(alert.bit)) != 0) {
            // alert is not new
            continue;
        }
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RangeFinder: %s alert", alert.name);
    }
}
#endif // AP_RANGEFINDER_AINSTEIN_LR_D1_SHOW_MALFUNCTIONS

#endif // AP_RANGEFINDER_AINSTEIN_LR_D1_ENABLED
