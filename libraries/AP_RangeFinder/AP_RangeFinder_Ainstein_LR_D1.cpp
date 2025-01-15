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

// get_reading - read a value from the sensor
bool AP_RangeFinder_Ainstein_LR_D1::get_reading(float &reading_m)
{
    if (uart == nullptr || uart->available() == 0) {
        return false;
    }

    bool has_data = false;

    uint32_t available = MAX(uart->available(), static_cast<unsigned int>(PACKET_SIZE*4));
    while (available >= PACKET_SIZE) {
        // ---------------
        // Sync up with the header
        const uint8_t header[] = {
            0xEB,   // Header MSB
            0x90,   // Header LSB
            0x00   // Device ID
        };
        for (uint8_t i = 0; i<ARRAY_SIZE(header); i++) {
            available--;
            if (uart->read() != header[i]) {
                continue;
            }
        }

        const uint8_t rest_of_packet_size = (PACKET_SIZE - ARRAY_SIZE(header));
        if (available < rest_of_packet_size) {  
            return false;
        }

        // ---------------
        // header is aligned!
        // ---------------

        uint8_t buffer[rest_of_packet_size];
        available -= uart->read(buffer, ARRAY_SIZE(buffer));

        const uint8_t checksum = buffer[ARRAY_SIZE(buffer)-1]; // last byte is a checksum
        if (crc_sum_of_bytes(buffer, ARRAY_SIZE(buffer)-1) != checksum) {
            // bad Checksum
            continue;
        }

        const uint8_t malfunction_alert = buffer[1];
        reading_m = UINT16_VALUE(buffer[3], buffer[4]) * 0.01;
        const uint8_t snr = buffer[5];

#if AP_RANGEFINDER_AINSTEIN_LR_D1_SHOW_MALFUNCTIONS
        const uint32_t now_ms = AP_HAL::millis();
        if (malfunction_alert_prev != malfunction_alert && now_ms - malfunction_alert_last_send_ms >= 1000) {
            report_malfunction(malfunction_alert, malfunction_alert_prev);
            malfunction_alert_prev = malfunction_alert;
            malfunction_alert_last_send_ms = now_ms;
        }
#endif

        /* From datasheet:
            Altitude measurements associated with a SNR value 
            of 13dB or lower are considered erroneous. 

            SNR values of 0 are considered out of maximum range (655 metres)

            The altitude measurements should not in any circumstances be used as true
            measurements independently of the corresponding SNR values. 
        */
        signal_quality_pct = (snr <= 13 || malfunction_alert != 0) ? RangeFinder::SIGNAL_QUALITY_MIN : RangeFinder::SIGNAL_QUALITY_MAX;

        if (snr <= 13) {            
            has_data = false;           
            if (snr == 0) {
                state.status = RangeFinder::Status::OutOfRangeHigh;
                reading_m = MAX(656, max_distance_cm() * 0.01 + 1);
            } else {
                state.status = RangeFinder::Status::NoData;
            }
        } else {
            has_data = true;
            state.status = RangeFinder::Status::Good;
        }
    }

    return has_data;
}

#if AP_RANGEFINDER_AINSTEIN_LR_D1_SHOW_MALFUNCTIONS
void AP_RangeFinder_Ainstein_LR_D1::report_malfunction(const uint8_t _malfunction_alert_, const uint8_t _malfunction_alert_prev_)
{
    static const struct {
        MalfunctionAlert bit;
        const char *name;
    } alerts[] {
        { MalfunctionAlert::Temperature, "Temperature" },
        { MalfunctionAlert::Voltage, "Voltage" },
        { MalfunctionAlert::IFSignalSaturation, "IF signal saturation" },
        { MalfunctionAlert::AltitudeReading, "Attitude reading overflow" },
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
#endif // AP_RANGEFINDER_AINSTEIN_LR_D1_SHOW_MALFUNCTIONS

#endif // AP_RANGEFINDER_AINSTEIN_LR_D1_ENABLED
