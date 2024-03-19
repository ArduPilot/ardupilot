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


#include "AP_RangeFinder_Ainstein_LRD1_Pro.h"

#if AP_RANGEFINDER_AINSTEIN_LRD1_PRO_SERIAL_ENABLED

#include <GCS_MAVLink/GCS.h>

// sums the bytes in the supplied buffer, returns that sum mod 0xFFFF
uint16_t crc_sum_of_bytes_16(const uint8_t *data, uint16_t count)
{
    uint16_t ret = 0;
    for (uint32_t i=0; i<count; i++) {
        ret += data[i];
    }
    return ret;
}

// sums the bytes in the supplied buffer, returns that sum mod 256
// (i.e. shoved into a uint8_t)
uint8_t crc_sum_of_bytes(const uint8_t *data, uint16_t count)
{
    return crc_sum_of_bytes_16(data, count) & 0xFF;
}


// get_reading - read a value from the sensor
bool AP_RangeFinder_Ainstein_LRD1_Pro::get_reading(float &reading_m)
{
    if (uart == nullptr || uart->available() == 0) {
        return false;
    }

    bool has_data = false;

    uint32_t available = MAX(uart->available(), static_cast<unsigned int>(PACKET_SIZE*4));
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Packet available :: %lu ", available);
    while (available >= PACKET_SIZE) {
        // ---------------
        // Sync up with the header
        const uint8_t header[] = {
            0xEB,   // Header MSB
            0x90,   // Header LSB
            0x00   // Device ID
        };
        for (uint8_t i = 0; i<ARRAY_SIZE(header); i++) {
            // available--;
            while (uart->read() != header[i]) {
                available--;
                continue;
            }
        }

        const uint8_t rest_of_packet_size = (PACKET_SIZE - ARRAY_SIZE(header));
        if (available < rest_of_packet_size) {  
            return false;
        }

        // 
        // header is aligned!
        // ---------------
        
        /*
        This is reading from the uart and storing in the buffer
        */ 
        uint8_t buffer[rest_of_packet_size];
        available -= uart->read(buffer, ARRAY_SIZE(buffer));

        /*
        Validating the : 
        Checksum: (data4+data5+…+data30+data31) bitwise-AND
        with 0xFF
        */
        const uint8_t checksum = buffer[ARRAY_SIZE(buffer)-1]; // last byte is a checksum
        if (crc_sum_of_bytes(buffer, ARRAY_SIZE(buffer)-1) != checksum) {
            // bad Checksum
            continue;
        }
        
        /*
        This is alert for any malfunction set in data5 and data6
        since buffer is starts from data 4 (header allign) so buffer[1] buffer[2]
        0x0000: Normal
        Others: Malfunction Alert
        */
        const uint8_t malfunction_alert = UINT16_VALUE(buffer[1], buffer[2]);

        /*
        Reading data packets:
        24 Gz:
        Altitude : data7 data8 or buffer[3] buffer[4]
        SNR:       data9 or buffer[5]
        Velocity:  data10 data 11 or buffer[6] buffer [7]
        60 Gz :
        Altitude : data12 data13 or buffer[8] buffer[9]
        SNR:       data14 or buffer[10]
        Velocity:  data15 data 16 or buffer[11] buffer [12]
        Integrated:
        Altitude : data17 data18 or buffer[13] buffer[14]
        SNR:       data19 or buffer[15]
        Velocity:  data20 data 21 or buffer[16] buffer [17]
        */
        reading_m = UINT16_VALUE(buffer[17], buffer[18]) * 0.01;
        const uint8_t snr = buffer[15];

        has_data = true;

        if (malfunction_alert_prev != malfunction_alert) {
            malfunction_alert_prev = malfunction_alert;
            report_malfunction(malfunction_alert);
        }

        /* From datasheet:
            Altitude measurements associated with a SNR value 
            of 13dB or lower are considered erroneous. 

            SNR values of 0 are considered out of maximum range (655 metres)

            The altitude measurements should not in any circumstances be used as true
            measurements independently of the corresponding SNR values. 
        */
        signal_quality_pct = (snr <= 13 || malfunction_alert != 0) ? SIGNAL_QUALITY_MIN : SIGNAL_QUALITY_MAX;

        if (snr <= 13) {            
            has_data = false;           
            if (snr == 0) {
                state.status = RangeFinder::Status::OutOfRangeHigh;
                reading_m = MAX(656, max_distance_cm() * 0.01 + 1);
            } else {
                state.status = RangeFinder::Status::NoData;
            }
        } else {
            state.status = RangeFinder::Status::Good;
        }
    }

    return has_data;
}

/*
    Reporting any warnings from the radar
*/

void AP_RangeFinder_Ainstein_LRD1_Pro::report_malfunction(const uint16_t _malfunction_alert_) {
    if (_malfunction_alert_ & static_cast<uint16_t>(MalfunctionAlert::Temperature)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RangeFinder: FPGA Temperature alert");
    }
    if (_malfunction_alert_ & static_cast<uint16_t>(MalfunctionAlert::FPGAVoltage)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RangeFinder: FPGA Voltage Warning");
    }
    if (_malfunction_alert_ & static_cast<uint16_t>(MalfunctionAlert::Temperature_60G)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RangeFinder: 60G-SOC Temperature Warning");
    }
    if (_malfunction_alert_ & static_cast<uint16_t>(MalfunctionAlert::Voltage_60G)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RangeFinder: 60G-SOC Voltage Warning");
    }
    if (_malfunction_alert_ & static_cast<uint16_t>(MalfunctionAlert::Temperature_24G)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RangeFinder: 24G RF Temperature Warning");
    }    
    if (_malfunction_alert_ & static_cast<uint16_t>(MalfunctionAlert::TransmitPower_24G)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RangeFinder: 24G RF Transmit Power Warning");
    }
    if (_malfunction_alert_ & static_cast<uint16_t>(MalfunctionAlert::OverCurrent)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RangeFinder: Whole Board Overcurrent Warning");
    }
    if (_malfunction_alert_ & static_cast<uint16_t>(MalfunctionAlert::IFSignalSaturation)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RangeFinder: IF Signal Saturation Warning");
    }
    if (_malfunction_alert_ & static_cast<uint16_t>(MalfunctionAlert::Software_24G)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RangeFinder: 24G Software Failure Warning");
    }
    if (_malfunction_alert_ & static_cast<uint16_t>(MalfunctionAlert::Temperature_60G)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RangeFinder: 60G Software Failure Warning");
    }
    if (_malfunction_alert_ & static_cast<uint16_t>(MalfunctionAlert::AltitudeReading)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RangeFinder: Altitude Reading Overflow Warning");
    }
    if (_malfunction_alert_ & static_cast<uint16_t>(MalfunctionAlert::AttitudeAngle)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RangeFinder: Excessive Attitude Angle Warning");
    }    
    if (_malfunction_alert_ & static_cast<uint16_t>(MalfunctionAlert::FrameError_60G)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RangeFinder: 60G Frame Error Warning");
    }
    if (_malfunction_alert_ & static_cast<uint16_t>(MalfunctionAlert::InvalidAltitude)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RangeFinder: Invalid Altitude Warning");
    }
    if (_malfunction_alert_ & static_cast<uint16_t>(MalfunctionAlert::InConAlttitude)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RangeFinder:24G & 60G Altitude Inconsistency Warning");
    }
    if (_malfunction_alert_ & static_cast<uint16_t>(MalfunctionAlert::BoardVoltage)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RangeFinder: Whole Board Voltage Warning");
    }
}

#endif