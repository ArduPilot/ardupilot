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

#include <ctype.h>
#include <AP_Logger/AP_Logger.h>

// sums the bytes in the supplied buffer, returns that sum mod 0xFFFF
uint16_t crc_sum_of_bytes_16(const uint8_t *data, uint16_t count)
{
    uint16_t ret = 0;
    for (uint32_t i = 0; i < count; i++)
    {
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
    if (uart == nullptr)
    {
        return false;
    }
    if (uart->available() == 0)
    {
        return false;
    }

    // Noting the current sensor value
    float current_dist_reading_m = reading_m;

    bool has_data = false;
    int16_t nbytes = uart->available();

    /* Adding the parameters to log the data */
    uint16_t reading_24Gz_cm, reading_60Gz_cm, reading_Int_cm;
    uint8_t snr_24, snr_60, snr_Int;

    while (nbytes-- > PACKET_SIZE)
    {
        // Syncing the Header to the start of the Packet
        const uint8_t header[] = {
            0xEB, // Header MSB
            0x90, // Header LSB
            0x00  // Device ID
        };
        for (uint8_t i = 0; i < ARRAY_SIZE(header); i++)
        {
            while (uart->read() != header[i] && nbytes >= PACKET_SIZE)
            {
                nbytes--;
                continue;
            }
        }
        /*
            header is aligned!
            Get the next 28 bytes
        */

        const uint8_t rest_of_packet_size = (PACKET_SIZE - ARRAY_SIZE(header));
        if (nbytes < rest_of_packet_size)
        {
            return false;
        }

        /*
            This is reading from the uart and storing in the buffer
        */
        uint8_t buffer[rest_of_packet_size];
        nbytes -= uart->read(buffer, ARRAY_SIZE(buffer));

        /*
            Validating the :
            Checksum: (data4+data5+…+data30+data31) bitwise-AND
            with 0xFF
        */
        const uint8_t checksum = buffer[ARRAY_SIZE(buffer) - 1]; // last byte is a checksum
        if (crc_sum_of_bytes(buffer, ARRAY_SIZE(buffer) - 1) != checksum)
        {
            // bad Checksum
            continue;
        }

        /*
            Checking that the required buffer is 28 bytes to read
        */
        if (buffer[0] != 0x1C)
        {
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

        uint8_t snr = 0;
        /*
            Param: RNGFND1_LRD1MOD will give value:
            0: 24GHz Mode (Default) and 1: Integrated Mode
        */
        if (lrd1_freq_mode() == 1){
            reading_m = UINT16_VALUE(buffer[13], buffer[14]) * 0.01;
            snr = buffer[15];
        }
        else{
            reading_m = UINT16_VALUE(buffer[3], buffer[4]) * 0.01;
            snr = buffer[5];
        }

        /* Setting Logging Params */
        reading_24Gz_cm = UINT16_VALUE(buffer[3], buffer[4]);
        reading_60Gz_cm = UINT16_VALUE(buffer[8], buffer[9]);
        reading_Int_cm = UINT16_VALUE(buffer[13], buffer[14]);
        snr_24 = buffer[5];
        snr_60 = buffer[10];
        snr_Int = buffer[15];

        /* Validate the Data */

        has_data = check_radar_reading(reading_m);

        /*
        Check for malfunction
        */
        if (malfunction_alert_prev != malfunction_alert)
        {
            malfunction_alert_prev = malfunction_alert;
            // The report malfunction is for debug use only
            // report_malfunction(malfunction_alert);
        }

        /* From datasheet:
            Altitude measurements associated with a SNR value
            of 13dB or lower are considered erroneous.

            SNR values of 0 are considered out of maximum range (655 metres)

            The altitude measurements should not in any circumstances be used as true
            measurements independently of the corresponding SNR values.
        */
        signal_quality_pct = (snr <= 13 || malfunction_alert != 0) ? SIGNAL_QUALITY_MIN : SIGNAL_QUALITY_MAX;

        if (snr <= 13)
        {
            has_data = false;
            if (snr == 0)
            {
                state.status = RangeFinder::Status::OutOfRangeHigh;
                reading_m = MAX(656, max_distance_cm() * 0.01 + 1);
            }
            /* Adding a check to ignore sudden jumps in height from Radar*/
            else if(state.status == RangeFinder::Status::Good && 
                   (reading_m - current_dist_reading_m > CHANGE_HEIGHT_THRESHOLD ||
                   reading_m - current_dist_reading_m < -CHANGE_HEIGHT_THRESHOLD)){
            state.status = RangeFinder::Status::NoData;
            has_data = false;
        }
            else
            {
                state.status = RangeFinder::Status::NoData;
            }
        }
        else
        {
            reading_m = get_avg_reading(reading_m);
            state.status = RangeFinder::Status::Good;
        }

        /* Logging Point Start */
#if HAL_LOGGING_ENABLED
        if (has_data)
        {
            Log_LRD1_Pro(reading_24Gz_cm, reading_60Gz_cm, reading_Int_cm, snr_24, snr_60, snr_Int);
        }
#endif
    }
    return has_data;
}

/*
    Logging Function
    Write rangefinder packet for logging
*/
void AP_RangeFinder_Ainstein_LRD1_Pro::Log_LRD1_Pro(
    uint16_t s_24, uint16_t s_60, uint16_t s_int,
    uint8_t snr_24, uint8_t snr_60, uint8_t snr_int) const
{
    const struct log_LRD1 pkt = {
        LOG_PACKET_HEADER_INIT(LOG_LRD1_MSG),
        time_us : AP_HAL::micros64(),
        dist_24_cm : s_24,
        dist_60_cm : s_60,
        dist_int_cm : s_int,
        snr_24 : snr_24,
        snr_60 : snr_60,
        snr_int : snr_int,
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

bool AP_RangeFinder_Ainstein_LRD1_Pro::check_radar_reading(float &reading_m)
{
    // Range of the LRD1 Pro 0.3m to 656m
    if (reading_m > 655)
    {
        reading_m = MIN(656, reading_m);
        return false;
    }
    return true;
}

/*
    Reporting any warnings from the radar
*/

void AP_RangeFinder_Ainstein_LRD1_Pro::report_malfunction(const uint16_t _malfunction_alert_)
{
    if (_malfunction_alert_ & static_cast<uint16_t>(MalfunctionAlert::Temperature))
    {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RangeFinder: FPGA Temperature alert");
    }
    if (_malfunction_alert_ & static_cast<uint16_t>(MalfunctionAlert::FPGAVoltage))
    {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RangeFinder: FPGA Voltage Warning");
    }
    if (_malfunction_alert_ & static_cast<uint16_t>(MalfunctionAlert::Temperature_60G))
    {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RangeFinder: 60G-SOC Temperature Warning");
    }
    if (_malfunction_alert_ & static_cast<uint16_t>(MalfunctionAlert::Voltage_60G))
    {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RangeFinder: 60G-SOC Voltage Warning");
    }
    if (_malfunction_alert_ & static_cast<uint16_t>(MalfunctionAlert::Temperature_24G))
    {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RangeFinder: 24G RF Temperature Warning");
    }
    if (_malfunction_alert_ & static_cast<uint16_t>(MalfunctionAlert::TransmitPower_24G))
    {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RangeFinder: 24G RF Transmit Power Warning");
    }
    if (_malfunction_alert_ & static_cast<uint16_t>(MalfunctionAlert::OverCurrent))
    {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RangeFinder: Whole Board Overcurrent Warning");
    }
    if (_malfunction_alert_ & static_cast<uint16_t>(MalfunctionAlert::IFSignalSaturation))
    {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RangeFinder: IF Signal Saturation Warning");
    }
    if (_malfunction_alert_ & static_cast<uint16_t>(MalfunctionAlert::Software_24G))
    {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RangeFinder: 24G Software Failure Warning");
    }
    if (_malfunction_alert_ & static_cast<uint16_t>(MalfunctionAlert::Temperature_60G))
    {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RangeFinder: 60G Software Failure Warning");
    }
    if (_malfunction_alert_ & static_cast<uint16_t>(MalfunctionAlert::AltitudeReading))
    {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RangeFinder: Altitude Reading Overflow Warning");
    }
    if (_malfunction_alert_ & static_cast<uint16_t>(MalfunctionAlert::AttitudeAngle))
    {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RangeFinder: Excessive Attitude Angle Warning");
    }
    if (_malfunction_alert_ & static_cast<uint16_t>(MalfunctionAlert::FrameError_60G))
    {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RangeFinder: 60G Frame Error Warning");
    }
    if (_malfunction_alert_ & static_cast<uint16_t>(MalfunctionAlert::InvalidAltitude))
    {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RangeFinder: Invalid Altitude Warning");
    }
    if (_malfunction_alert_ & static_cast<uint16_t>(MalfunctionAlert::InConAlttitude))
    {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RangeFinder:24G & 60G Altitude Inconsistency Warning");
    }
    if (_malfunction_alert_ & static_cast<uint16_t>(MalfunctionAlert::BoardVoltage))
    {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RangeFinder: Whole Board Voltage Warning");
    }
}

#endif