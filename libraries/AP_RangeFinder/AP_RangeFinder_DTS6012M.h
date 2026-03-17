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

#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_DTS6012M_ENABLED

#include "AP_RangeFinder_Backend_Serial.h"
#include <AP_HAL/utility/sparse-endian.h>


class AP_RangeFinder_DTS6012M : public AP_RangeFinder_Backend_Serial
{
public:
    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params)
    {
        return NEW_NOTHROW AP_RangeFinder_DTS6012M(_state, _params);
    }

protected:
    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override { return MAV_DISTANCE_SENSOR_LASER; }

private:
    bool get_reading(float &reading_m) override;
    uint32_t initial_baudrate(uint8_t serial_instance) const override { return 921600; }
    int8_t get_signal_quality_pct() const override { return _signal_quality_pct; }

    // protocol parsing state
    // response frame layout matching the DTS6012M wire format
    struct PACKED {
        uint8_t header;         // 0xA5
        uint8_t device_id;      // 0x03
        uint8_t device_type;    // 0x20
        uint8_t cmd_echo;       // 0x01
        uint8_t reserved;
        be16_t data_len;        // big-endian length
        // measurement data (14 bytes)
        le16_t secondary_distance_mm;
        le16_t secondary_correction;
        le16_t secondary_intensity;
        le16_t primary_distance_mm;
        le16_t primary_correction;
        le16_t primary_intensity;
        le16_t sunlight_base;
        // CRC16 (big-endian)
        be16_t crc;
    } frame;
    uint8_t linebuf_len;
    bool got_reading;
    int8_t _signal_quality_pct = RangeFinder::SIGNAL_QUALITY_UNKNOWN;

    // send start stream command to sensor
    void send_start_command();

    // search for frame header byte starting at given offset,
    // shifting buffer contents to re-sync after line noise
    void find_signature_in_buffer(uint8_t start);
};

#endif  // AP_RANGEFINDER_DTS6012M_ENABLED
