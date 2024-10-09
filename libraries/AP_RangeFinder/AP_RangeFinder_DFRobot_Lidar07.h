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

#if AP_RANGEFINDER_DFROBOT_LIDAR07_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

class AP_RangeFinder_DFRobot_Lidar07 : public AP_RangeFinder_Backend_Serial
{

public:

    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params) {
        return NEW_NOTHROW AP_RangeFinder_DFRobot_Lidar07(_state, _params);
    }

protected:

    // 850 nm LED
    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_INFRARED;
    }

private:

    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

    // 115200 baud as per datasheet
    uint32_t initial_baudrate(uint8_t serial_instance) const override { return 115200; }

    // get a distance reading
    bool get_reading(float &reading_m) override;

    // get the latest encountered temperature
    bool get_temp(float &temp) const override;

    // read the UART or a response on the register reg for a message
    // not exceeding read_max bytes read in total
    // stores the full message in the buffer on success
    // stores the partial message in the buffer if was not able to read it in full
    // returns true if a valid message was found
    bool scan_input(uint8_t reg, size_t read_max);

    // send a command over to the sensor
    void send_command(uint8_t reg, uint32_t value);

    // longest message size: the measurement one
    static constexpr size_t _buf_max = 1 + 1 + 2 + 16 + 4;

    // unreflected crc32 checksum with 0xffffffff initial value
    static uint32_t crc32_unreflected(uint8_t *buf, size_t n);

    // whether we have initialized the sensor or not
    bool _initialized = false;

    // one message buffer
    uint8_t _buf[_buf_max];
    size_t _buf_pos = 0;

    // last time we've received a valid reading
    uint32_t _last_read_ms = 0;

    // last temperature seen
    float _temperature;

    // currently unused, but captured
    // NOTE: taken as is from the wire without byte order conversions
    uint16_t _amplitude;
    uint16_t _ambient_light;
    uint64_t _tof_phase;
};

#endif  // AP_RANGEFINDER_DFROBOT_LIDAR07_ENABLED
