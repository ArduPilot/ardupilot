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

#include <AP_HAL/utility/sparse-endian.h>

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

    // generate a command to the sensor and place it in the message buffer
    void prepare_command(uint8_t reg, uint32_t value);

    // unreflected crc32 checksum with 0xffffffff initial value
    static uint32_t crc32_unreflected(uint8_t *buf, size_t n);

    // last temperature seen
    float _temperature;

    // packet structure
    struct Packet
    {
        uint8_t start;
        uint8_t reg;
        union
        {
            // single value for packets sent to the sensor
            struct 
            {
                le32_t value;
                le32_t checksum;
            } command __attribute__((packed));
            // sensor responce
            struct
            {
                // responces include a 16-bit data length although only 2 values are ever used
                le16_t len;
                union
                {
                    // generic 32-bit reply
                    struct
                    {
                        le32_t data;
                        le32_t checksum;
                    } responce __attribute__((packed));
                    // measurement response
                    struct
                    {
                        struct {
                            le16_t distance;
                            le16_t temperature;
                            le16_t amplitude;
                            le16_t ambient_light;
                            le64_t tof_phase;
                        } data __attribute__((packed));
                        le32_t checksum;
                    } measurement __attribute__((packed));
                };
            } __attribute__((packed));
        };

    } __attribute__((packed));

    // one message buffer
    union
    {
        Packet _packet;
        uint8_t _buf[sizeof(Packet)];
    };

    // current buffer position
    size_t _buf_pos = 0;

    // whether we are reading or writing
    bool _writing = false;

    // expected message register
    // 0 carries a special meaning of a state after reset
    uint8_t _exp_reg = 0;

    // last time we've received a valid reading
    uint32_t _last_read_ms = 0;
};

#endif  // AP_RANGEFINDER_DFROBOT_LIDAR07_ENABLED
