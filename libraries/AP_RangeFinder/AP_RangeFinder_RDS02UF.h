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

#if AP_RANGEFINDER_RDS02UF_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

#define RDS02_BUFFER_SIZE           50
#define RDS02UF_DIST_MAX            20.00
#define RDS02UF_DIST_MIN            1.50
#define RDS02UF_DATA_LEN            10

class AP_RangeFinder_RDS02UF : public AP_RangeFinder_Backend_Serial
{

public:

    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params) {
        return NEW_NOTHROW AP_RangeFinder_RDS02UF(_state, _params);
    }

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_RADAR;
    }

private:
    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

    // find a RDS02UF message in the buffer, starting at
    // initial_offset.  If found, that message (or partial message) will
    // be moved to the start of the buffer.
    void move_header_in_buffer(uint8_t initial_offset);

    // get a distance reading
    bool get_reading(float &reading_m) override;
    uint16_t read_timeout_ms() const override { return 500; }

    // make sure readings go out-of-range when necessary
    float max_distance() const override  {
        return MIN(AP_RangeFinder_Backend::max_distance(), RDS02UF_DIST_MAX);
    }
    float min_distance() const override {
        return MAX(AP_RangeFinder_Backend::min_distance(), RDS02UF_DIST_MIN);
    }

     // Data Format for Benewake Rds02UF
    // ===============================
    // 21 bytes total per message:
    // 1) 0x55
    // 2) 0x55
    // 3) address
    // 4) error_code
    // 5) FC_CODE_L (low 8bit)
    // 6) FC_CODE_H (high 8bit)
    // 7) LENGTH_L (low 8bit)
    // 8) LENGTH_H (high 8bit)
    // 9) REAL_DATA (10Byte)
    // 10) CRC8
    // 11) END_1 0xAA
    // 12) END_2 0xAA
    struct PACKED RDS02UFPacket {
        uint8_t headermagic1;
        uint8_t headermagic2;
        uint8_t address;
        uint8_t error_code;
        uint8_t fc_low;
        uint8_t fc_high;
        uint8_t length_l;
        uint8_t length_h;
        uint8_t data[RDS02UF_DATA_LEN];
        uint8_t checksum;
        uint8_t footermagic1;
        uint8_t footermagic2;
    };

    union RDS02UF_Union {
        uint8_t parse_buffer[21];
        struct RDS02UFPacket packet;
    };
    RDS02UF_Union u;

    // number of bytes currently in the buffer
    uint8_t body_length;
};
#endif  // AP_RANGEFINDER_RDS02UF_ENABLED
