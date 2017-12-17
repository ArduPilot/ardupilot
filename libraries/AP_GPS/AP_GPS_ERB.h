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

//
//  Emlid Reach Binary (ERB) GPS driver for ArduPilot.
//  ERB protocol: http://files.emlid.com/ERB.pdf

#pragma once

#include <AP_HAL/AP_HAL.h>

#include "AP_GPS.h"
#include "GPS_Backend.h"

class AP_GPS_ERB : public AP_GPS_Backend
{
public:
    AP_GPS_ERB(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port);

    // Methods
    bool read();

    AP_GPS::GPS_Status highest_supported_status(void) { return AP_GPS::GPS_OK_FIX_3D_RTK_FIXED; }

    bool supports_mavlink_gps_rtk_message() { return true; }

    static bool _detect(struct ERB_detect_state &state, uint8_t data);

    const char *name() const override { return "ERB"; }

private:
    struct PACKED erb_header {
        uint8_t preamble1;
        uint8_t preamble2;
        uint8_t msg_id;
        uint16_t length;
    };
    struct PACKED erb_ver {
        uint32_t time;      ///< GPS time of week of the navigation epoch [ms]
        uint8_t ver_high;
        uint8_t ver_medium;
        uint8_t ver_low;
    };
    struct PACKED erb_pos {
        uint32_t time;      ///< GPS time of week of the navigation epoch [ms]
        double longitude;
        double latitude;
        double altitude_ellipsoid;    ///< Height above ellipsoid [m]
        double altitude_msl;          ///< Height above mean sea level [m]
        uint32_t horizontal_accuracy; ///< Horizontal accuracy estimate [mm]
        uint32_t vertical_accuracy;   ///< Vertical accuracy estimate [mm]
    };
    struct PACKED erb_stat {
        uint32_t time;      ///< GPS time of week of the navigation epoch [ms]
        uint16_t week;
        uint8_t fix_type;   ///< see erb_fix_type enum
        uint8_t fix_status;
        uint8_t satellites;
    };
    struct PACKED erb_dops {
        uint32_t time;      ///< GPS time of week of the navigation epoch [ms]
        uint16_t gDOP;      ///< Geometric DOP
        uint16_t pDOP;      ///< Position DOP
        uint16_t vDOP;      ///< Vertical DOP
        uint16_t hDOP;      ///< Horizontal DOP
    };
    struct PACKED erb_vel {
        uint32_t time;      ///< GPS time of week of the navigation epoch [ms]
        int32_t vel_north;  ///< North velocity component [cm/s]
        int32_t vel_east;   ///< East velocity component [cm/s]
        int32_t vel_down;   ///< Down velocity component [cm/s]
        uint32_t speed_2d;  ///< Ground speed (2-D) [cm/s]
        int32_t heading_2d; ///< Heading of motion 2-D [1e5 deg]
        uint32_t speed_accuracy; ///< Speed accuracy Estimate [cm/s]
    };
    struct PACKED erb_rtk {
        uint8_t base_num_sats;       ///< Current number of satellites used for RTK calculation
        uint16_t age_cs;             ///< Age of the corrections in centiseconds (0 when no corrections, 0xFFFF indicates overflow)
        int32_t baseline_N_mm;       ///< distance between base and rover along the north axis in millimeters
        int32_t baseline_E_mm;       ///< distance between base and rover along the east axis in millimeters
        int32_t baseline_D_mm;       ///< distance between base and rover along the down axis in millimeters
        uint16_t ar_ratio;           ///< AR ratio multiplied by 10
        uint16_t base_week_number;   ///< GPS Week Number of last baseline
        uint32_t base_time_week_ms;  ///< GPS Time of Week of last baseline in milliseconds
    };

    // Receive buffer
    union PACKED {
        DEFINE_BYTE_ARRAY_METHODS
        erb_ver ver;
        erb_pos pos;
        erb_stat stat;
        erb_dops dops;
        erb_vel vel;
        erb_rtk rtk;
    } _buffer;

    enum erb_protocol_bytes {
        PREAMBLE1 = 0x45,
        PREAMBLE2 = 0x52,
        MSG_VER = 0x01,
        MSG_POS = 0x02,
        MSG_STAT = 0x03,
        MSG_DOPS = 0x04,
        MSG_VEL = 0x05,
        MSG_RTK = 0x07,
    };

    enum erb_fix_type {
        FIX_NONE = 0x00,
        FIX_SINGLE = 0x01,
        FIX_FLOAT = 0x02,
        FIX_FIX = 0x03,
    };

    // Packet checksum accumulators
    uint8_t _ck_a;
    uint8_t _ck_b;

    // State machine state
    uint8_t _step;
    uint8_t _msg_id;
    uint16_t _payload_length;
    uint16_t _payload_counter;

    // 8 bit count of fix messages processed, used for periodic processing
    uint8_t _fix_count;

    uint32_t _last_pos_time;
    uint32_t _last_vel_time;

    // do we have new position information?
    bool _new_position:1;
    // do we have new speed information?
    bool _new_speed:1;

    // Buffer parse & GPS state update
    bool _parse_gps();

    // used to update fix between status and position packets
    AP_GPS::GPS_Status next_fix;
};
