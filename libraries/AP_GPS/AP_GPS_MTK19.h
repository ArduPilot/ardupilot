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
//  DIYDrones Custom Mediatek GPS driver for ArduPilot and ArduPilotMega.
//	Code by Michael Smith, Jordi Munoz and Jose Julio, Craig Elder, DIYDrones.com
//
//	GPS configuration : Custom protocol per "Customize Function Specification, 3D Robotics, v1.6, v1.7, v1.8, v1.9"
//
#pragma once

#include "AP_GPS.h"
#include "GPS_Backend.h"
#include "AP_GPS_MTK_Common.h"

#define MTK_GPS_REVISION_V16  16
#define MTK_GPS_REVISION_V19  19

class AP_GPS_MTK19 : public AP_GPS_Backend {
public:
    AP_GPS_MTK19(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port);

    bool        read(void);

    static bool _detect(struct MTK19_detect_state &state, uint8_t data);

    const char *name() const override { return "MTK19"; }

private:
    struct PACKED diyd_mtk_msg {
        int32_t latitude;
        int32_t longitude;
        int32_t altitude;
        int32_t ground_speed;
        int32_t ground_course;
        uint8_t satellites;
        uint8_t fix_type;
        uint32_t utc_date;
        uint32_t utc_time;
        uint16_t hdop;
    };
    enum diyd_mtk_fix_type {
        FIX_NONE = 1,
        FIX_2D = 2,
        FIX_3D = 3,
		FIX_2D_SBAS = 6,
        FIX_3D_SBAS = 7
    };

    enum diyd_mtk_protocol_bytes {
	    PREAMBLE1_V16 = 0xd0,
        PREAMBLE1_V19 = 0xd1,
        PREAMBLE2     = 0xdd,
    };

    // Packet checksum accumulators
    uint8_t         _ck_a;
    uint8_t         _ck_b;

    // State machine state
    uint8_t         _step;
    uint8_t         _payload_counter;
	uint8_t			_mtk_revision;

    uint8_t         _fix_counter;

    // Receive buffer
    union {
        DEFINE_BYTE_ARRAY_METHODS
        diyd_mtk_msg msg;
    } _buffer;
};
