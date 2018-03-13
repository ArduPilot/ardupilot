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
//	Code by Michael Smith, Jordi Munoz and Jose Julio, DIYDrones.com
//
//	GPS configuration : Custom protocol per "DIYDrones Custom Binary Sentence Specification V1.1"
//
// Note - see AP_GPS_MTK16.h for firmware 1.6 and later.
//
#pragma once

#include "AP_GPS.h"
#include "GPS_Backend.h"
#include "AP_GPS_MTK_Common.h"

class AP_GPS_MTK : public AP_GPS_Backend {
public:
    AP_GPS_MTK(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port);

    bool read(void);

    static bool _detect(struct MTK_detect_state &state, uint8_t data);
    static void send_init_blob(uint8_t instance, AP_GPS &gps);

    const char *name() const override { return "MTK"; }

private:
    struct PACKED diyd_mtk_msg {
        int32_t latitude;
        int32_t longitude;
        int32_t altitude;
        int32_t ground_speed;
        int32_t ground_course;
        uint8_t satellites;
        uint8_t fix_type;
        uint32_t utc_time;
    };
    enum diyd_mtk_fix_type {
        FIX_NONE = 1,
        FIX_2D = 2,
        FIX_3D = 3
    };

    enum diyd_mtk_protocol_bytes {
        PREAMBLE1 = 0xb5,
        PREAMBLE2 = 0x62,
        MESSAGE_CLASS = 1,
        MESSAGE_ID = 5
    };

    // Packet checksum accumulators
    uint8_t         _ck_a;
    uint8_t         _ck_b;

    // State machine state
    uint8_t         _step;
    uint8_t         _payload_counter;

    // Receive buffer
    union PACKED {
        DEFINE_BYTE_ARRAY_METHODS
        diyd_mtk_msg msg;
    } _buffer;

    // Buffer parse & GPS state update
    void        _parse_gps();

    static const char _initialisation_blob[];
};
