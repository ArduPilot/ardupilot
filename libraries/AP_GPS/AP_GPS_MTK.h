// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
//
//  DIYDrones Custom Mediatek GPS driver for ArduPilot and ArduPilotMega.
//	Code by Michael Smith, Jordi Munoz and Jose Julio, DIYDrones.com
//
//	This library is free software; you can redistribute it and / or
//	modify it under the terms of the GNU Lesser General Public
//	License as published by the Free Software Foundation; either
//	version 2.1 of the License, or (at your option) any later version.
//
//	GPS configuration : Custom protocol per "DIYDrones Custom Binary Sentence Specification V1.1"
//
// Note - see AP_GPS_MTK16.h for firmware 1.6 and later.
//
#ifndef __AP_GPS_MTK_H__
#define __AP_GPS_MTK_H__

#include "GPS.h"
#include "AP_GPS_MTK_Common.h"

class AP_GPS_MTK : public GPS {
public:
    AP_GPS_MTK(AP_HAL::UARTDriver *s);
    virtual void        init(enum GPS_Engine_Setting nav_setting = GPS_ENGINE_NONE);
    virtual bool        read(void);
    static bool _detect(uint8_t );

private:
// XXX this is being ignored by the compiler #pragma pack(1)
    struct diyd_mtk_msg {
        int32_t latitude;
        int32_t longitude;
        int32_t altitude;
        int32_t ground_speed;
        int32_t ground_course;
        uint8_t satellites;
        uint8_t fix_type;
        uint32_t utc_time;
    };
// #pragma pack(pop)
    enum diyd_mtk_fix_type {
        FIX_NONE = 1,
        FIX_2D = 2,
        FIX_3D = 3,
		FIX_2D_SBAS = 6,
		FIX_3D_SBAS = 7
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
    union {
        diyd_mtk_msg msg;
        uint8_t bytes[];
    } _buffer;

    // Buffer parse & GPS state update
    void        _parse_gps();
};

#endif  // __AP_GPS_MTK_H__
