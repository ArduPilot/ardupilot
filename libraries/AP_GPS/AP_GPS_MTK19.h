// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
//
//  DIYDrones Custom Mediatek GPS driver for ArduPilot and ArduPilotMega.
//	Code by Michael Smith, Jordi Munoz and Jose Julio, Craig Elder, DIYDrones.com
//
//	This library is free software; you can redistribute it and / or
//	modify it under the terms of the GNU Lesser General Public
//	License as published by the Free Software Foundation; either
//	version 2.1 of the License, or (at your option) any later version.
//
//	GPS configuration : Custom protocol per "Customize Function Specification, 3D Robotics, v1.6, v1.7, v1.8, v1.9"
//
#ifndef AP_GPS_MTK19_h
#define AP_GPS_MTK19_h

#include "GPS.h"
#include "AP_GPS_MTK_Common.h"

#define MTK_GPS_REVISION_V16  16
#define MTK_GPS_REVISION_V19  19


class AP_GPS_MTK19 : public GPS {
public:
    AP_GPS_MTK19();
    virtual void        init(AP_HAL::UARTDriver *s, enum GPS_Engine_Setting nav_setting = GPS_ENGINE_NONE);
    virtual bool        read(void);
    static bool 		_detect(uint8_t );

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
        uint32_t utc_date;
        uint32_t utc_time;
        uint16_t hdop;
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
	    PREAMBLE1 = 0xd0,
        PREAMBLE2 = 0xdd,
	    PREAMBLE1_V16 = 0xd0,
        PREAMBLE2_V16 = 0xdd,
        PREAMBLE1_V19 = 0xd1,
        PREAMBLE2_V19 = 0xdd
    };

    // Packet checksum accumulators
    uint8_t         _ck_a;
    uint8_t         _ck_b;

    // State machine state
    uint8_t         _step;
    uint8_t         _payload_counter;
	uint8_t			_mtk_type_step1;
	uint8_t 		_mtk_type_step2;

    // Time from UNIX Epoch offset
    long            _time_offset;
    bool            _offset_calculated;

    // Receive buffer
    union {
        diyd_mtk_msg msg;
        uint8_t bytes[];
    } _buffer;
};

#endif  // AP_GPS_MTK19_H
