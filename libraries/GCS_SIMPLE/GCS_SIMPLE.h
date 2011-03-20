// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
//
//  DIYDrones Custom Mediatek GPS driver for ArduPilot and ArduPilotMega.
//  Code by Michael Smith, Jordi Munoz and Jose Julio, DIYDrones.com
//
//  This library is free software; you can redistribute it and / or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  GPS configuration : Custom protocol per "DIYDrones Custom Binary Sentence Specification V1.1"
//
// Note - see GPS_SIMPLE16.h for firmware 1.6 and later.
//
#ifndef GPS_SIMPLE_h
#define GPS_SIMPLE_h

#include <GCS_MAVLink.h>    // MAVLink GCS definitions

class GPS_SIMPLE {
public:
    GPS_SIMPLE(Stream *s);
    virtual bool    read(void);
	Stream	*_port;			///< port the GPS is attached to

private:
#pragma pack(1)
    struct diyd_mtk_msg {
        int8_t      id;
        int8_t      p1;
        int32_t     altitude;
        int32_t     latitude;
        int32_t     longitude;
    };
#pragma pack(pop)

    // State machine state
    uint8_t     _step;
    uint8_t     _payload_counter;
    uint8_t     length;
    uint8_t     id;

    // Receive buffer
    union {
        diyd_mtk_msg    msg;
        uint8_t         bytes[];
    } _buffer;
};

#endif  // GPS_SIMPLE_H
