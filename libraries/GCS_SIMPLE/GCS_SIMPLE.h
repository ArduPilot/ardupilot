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
// Note - see GCS_SIMPLE16.h for firmware 1.6 and later.
//
#ifndef GCS_SIMPLE_h
#define GCS_SIMPLE_h

#include <GCS_MAVLink.h>    // MAVLink GCS definitions

class GCS_SIMPLE {
public:
    GCS_SIMPLE(Stream *s);
    bool    read(void);
    void    ack(void);
    void    write_byte(uint8_t val);
    void    write_int(int val);
    void    write_float(float val);
    void    write_long(long val);
    void    flush(uint8_t msg_id);

	Stream	*_port;			///< port the GPS is attached to

    int8_t      index;
    int8_t      id;
    int8_t      p1;
    int32_t     altitude;
    int32_t     latitude;
    int32_t     longitude;

#pragma pack(1)
    struct diyd_mtk_msg {
        int8_t      index;
        int8_t      id;
        int8_t      p1;
        int32_t     altitude;
        int32_t     latitude;
        int32_t     longitude;
    };
#pragma pack(pop)

    // Receive buffer
    union {
        diyd_mtk_msg    msg;
        uint8_t         bytes[];
    } _buffer;

private:
    // State machine state
    // incoming
    uint8_t     _step;
    uint8_t     _payload_counter;
    uint8_t     _length;
    uint8_t     _id;

    // outgoing
    union d_out{
        uint8_t bytes[4];
        int32_t long_value;
        float float_value;
    } double_out;

    union i_out {
        uint8_t bytes[2];
        int16_t value;
    } int_out;

    uint8_t mess_buffer[50];
    uint8_t buff_pointer;
};

#endif  // GCS_SIMPLE_H
