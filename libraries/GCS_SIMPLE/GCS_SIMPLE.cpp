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

#include "GPS_SIMPLE.h"
#include <stdint.h>

// Public Methods //////////////////////////////////////////////////////////////
GPS_SIMPLE::GPS_SIMPLE(Stream *s) : _port(s)
{
}

// Process bytes available from the stream
//
// The stream is assumed to contain only our custom message.  If it
// contains other messages, and those messages contain the preamble bytes,
// it is possible for this code to become de-synchronised.  Without
// buffering the entire message and re-processing it from the top,
// this is unavoidable.
//
// The lack of a standard header length field makes it impossible to skip
// unrecognised messages.
//

bool
GPS_SIMPLE::read(void)
{
/*
    uint8_t data;
    int     numc;
    bool    parsed = false;

    numc = _port->available();

    for (int i = 0; i < numc; i++){
        // Process bytes received

        // read the next byte
        data = _port->read();

        restart:
        switch(_step){
            case 0:
                if(52 == data){
                    _step++;
                    _payload_counter = 0;
                }
                break;

            case 1:
                if (68 == data) {
                    _step++;
                    break;
                }
                _step = 0;
                goto restart;

            case 2:
                length = data;
                _step++;
                break;

            case 3:
                id = data;
                _step++;
                break;

            case 4:
                _buffer.bytes[_payload_counter++] = data;
                if (_payload_counter == sizeof(_buffer)){
                    _step = 0;
                    parsed = true;
                }
                break;
            }
    }
    return parsed;
    */
    return true;
}

