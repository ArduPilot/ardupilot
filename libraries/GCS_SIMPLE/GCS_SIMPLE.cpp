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

#include "GCS_SIMPLE.h"
#include <stdint.h>

// Public Methods //////////////////////////////////////////////////////////////
GCS_SIMPLE::GCS_SIMPLE(Stream *s) : _port(s)
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

void
GCS_SIMPLE::ack(void)
{
    buff_pointer = 0;
    flush(1);
}

bool
GCS_SIMPLE::read(void)
{
    uint8_t data;
    int     numc;
    bool    parsed = false;

    numc = _port->available();

    for (int i = 0; i < numc; i++){
        // Process bytes received

        // read the next byte
        data = _port->read();

    	//_port->write(data);

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
                _length = data;
                _step++;
                break;

            case 3:
                _id = data;
                _step++;
                break;

            case 4:
                _buffer.bytes[_payload_counter++] = data;
                if (_payload_counter == sizeof(_buffer)){
                    _step = 0;
                    parsed = true;
                }

                index       =  _buffer.msg.index;
                id          =  _buffer.msg.id;
                p1          =  _buffer.msg.p1;
                altitude    =  _buffer.msg.altitude;
                latitude    =  _buffer.msg.latitude;
                longitude   =  _buffer.msg.longitude;
                break;
            }
    }
    return parsed;
}


// Add binary values to the buffer
void
GCS_SIMPLE::write_byte(uint8_t val)
{
	mess_buffer[buff_pointer++] = val;
}

void
GCS_SIMPLE::write_int(int val )
{
	int_out.value = val;
	mess_buffer[buff_pointer++] = int_out.bytes[0];
	mess_buffer[buff_pointer++] = int_out.bytes[1];
}

void
GCS_SIMPLE::write_float(float val)
{
	double_out.float_value = val;
	mess_buffer[buff_pointer++] = double_out.bytes[0];
	mess_buffer[buff_pointer++] = double_out.bytes[1];
	mess_buffer[buff_pointer++] = double_out.bytes[2];
	mess_buffer[buff_pointer++] = double_out.bytes[3];
}

void
GCS_SIMPLE::write_long(long val)
{
	double_out.long_value = val;
	mess_buffer[buff_pointer++] = double_out.bytes[0];
	mess_buffer[buff_pointer++] = double_out.bytes[1];
	mess_buffer[buff_pointer++] = double_out.bytes[2];
	mess_buffer[buff_pointer++] = double_out.bytes[3];
}

void
GCS_SIMPLE::flush(uint8_t msg_id)
{
	_port->print("4D");  			// This is the message preamble
	_port->write(buff_pointer);  	// Length
	_port->write(msg_id);  				// id

	for (uint8_t i = 0; i < buff_pointer; i++) {
		_port->write(mess_buffer[i]);
	}

	buff_pointer = 0;
}

