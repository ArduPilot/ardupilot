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

#include "AP_GPS_MTK.h"
#include "WProgram.h"

// Constructors ////////////////////////////////////////////////////////////////
AP_GPS_MTK::AP_GPS_MTK(Stream *s) : GPS(s)
{
}

// Public Methods //////////////////////////////////////////////////////////////
void AP_GPS_MTK::init(void)
{	
	// initialize serial port for binary protocol use
	_port->print(MTK_SET_BINARY);
	_port->print(MTK_OUTPUT_4HZ);
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
void AP_GPS_MTK::update(void)
{
	byte data;
	int numc;

	numc = _port->available();
	for (int i = 0; i < numc; i++){	// Process bytes received

		// read the next byte
		data = _port->read();

restart:		
		switch(step){

			// Message preamble, class, ID detection
			//
			// If we fail to match any of the expected bytes, we
			// reset the state machine and re-consider the failed
			// byte as the first byte of the preamble.  This 
			// improves our chances of recovering from a mismatch
			// and makes it less likely that we will be fooled by
			// the preamble appearing as data in some other message.
			//
		case 0:
			if(PREAMBLE1 == data)
				step++;
			break;
		case 1:
			if (PREAMBLE2 == data) {
				step++;
				break;
			}
			step = 0;
			goto restart;
		case 2:
			if (MESSAGE_CLASS == data) {
				step++;
				ck_b = ck_a = data;					// reset the checksum accumulators
			} else {
				step = 0;							// reset and wait for a message of the right class
				goto restart;
			}
			break;
		case 3:
			if (MESSAGE_ID == data) {
				step++;
				ck_b += (ck_a += data);
				payload_length = sizeof(buffer);	// prepare to receive our message
				payload_counter = 0;
			} else {
				step = 0;
				goto restart;
			}
			break;

			// Receive message data
			//
		case 4:
			buffer.bytes[payload_counter++] = data;
			ck_b += (ck_a += data);
			if (payload_counter == payload_length)
				step++;
			break;

			// Checksum and message processing
			//
		case 5:
			step++;
			if (ck_a != data) {
				_error("GPS_MTK: checksum error\n");
				step = 0;
			}
			break;
		case 6:
			step = 0;
			if (ck_b != data) {
				_error("GPS_MTK: checksum error\n");
				break;
			}
			_parse_gps();							 // Parse the new GPS packet
		}
	} 
}


// Private Methods 
void 
AP_GPS_MTK::_parse_gps(void)
{
	if (FIX_3D != buffer.msg.fix_type) {
		fix = false;
	} else {
		fix = true;
		latitude		= _swapl(&buffer.msg.latitude)  * 10;
		longitude		= _swapl(&buffer.msg.longitude) * 10;
		altitude		= _swapl(&buffer.msg.altitude);
		ground_speed	= _swapl(&buffer.msg.ground_speed);
		ground_course	= _swapl(&buffer.msg.ground_course) / 10000;
		num_sats		= buffer.msg.satellites;
			
		// XXX docs say this is UTC, but our clients expect msToW
		time			= _swapl(&buffer.msg.utc_time);
	}
	new_data = true;
}
