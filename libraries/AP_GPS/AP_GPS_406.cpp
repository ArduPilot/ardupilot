// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
//
// 	GPS_406.cpp - 406 GPS library for Arduino
// 	Code by Michael Smith, Jason Short, Jordi Muñoz and Jose Julio. DIYDrones.com
// 	This code works with boards based on ATMega168/328 ATMega1280 (Serial port 1)
//
// 	This library is free software; you can redistribute it and / or
// 	modify it under the terms of the GNU Lesser General Public
// 	License as published by the Free Software Foundation; either
// 	version 2.1 of the License, or (at your option) any later version.

#include "../FastSerial/FastSerial.h"	// because we need to change baud rates... ugh.
#include "AP_GPS_406.h"
#include "WProgram.h"

static const char init_str[] = "$PSRF100,0,57600,8,1,0*37";

// Constructors ////////////////////////////////////////////////////////////////
AP_GPS_406::AP_GPS_406(Stream *s) : AP_GPS_SIRF(s)
{
}

// Public Methods ////////////////////////////////////////////////////////////////////
void AP_GPS_406::init(void)
{
	_change_to_sirf_protocol();		// Changes to SIRF protocol and sets baud rate
	_configure_gps(); 				// Function to configure GPS, to output only the desired msg's

	AP_GPS_SIRF::init();			// let the superclass do anything it might need here
}

// Private Methods //////////////////////////////////////////////////////////////

void 
AP_GPS_406::_configure_gps(void)
{
	const uint8_t gps_header[] 		= {0xA0, 0xA2, 0x00, 0x08, 0xA6, 0x00};
	const uint8_t gps_payload[] 	= {0x02, 0x04, 0x07, 0x09, 0x1B};
	const uint8_t gps_checksum[] 	= {0xA8, 0xAA, 0xAD, 0xAF, 0xC1};
	const uint8_t gps_ender[]		= {0xB0, 0xB3};
	
	for(int z = 0; z < 2; z++){
		for(int x = 0; x < 5; x++){
			_port->write(gps_header, sizeof(gps_header));	// Prints the msg header, is the same header for all msg..
			_port->write(gps_payload[x]);					// Prints the payload, is not the same for every msg
			for(int y = 0; y < 6; y++)						// Prints 6 zeros
				_port->write((uint8_t)0);
			_port->write(gps_checksum[x]);					// Print the Checksum
			_port->write(gps_ender[0]);						// Print the Ender of the string, is same on all msg's. 
			_port->write(gps_ender[1]);						// ender	
		}
	}	
}

// The EM406 defalts to NMEA at 4800bps.  We want to switch it to SiRF binary
// mode at a higher rate.
//
// The change is sticky, but only for as long as the internal supercap holds
// settings (usually less than a week).
//
void 
AP_GPS_406::_change_to_sirf_protocol(void)
{
	FastSerial	*fs = (FastSerial *)_port;	// this is a bit grody...

	fs->begin(4800);
	delay(300);
	_port->print(init_str);
	delay(300);

	fs->begin(9600);
	delay(300);
	_port->print(init_str);
	delay(300);

	fs->begin(GPS_406_BITRATE);
	delay(300);
	_port->print(init_str);
	delay(300);
}

