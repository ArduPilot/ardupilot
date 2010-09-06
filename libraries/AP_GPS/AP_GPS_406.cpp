// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
/*
	GPS_406.cpp - 406 GPS library for Arduino
	Code by Jason Short, Jordi Muñoz and Jose Julio. DIYDrones.com
	This code works with boards based on ATMega168/328 ATMega1280 (Serial port 1)

	This library is free software; you can redistribute it and / or
	modify it under the terms of the GNU Lesser General Public
	License as published by the Free Software Foundation; either
	version 2.1 of the License, or (at your option) any later version.

	GPS configuration : 406 protocol
	Baud rate : 57600

	Methods:
		init() : GPS initialization
		update() : Call this funcion as often as you want to ensure you read the incomming gps data
		
	Properties:
		Latitude : Latitude * 10,000,000 (long value)
		Longitude : Longitude * 10,000,000 (long value)
		altitude :	altitude * 100 (meters) (long value)
		ground_speed : Speed (m/s) * 100 (long value)
		ground_course : Course (degrees) * 100 (long value)
		new_data : 1 when a new data is received.
				You need to write a 0 to new_data when you read the data
		fix : 1: GPS FIX, 0: No fix (normal logic)
			
*/

#include "../FastSerial/FastSerial.h"	// because we need to change baud rates... ugh.
#include "AP_GPS_406.h"
#include "WProgram.h"
#include <Stream.h>

uint8_t AP_GPS_406::buffer[MAXPAYLOAD] = {0x24,0x50,0x53,0x52,0x46,0x31,0x30,0x30,0x2C,0x30,0x2C,0x35,0x37,0x36,0x30,0x30,0x2C,0x38,0x2C,0x31,0x2C,0x30,0x2A,0x33,0x37,0x0D,0x0A};

#define PAYLOAD_LENGTH 92
#define BAUD_RATE 57600

// Constructors ////////////////////////////////////////////////////////////////
AP_GPS_406::AP_GPS_406(Stream *s) : GPS(s)
{
}

// Public Methods ////////////////////////////////////////////////////////////////////
void AP_GPS_406::init(void)
{
	change_to_sirf_protocol();		// Changes to SIRF protocol and sets baud rate
	delay(100);						// Waits fot the GPS to start_UP
	configure_gps(); 				// Function to configure GPS, to output only the desired msg's
}

// optimization : This code doesn´t wait for data, only proccess the data available
// We can call this function on the main loop (50Hz loop)
// If we get a complete packet this function calls parse_gps() to parse and update the GPS info.
void AP_GPS_406::update(void)
{
	byte data;
	int numc;

	numc = _port->available();
	
	if (numc > 0){
		for (int i = 0; i < numc; i++){	// Process bytes received
			data = _port->read();
		
		switch(step){
			case 0:
				if(data == 0xA0)
					step++;
				break;
				
			case 1:	
				if(data == 0xA2)
					step++;
				else
					step = 0;
				break;

/*			case 2:	
				if(data == 0xA2)
					step++;
				else
					step = 0;
				break;
*/
			case 2:	
				if(data == 0x00)
					step++;
				else
					step = 0;
				break;

			case 3:
				if(data == 0x5B)
					step++;
				else
					step = 0;
				break;

			case 4:
				if(data == 0x29){
					payload_counter = 0;
					step++;
				}else
					step = 0;
				break;
								
			case 5:	// Payload data read...
				if (payload_counter <= PAYLOAD_LENGTH){	// We stay in this state until we reach the payload_length
					buffer[payload_counter] = data;
					payload_counter++;
					if (payload_counter == PAYLOAD_LENGTH){
						parse_gps();
						step = 0;
					}
				}
				break;
			}
		} // End for...
	}
}

// Private Methods //////////////////////////////////////////////////////////////
void
AP_GPS_406::parse_gps(void)
{
	uint8_t j;

	fix = buffer[1] > 0;

	latitude     = _swapl(&buffer[22]);	// lat * 10, 000, 000
	longitude    = _swapl(&buffer[26]);	// lon * 10, 000, 000
	altitude     = _swapl(&buffer[34]);	// alt in meters * 100
	ground_speed = _swapi(&buffer[39]);	// meters / second * 100

	if (ground_speed >= 50) {			// Only updates data if we are really moving... 
		ground_course = (unsigned int)_swapi(&buffer[41]);	// meters / second * 100
	}

	//climb_rate = _swapi(&buffer[45]);	// meters / second * 100

	if (latitude == 0) {
		new_data = false;
		fix = false;
	} else {
		new_data = true;
	}
}

void 
AP_GPS_406::configure_gps(void)
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

void 
AP_GPS_406::change_to_sirf_protocol(void)
{
	FastSerial	*fs = (FastSerial *)_port;	// this is a bit grody...

	fs->begin(4800);				// First try at 4800bps
	delay(300);
	_port->write(buffer, 28);		// Sending special bytes declared at the beginning 
	delay(300);
	
	fs->begin(9600);				// Then try at 9600bps
	delay(300);
	_port->write(buffer, 28);
	delay(300);
	
	fs->begin(BAUD_RATE);			// Finally try at the configured bit rate (57600?)
	delay(300);
	_port->write(buffer, 28);
}

