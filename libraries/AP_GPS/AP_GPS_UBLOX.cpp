// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
/*
	GPS_UBLOX.cpp - Ublox GPS library for Arduino
	Code by Jordi Muñoz and Jose Julio. DIYDrones.com
	This code works with boards based on ATMega168/328 and ATMega1280 (Serial port 1)

	This library is free software; you can redistribute it and / or
	modify it under the terms of the GNU Lesser General Public
	License as published by the Free Software Foundation; either
	version 2.1 of the License, or (at your option) any later version.

	GPS configuration : Ublox protocol
	Baud rate : 38400
	Active messages : 
		NAV - POSLLH Geodetic Position Solution, PAGE 66 of datasheet
		NAV - VELNED Velocity Solution in NED, PAGE 71 of datasheet
		NAV - STATUS Receiver Navigation Status
			or 
		NAV - SOL Navigation Solution Information

	Methods:
		init() : GPS initialization
		update() : Call this funcion as often as you want to ensure you read the incomming gps data
		
	Properties:
		Lattitude : Lattitude * 10000000 (long value)
		Longitude : Longitude * 10000000 (long value)
		altitude :	altitude * 100 (meters) (long value)
		ground_speed : Speed (m/s) * 100 (long value)
		ground_course : Course (degrees) * 100 (long value)
		new_data : 1 when a new data is received.
							You need to write a 0 to new_data when you read the data
		fix : 1: GPS FIX, 0: No fix (normal logic)
			
*/

#include "AP_GPS_UBLOX.h"
#include "WProgram.h"

// Constructors ////////////////////////////////////////////////////////////////
AP_GPS_UBLOX::AP_GPS_UBLOX(Stream *s) : GPS(s)
{
}


// Public Methods ////////////////////////////////////////////////////////////////////
void AP_GPS_UBLOX::init(void)
{
}

// optimization : This code doesn't wait for data, only proccess the data available
// We can call this function on the main loop (50Hz loop)
// If we get a complete packet this function calls parse_gps() to parse and update the GPS info.
void AP_GPS_UBLOX::update(void)
{
	byte data;
	int numc;

	numc = _port->available();

	if (numc > 0){
		for (int i = 0;i < numc;i++){	// Process bytes received
			data = _port->read();
		
		switch(step){
			case 0:
				if(data == 0xB5)
					step++;
				break;
				
			case 1:	
				if(data == 0x62)
					step++;
				else 
					step = 0;
				break;
				
			case 2:
				msg_class = data;
				checksum(msg_class);
				step++;
				break;
				
			case 3:
				id = data;
				checksum(id);
				step++;
				break;
				
			case 4:
				payload_length_hi = data;
				checksum(payload_length_hi);
				step++;
				// We check if the payload lenght is valid...
				if (payload_length_hi >= MAXPAYLOAD){
					_error("ERR:GPS_BAD_PAYLOAD_LENGTH!!\n");
					step = 0;	 // Bad data, so restart to step zero and try again.		 
					ck_a = 0;
					ck_b = 0;
				}
				break;
				
			case 5:
				payload_length_lo = data;
				checksum(payload_length_lo);
				step++;
				payload_counter = 0;
				break;
				
			case 6:				 // Payload data read...
				if (payload_counter < payload_length_hi){	// We stay in this state until we reach the payload_length
					buffer[payload_counter] = data;
					checksum(data);
					payload_counter++;
					if (payload_counter == payload_length_hi)
						step++;
				}
				break;
			case 7:
				GPS_ck_a = data;	 // First checksum byte
				step++;
				break;
				
			case 8:
				GPS_ck_b = data;	 // Second checksum byte	 
				// We end the GPS read...
				if((ck_a == GPS_ck_a) && (ck_b == GPS_ck_b)){	 // Verify the received checksum with the generated checksum.. 
					parse_gps();							 // Parse the new GPS packet
				}else{
					_error("ERR:GPS_CHK!!\n");
				}
				// Variable initialization
				step = 0;
				ck_a = 0;
				ck_b = 0;
				break;
			}
		}		// End for...
	}
}

// Private Methods //////////////////////////////////////////////////////////////
void
AP_GPS_UBLOX::parse_gps(void)
{
//Verifing if we are in msg_class 1, you can change this "IF" for a "Switch" in case you want to use other UBX classes.. 
// In this case all the message im using are in msg_class 1, to know more about classes check PAGE 60 of DataSheet.
	if(msg_class == 0x01){
		switch(id) {//Checking the UBX ID
		case 0x02: // ID NAV - POSLLH 
			time = *(long *)&buffer[0];				// ms Time of week
			longitude = *(long *)&buffer[4];		// lon * 10,000,000
			latitude = *(long *)&buffer[8];			// lat * 10,000,000
			//altitude = *(long *)&buffer[12];		// elipsoid heigth mm
			altitude = *(long *)&buffer[16] / 10;	// MSL heigth mm
			/*
			hacc = (float)*(long *)&buffer[20];
			vacc = (float)*(long *)&buffer[24];
			*/
			new_data = true;
			break;

		case 0x03: //ID NAV - STATUS 
			fix = ((buffer[4] >= 0x03) && (buffer[5] & 0x01));
			break;

		case 0x06: //ID NAV - SOL
			fix = ((buffer[10] >= 0x03) && (buffer[11] & 0x01));
			num_sats = buffer[47];										// Number of sats...		 
			break;

		case 0x12: // ID NAV - VELNED 
			speed_3d = *(long *)&buffer[16];			// cm / s
			ground_speed = *(long *)&buffer[20];		// Ground speed 2D cm / s
			ground_course = *(long *)&buffer[24] / 1000; // Heading 2D deg * 100000 rescaled to deg * 100
			break; 
		}
	}	 
}

// Ublox checksum algorithm
void AP_GPS_UBLOX::checksum(byte data)
{
	ck_a += data;
	ck_b += ck_a; 
}
