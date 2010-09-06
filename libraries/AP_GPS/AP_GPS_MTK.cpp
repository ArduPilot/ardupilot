// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
/*
	GPS_MTK.cpp - Ublox GPS library for Arduino
	Code by Jordi Munoz and Jose Julio. DIYDrones.com
	This code works with boards based on ATMega168 / 328 and ATMega1280 (Serial port 1)

	This library is free software; you can redistribute it and / or
	modify it under the terms of the GNU Lesser General Public
	License as published by the Free Software Foundation; either
	version 2.1 of the License, or (at your option) any later version.

	GPS configuration : Custum protocol
	Baud rate : 38400

	Methods:
		init() : GPS initialization
		update() : Call this funcion as often as you want to ensure you read the incomming gps data
		
	Properties:
		lattitude : lattitude * 10000000 (long value)
		longitude : longitude * 10000000 (long value)
		altitude :	altitude * 100 (meters) (long value)
		ground_speed : Speed (m/s) * 100 (long value)
		ground_course : Course (degrees) * 100 (long value)
		new_data : 1 when a new data is received.
							You need to write a 0 to new_data when you read the data
		Fix : 0: GPS NO FIX or 2D FIX, 1: 3D FIX.
			
*/

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

// optimization : This code doesn't wait for data, only proccess the data available
// We can call this function on the main loop (50Hz loop)
// If we get a complete packet this function calls parse_gps() to parse and update the GPS info.
void AP_GPS_MTK::update(void)
{
	byte data;
	int numc;

	numc = _port->available();
	if (numc > 0)
		for (int i = 0; i < numc; i++){	// Process bytes received
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
				step = 4;
				payload_length_hi = 26;
				payload_length_lo = 0;
				payload_counter = 0;
				checksum(id);
				break;

			case 4:
				if (payload_counter < payload_length_hi){	// We stay in this state until we reach the payload_length
					buffer[payload_counter] = data;
					checksum(data);
					payload_counter++;
					if (payload_counter == payload_length_hi)
						step++;
				}
				break;
			case 5:
				GPS_ck_a = data;	 // First checksum byte
				step++;
				break;
			case 6:
				GPS_ck_b = data;	 // Second checksum byte
				// We end the GPS read...
				if((ck_a == GPS_ck_a) && (ck_b == GPS_ck_b)){	 // Verify the received checksum with the generated checksum.. 
					parse_gps();							 // Parse the new GPS packet
				}else {
					_error("ERR:GPS_CHK!!\n");
				}
				// Variable initialization
				step = 0;
				ck_a = 0;
				ck_b = 0;
				break;
		} // End switch
	} // End for
}


// Private Methods 
void 
AP_GPS_MTK::parse_gps(void)
{
//Verifing if we are in class 1, you can change this "IF" for a "Switch" in case you want to use other UBX classes.. 
//In this case all the message im using are in class 1, to know more about classes check PAGE 60 of DataSheet.
	if(msg_class == 0x01) {
		switch(id){
			//Checking the UBX ID
			case 0x05: // ID Custom
				latitude      = _swapl(&buffer[0]) * 10;
				longitude     = _swapl(&buffer[4]) * 10;
				altitude      = _swapl(&buffer[8]);
				speed_3d      = ground_speed = _swapl(&buffer[12]);
				ground_course = _swapl(&buffer[16]) / 10000;
				num_sats      = buffer[20];
				fix           = buffer[21] == 3;
				time          = _swapl(&buffer[22]);
				new_data      = true;
				break;
		}
	}
}

// checksum algorithm
void
AP_GPS_MTK::checksum(byte data)
{
	ck_a += data;
	ck_b += ck_a; 
}
