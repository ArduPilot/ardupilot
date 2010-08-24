/*
	AP_GPS_UBLOX.cpp - Ublox GPS library for Arduino
	Code by Jordi Muñoz and Jose Julio. DIYDrones.com
	This code works with boards based on ATMega168 / 328 and ATMega1280 (Serial port 1)

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
		Ground_speed : Speed (m / s) * 100 (long value)
		Ground_course : Course (degrees) * 100 (long value)
		new_data : 1 when a new data is received.
							You need to write a 0 to new_data when you read the data
		fix : 1: GPS FIX, 0: No fix (normal logic)
			
*/

#include "AP_GPS_UBLOX.h"

// Constructors // /// /// /// /// /// /// /// /// /// /// /// /// /// /// /// /// /// /// /// /// //
AP_GPS_UBLOX::AP_GPS_UBLOX()
{
	print_errors = 1;
}


// Public Methods // /// /// /// /// /// /// /// /// /// /// /// /// /// /// /// /// /// /// /// ///
void AP_GPS_UBLOX::init(void)
{
	ck_a 		= 0;
	ck_b 		= 0;
	step 		= 0;
	new_data 	= 0;
	fix 		= 0;
	print_errors = 0;


	// initialize serial port
	#if defined(__AVR_ATmega1280__)
		Serial1.begin(38400);				 // Serial port 1 on ATMega1280
	#else
		Serial.begin(38400);
	#endif
}

// optimization : This code don´t wait for data, only proccess the data available
// We can call this function on the main loop (50Hz loop)
// If we get a complete packet this function calls parse_gps() to parse and update the GPS info.
void AP_GPS_UBLOX::update(void)
{
	byte data;
	int numc;
	
	#if defined(__AVR_ATmega1280__)		// If AtMega1280 then Serial port 1...
	numc = Serial1.available();
	#else
	numc = Serial.available();
	#endif
	Serial.print("numc ");
	Serial.println(numc,DEC);
	
	if (numc > 0){
		Serial.println(" ");
		for (int i = 0;i < numc;i++){	// Process bytes received
		#if defined(__AVR_ATmega1280__)
			data = Serial1.read();
		#else
			data = Serial.read();
		#endif
		Serial.print(data,HEX);
		Serial.println(",");
		switch(step){		 // Normally we start from zero. This is a state machine
			case 0:	
				if(data == 0xB5)	// UBX sync char 1
					step++;	 // OH first data packet is correct, so jump to the next step
				break;
				
			case 1:	
				if(data == 0x62)	// UBX sync char 2
					step++;	 // ooh! The second data packet is correct, jump to the step 2
				else 
					step = 0;	 // Nop, is not correct so restart to step zero and try again.		 
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
					if (print_errors)
						Serial.println("ERR:GPS_BAD_PAYLOAD_LENGTH!!");					
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
					if (print_errors) Serial.println("ERR:GPS_CHK!!");
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
void AP_GPS_UBLOX::parse_gps(void)
{
	int j;
//Verifing if we are in msg_class 1, you can change this "IF" for a "Switch" in case you want to use other UBX classes.. 
// In this case all the message im using are in msg_class 1, to know more about classes check PAGE 60 of DataSheet.
	if(msg_class == 0x01) 
	{
		switch(id) //Checking the UBX ID
		{
		case 0x02: // ID NAV - POSLLH 
			j = 0;
			time = join_4_bytes(&buffer[j]); // ms Time of week
			j += 4;
			longitude = join_4_bytes(&buffer[j]); // lon * 10000000
			j += 4;
			lattitude = join_4_bytes(&buffer[j]); // lat * 10000000
			j += 4;
			//altitude = join_4_bytes(&buffer[j]);	// elipsoid heigth mm
			j += 4;
			altitude = (float)join_4_bytes(&buffer[j]);	// MSL heigth mm
			//j+=4;
			/*
			hacc = (float)join_4_bytes(&buffer[j]) / (float)1000;
			j += 4;
			vacc = (float)join_4_bytes(&buffer[j]) / (float)1000;
			j += 4;
			*/
			new_data = 1;
			break;

		case 0x03: //ID NAV - STATUS 
			//if(buffer[4] >= 0x03)
		if((buffer[4] >= 0x03) && (buffer[5] & 0x01))				
				fix = 1; // valid position				
			else
				fix = 0; // invalid position
			break;

		case 0x06: //ID NAV - SOL
			if((buffer[10] >= 0x03) && (buffer[11] & 0x01))
				fix = 1; // valid position
			else
				fix = 0; // invalid position				
			//ecefVZ = join_4_bytes(&buffer[36]);	// Vertical Speed in cm / s
			num_sats = buffer[47];										// Number of sats...		 
			break;

		case 0x12: // ID NAV - VELNED 
			j = 16;
			speed_3d = join_4_bytes(&buffer[j]); // cm / s
			j += 4;
			ground_speed = join_4_bytes(&buffer[j]); // Ground speed 2D cm / s
			j += 4;
			ground_course = join_4_bytes(&buffer[j]); // Heading 2D deg * 100000
			ground_course /= 1000;	// Rescale heading to deg * 100
			j += 4;
			break; 
			}
		}	 
}

 // Join 4 bytes into a long
long AP_GPS_UBLOX::join_4_bytes(unsigned char Buffer[])
{
	longUnion.byte[0] = *Buffer;
	longUnion.byte[1] = *(Buffer + 1);
	longUnion.byte[2] = *(Buffer + 2);
	longUnion.byte[3] = *(Buffer + 3);
	return(longUnion.dword);
}

// Ublox checksum algorithm
void AP_GPS_UBLOX::checksum(byte data)
{
	ck_a += data;
	ck_b += ck_a; 
}
