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

#include "AP_GPS_406.h"
#include "WProgram.h"

AP_GPS_406::buffer[MAXPAYLOAD] = {0x24,0x50,0x53,0x52,0x46,0x31,0x30,0x30,0x2C,0x30,0x2C,0x35,0x37,0x36,0x30,0x30,0x2C,0x38,0x2C,0x31,0x2C,0x30,0x2A,0x33,0x37,0x0D,0x0A};


#define PAYLOAD_LENGTH 92
#define BAUD_RATE 57600

// Constructors ////////////////////////////////////////////////////////////////
AP_GPS_406::AP_GPS_406()
{
}


// Public Methods ////////////////////////////////////////////////////////////////////
void AP_GPS_406::init(void)
{
	change_to_sirf_protocol();
	delay(100); //Waits fot the GPS to start_UP
	configure_gps(); //Function to configure GPS, to output only the desired msg's

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
void AP_GPS_406::update(void)
{
	byte data;
	int numc;
	
	#if defined(__AVR_ATmega1280__)		// If AtMega1280 then Serial port 1...
		numc = Serial1.available();
	#else
		numc = Serial.available();
	#endif
	
	if (numc > 0){
		for (int i = 0; i < numc; i++){	// Process bytes received
		#if defined(__AVR_ATmega1280__)
			data = Serial1.read();
		#else
			data = Serial.read();
		#endif
		
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

			case 2:	
				if(data == 0xA2)
					step++;
				else
					step = 0;
				break;

			case 3:	
				if(data == 0x00)
					step++;
				else
					step = 0;
				break;

			case 4:
				if(data == 0x5B)
					step++;
				else
					step = 0;
				break;

			case 5:
				if(data == 0x29){
					payload_counter = 0;
					step++;
				}else
					step = 0;
				break;
								
			case 6:	// Payload data read...
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

	fix = (buffer[1] > 0) ? 1:0;

	j = 22;
	lattitude = join_4_bytes(&buffer[j]); // lat * 10, 000, 000

	j = 26;
	longitude = join_4_bytes(&buffer[j]); // lon * 10, 000, 000

	j = 34;
	altitude = join_4_bytes(&buffer[j]);	// alt in meters * 100
	
	j = 39;
	ground_speed = join_2_bytes(&buffer[j]);	// meters / second * 100

	if(ground_speed >= 50){
		//Only updates data if we are really moving... 
		j = 41;
		ground_course 		= (unsigned int)join_2_bytes(&buffer[j]);	// meters / second * 100
	}

	j = 45;
	//climb_rate = join_2_bytes(&buffer[j]);	// meters / second * 100

	if(lattitude == 0){
		new_data = false;
		fix = 0;
	}else{
		new_data = true;
	}
}

 // Join 4 bytes into a long
int32_t
AP_GPS_406::join_4_bytes(unsigned char Buffer[])
{
	longUnion.byte[3] = *Buffer;
	longUnion.byte[2] = *(Buffer + 1);
	longUnion.byte[1] = *(Buffer + 2);
	longUnion.byte[0] = *(Buffer + 3);
	return(longUnion.dword);
}

// Join 2 bytes into an int
int16_t
AP_GPS_406::join_2_bytes(unsigned char Buffer[])
{
	intUnion.byte[1] = *Buffer;
	intUnion.byte[0] = *(Buffer + 1);
	return(intUnion.word);
}

void 
AP_GPS_406::configure_gps(void)
{
	const uint8_t gps_header[] 		= {0xA0, 0xA2, 0x00, 0x08, 0xA6, 0x00};
	const uint8_t gps_payload[] 	= {0x02, 0x04, 0x07, 0x09, 0x1B};
	const uint8_t gps_checksum[] 	= {0xA8, 0xAA, 0xAD, 0xAF, 0xC1};
	const uint8_t gps_ender[]		= {0xB0, 0xB3};
	const uint8_t cero 				= 0x00;
	
	for(int z = 0; z < 2; z++){
		for(int x = 0; x < 5; x++){
			for(int y = 0; y < 6; y++){
				Serial.print(gps_header[y]); // Prints the msg header, is the same header for all msg..	
			}
			Serial.print(gps_payload[x]); // Prints the payload, is not the same for every msg
			for(int y = 0; y < 6; y++){
				Serial.print(cero); // Prints 6 zeros
			}
			Serial.print(gps_checksum[x]); // Print the Checksum
			Serial.print(gps_ender[0]);	// Print the Ender of the string, is same on all msg's. 
			Serial.print(gps_ender[1]);	// ender	
		}
	}	
}

void 
AP_GPS_406::change_to_sirf_protocol(void)
{
	Serial.begin(4800); // First try in 4800
	
	delay(300);
	
	for (byte x = 0; x <= 28; x++){
		Serial.print(buffer[x]); // Sending special bytes declared at the beginning 
	}
	delay(300);
	
	Serial.begin(9600); // Then try in 9600 
	
	delay(300);
	
	for (byte x = 0; x <= 28; x++){
		Serial.print(buffer[x]);
	}
	Serial.begin(BAUD_RATE); // Universal Sincronus Asyncronus Receiveing Transmiting
}

