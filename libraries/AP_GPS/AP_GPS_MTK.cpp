/*
	AP_GPS_MTK.cpp - Ublox GPS library for Arduino
	Code by Jordi MuÒoz and Jose Julio. DIYDrones.com
	This code works with boards based on ATMega168/328 and ATMega1280 (Serial port 1)

	This library is free software; you can redistribute it and/or
		modify it under the terms of the GNU Lesser General Public
		License as published by the Free Software Foundation; either
		version 2.1 of the License, or (at your option) any later version.

	GPS configuration : Costum protocol
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
		fix : 1: GPS NO fix, 2: 2D fix, 3: 3D fix.
			
*/
#include "AP_GPS_MTK.h"


// Constructors ////////////////////////////////////////////////////////////////
AP_GPS_MTK::AP_GPS_MTK()
{
}


// Public Methods //////////////////////////////////////////////////////////////
void AP_GPS_MTK::init(void)
{
	new_data = 0;
	ck_a = 0;
	ck_b = 0;
	step = 0;
	fix = 0;
	print_errors = 0;
	// initialize serial port
	#if defined(__AVR_ATmega1280__)
		Serial1.begin(38400);				 // Serial port 1 on ATMega1280
	#else
		Serial.begin(38400); 
	#endif
}

// optimization : This code don¥t wait for data, only proccess the data available
// We can call this function on the main loop (50Hz loop)
// If we get a complete packet this function calls parse_gps() to parse and update the GPS info.
void AP_GPS_MTK::update(void)
{
	byte data;
	int numc;
	
	#if defined(__AVR_ATmega1280__)		// If AtMega1280 then Serial port 1...
		numc = Serial1.available();
	#else
		numc = Serial.available();
	#endif
	if (numc > 0)
		for (int i=0;i<numc;i++){	// Process bytes received
		#if defined(__AVR_ATmega1280__)
			data = Serial1.read();
		#else
			data = Serial.read();
		#endif
		
		switch(step){
			case 0:	
				if(data==0xB5)	// UBX sync char 1
					step++;	 	//OH first data packet is correct, so jump to the next step
				break;
				
			case 1:	
				if(data==0x62)	// UBX sync char 2
					step++;	 	//ooh! The second data packet is correct, jump to the step 2
				else 
					step=0;	 	//Nop, is not correct so restart to step zero and try again.		 
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
					if (print_errors)
						Serial.println("ERR:GPS_CHK!!");
				}
				// Variable initialization
				step = 0;
				ck_a = 0;
				ck_b = 0;
				break;
		}// End switch
	}// End for
}


// Private Methods 
void 
AP_GPS_MTK::parse_gps(void)
{
	int j;
//Verifing if we are in class 1, you can change this "IF" for a "Switch" in case you want to use other UBX classes.. 
//In this case all the message im using are in class 1, to know more about classes check PAGE 60 of DataSheet.
	if(msg_class == 0x01) {
		switch(id){
			//Checking the UBX ID
			case 0x05: //ID Custom
				j = 0;
				lattitude= join_4_bytes(&buffer[j]); // lon*10000000
				j += 4;
				longitude = join_4_bytes(&buffer[j]); // lat*10000000
				j += 4;
				altitude = join_4_bytes(&buffer[j]);	// MSL
				j += 4;
				speed_3d = ground_speed = join_4_bytes(&buffer[j]);
				j += 4;
				ground_course = join_4_bytes(&buffer[j]);
				j += 4;
				num_sats = buffer[j];
				j++;
				fix = buffer[j];
				j++;
				time = join_4_bytes(&buffer[j]);
				new_data = 1;
				break;
		}
	}
}

 // Join 4 bytes into a long
long 
AP_GPS_MTK::join_4_bytes(unsigned char buffer[])
{
	longUnion.byte[3] = *buffer;
	longUnion.byte[2] = *(buffer + 1);
	longUnion.byte[1] = *(buffer + 2);
	longUnion.byte[0] = *(buffer + 3);
	return(longUnion.dword);
}

// checksum algorithm
void
AP_GPS_MTK::checksum(byte data)
{
	ck_a += data;
	ck_b += ck_a; 
}
