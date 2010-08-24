/*
	GPS_MTK.cpp - Ublox GPS library for Arduino
	Code by Jordi Muñoz and Jose Julio. DIYDrones.com
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
#include "AP_GPS_IMU.h"


// Constructors ////////////////////////////////////////////////////////////////
AP_GPS_IMU::AP_GPS_IMU()
{
}


// Public Methods //////////////////////////////////////////////////////////////
void AP_GPS_IMU::init(void)
{
	ck_a			= 0;
	ck_b			= 0;
	step			= 0;
	new_data		= 0;
	fix 			= 0;
	print_errors	= 0;

	// initialize serial port
	#if defined(__AVR_ATmega1280__)
		Serial1.begin(38400);				 // Serial port 1 on ATMega1280
	#else
		Serial.begin(38400);
	#endif
}

// optimization : This code doesn't wait for data. It only proccess the data available.
// We can call this function on the main loop (50Hz loop)
// If we get a complete packet this function calls parse_IMU_gps() to parse and update the GPS info.
void AP_GPS_IMU::update(void)
{
	byte data;
	int numc = 0;
	static byte message_num = 0;
	
	#if defined(__AVR_ATmega1280__)		// If AtMega1280 then Serial port 1...
		numc = Serial.available();
	#else
		numc = Serial.available();
	#endif
	
	if (numc > 0){
		for (int i=0;i<numc;i++){	// Process bytes received		
	
			#if defined(__AVR_ATmega1280__)
				data = Serial.read();
			#else
				data = Serial.read();
			#endif
			
			switch(step){		 //Normally we start from zero. This is a state machine
				case 0:	
					if(data == 0x44)	// IMU sync char 1
						step++;	 //OH first data packet is correct, so jump to the next step
					break; 
					
				case 1:	
					if(data == 0x49)	// IMU sync char 2
						step++;	 //ooh! The second data packet is correct, jump to the step 2
					else 
						step=0;	 //Nop, is not correct so restart to step zero and try again.		 
					break;
	
				case 2:	
					if(data == 0x59)	// IMU sync char 3
						step++;	 //ooh! The second data packet is correct, jump to the step 2
					else 
						step=0;	 //Nop, is not correct so restart to step zero and try again.		 
					break;
	
				case 3:	
					if(data == 0x64)	// IMU sync char 4
						step++;	 //ooh! The second data packet is correct, jump to the step 2
					else 
						step=0;	 //Nop, is not correct so restart to step zero and try again.		 
					break;
										
				case 4:
					payload_length = data;
					checksum(payload_length);
					step++;
					if (payload_length > 28){
						step = 0;	 //Bad data, so restart to step zero and try again.		 
						payload_counter = 0;
						ck_a = 0;
						ck_b = 0;
						//payload_error_count++;
					} 
					break;
					
				case 5:
					message_num = data;
					checksum(data);
					step++;
					break;
	
				case 6:	// Payload data read...
					// We stay in this state until we reach the payload_length
					buffer[payload_counter] = data;
					checksum(data);
					payload_counter++;
					if (payload_counter >= payload_length) { 
						step++; 
					}
					break;
					
				case 7:
					GPS_ck_a = data;	 // First checksum byte
					step++;
					break;
										
				case 8:
					GPS_ck_b = data;	 // Second checksum byte
	
					// We end the IMU/GPS read...
					// Verify the received checksum with the generated checksum.. 
					if((ck_a == GPS_ck_a) && (ck_b == GPS_ck_b)) {
						if (message_num == 0x02) {
							join_data();
						} else if (message_num == 0x03) {
							GPS_join_data();
						} else if (message_num == 0x04) {
							join_data_xplane();
						} else if (message_num == 0x0a) {
							//PERF_join_data();
						} else {
							Serial.print("Invalid message number = ");
							Serial.println(message_num, DEC);
						}
					} else {
						Serial.println("XXX Checksum error");	//bad checksum
						//imu_checksum_error_count++;
					} 						 
					// Variable initialization
					step = 0;
					payload_counter = 0;
					ck_a = 0;
					ck_b = 0;
					break;
			}
		}// End for...
	}
}

/****************************************************************
 * 
 ****************************************************************/

void AP_GPS_IMU::join_data(void)
{
	int j = 0;
	//Verifing if we are in class 1, you can change this "IF" for a "Switch" in case you want to use other IMU classes.. 
	//In this case all the message im using are in class 1, to know more about classes check PAGE 60 of DataSheet.

	 //Storing IMU roll
	intUnion.byte[0] = buffer[j++];
	intUnion.byte[1] = buffer[j++];
	roll_sensor = intUnion.word;

	 //Storing IMU pitch
	intUnion.byte[0] = buffer[j++];
	intUnion.byte[1] = buffer[j++];
	pitch_sensor = intUnion.word;

	 //Storing IMU heading (yaw)
	intUnion.byte[0] = buffer[j++];
	intUnion.byte[1] = buffer[j++];
	ground_course = intUnion.word;
	imu_ok = true;
}

void AP_GPS_IMU::join_data_xplane()
{
	int j = 0;

	 //Storing IMU roll
	intUnion.byte[0] = buffer[j++];
	intUnion.byte[1] = buffer[j++];
	roll_sensor = intUnion.word;

	 //Storing IMU pitch
	intUnion.byte[0] = buffer[j++];
	intUnion.byte[1] = buffer[j++];
	pitch_sensor = intUnion.word;

	 //Storing IMU heading (yaw)
	intUnion.byte[0] = buffer[j++];
	intUnion.byte[1] = buffer[j++];
	ground_course = (unsigned int)intUnion.word;
	
	 //Storing airspeed
	intUnion.byte[0] = buffer[j++];
	intUnion.byte[1] = buffer[j++];
	airspeed = intUnion.word;

	imu_ok = true;

}

void AP_GPS_IMU::GPS_join_data(void)
{
	//gps_messages_received++;
	int j = 0;				 

	longitude = join_4_bytes(&buffer[j]);		// Lat and Lon * 10**7
	j += 4;

	lattitude = join_4_bytes(&buffer[j]);
	j += 4;

	//Storing GPS Height above the sea level
	intUnion.byte[0] = buffer[j++];
	intUnion.byte[1] = buffer[j++];
	altitude = (long)intUnion.word * 10;		 //	Altitude in meters * 100 

	//Storing Speed
	intUnion.byte[0] = buffer[j++];
	intUnion.byte[1] = buffer[j++];
	speed_3d = ground_speed = (float)intUnion.word;			// Speed in M/S * 100
	
	//We skip the gps ground course because we use yaw value from the IMU for ground course
	j += 2;
	time = join_4_bytes(&buffer[j]);		//	Time of Week in milliseconds

	j += 4;
	imu_health = buffer[j++];
	
	new_data 	= 1;
	fix 		= 1;
}


/****************************************************************
 * 
 ****************************************************************/
 // Join 4 bytes into a long
long AP_GPS_IMU::join_4_bytes(unsigned char Buffer[])
{
	longUnion.byte[0] = *Buffer;
	longUnion.byte[1] = *(Buffer + 1);
	longUnion.byte[2] = *(Buffer + 2);
	longUnion.byte[3] = *(Buffer + 3);
	return(longUnion.dword);
}


/****************************************************************
 * 
 ****************************************************************/
// checksum algorithm
void AP_GPS_IMU::checksum(byte data)
{
	ck_a += data;
	ck_b += ck_a; 
}
