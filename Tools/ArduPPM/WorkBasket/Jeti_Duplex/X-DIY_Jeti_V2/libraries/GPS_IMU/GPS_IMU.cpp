/*
	GPS_IMU.cpp - IMU/X-Plane GPS library for Arduino
*/

#include "GPS_IMU.h"
#include <avr/interrupt.h>
#include "WProgram.h"


// Constructors ////////////////////////////////////////////////////////////////
GPS_IMU_Class::GPS_IMU_Class()
{
}


// Public Methods //////////////////////////////////////////////////////////////
void GPS_IMU_Class::Init(void)
{
	ck_a		= 0;
	ck_b		= 0;
	IMU_step	= 0;
	NewData		= 0;
	Fix 		= 0;
	PrintErrors	= 0;
	
	IMU_timer = millis();	 //Restarting timer...
	// Initialize serial port
	#if defined(__AVR_ATmega1280__)
		Serial1.begin(38400);				 // Serial port 1 on ATMega1280
	#else
		Serial.begin(38400);
	#endif
}

// optimization : This code don´t wait for data, only proccess the data available
// We can call this function on the main loop (50Hz loop)
// If we get a complete packet this function calls parse_IMU_gps() to parse and update the GPS info.
void GPS_IMU_Class::Read(void)
{
	static unsigned long GPS_timer = 0;
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
			
			switch(IMU_step){		 //Normally we start from zero. This is a state machine
				case 0:	
					if(data == 0x44)	// IMU sync char 1
						IMU_step++;	 //OH first data packet is correct, so jump to the next IMU_step
					break; 
					
				case 1:	
					if(data == 0x49)	// IMU sync char 2
						IMU_step++;	 //ooh! The second data packet is correct, jump to the IMU_step 2
					else 
						IMU_step=0;	 //Nop, is not correct so restart to IMU_step zero and try again.		 
					break;
	
				case 2:	
					if(data == 0x59)	// IMU sync char 3
						IMU_step++;	 //ooh! The second data packet is correct, jump to the IMU_step 2
					else 
						IMU_step=0;	 //Nop, is not correct so restart to IMU_step zero and try again.		 
					break;
	
				case 3:	
					if(data == 0x64)	// IMU sync char 4
						IMU_step++;	 //ooh! The second data packet is correct, jump to the IMU_step 2
					else 
						IMU_step=0;	 //Nop, is not correct so restart to IMU_step zero and try again.		 
					break;
										
				case 4:
					payload_length = data;
					checksum(payload_length);
					IMU_step++;
					if (payload_length > 28){
						IMU_step = 0;	 //Bad data, so restart to IMU_step zero and try again.		 
						payload_counter = 0;
						ck_a = 0;
						ck_b = 0;
						//payload_error_count++;
					} 
					break;
					
				case 5:
					message_num = data;
					checksum(data);
					IMU_step++;
					break;
	
				case 6:	// Payload data read...
					// We stay in this state until we reach the payload_length
					buffer[payload_counter] = data;
					checksum(data);
					payload_counter++;
					if (payload_counter >= payload_length) { 
						IMU_step++; 
					}
					break;
					
				case 7:
					IMU_ck_a = data;	 // First checksum byte
					IMU_step++;
					break;
										
				case 8:
					IMU_ck_b = data;	 // Second checksum byte
	
					// We end the IMU/GPS read...
					// Verify the received checksum with the generated checksum.. 
					if((ck_a == IMU_ck_a) && (ck_b == IMU_ck_b)) {
						if (message_num == 0x02) {
							IMU_join_data();
							IMU_timer = millis();
						} else if (message_num == 0x03) {
							GPS_join_data();
							GPS_timer = millis();
						} else if (message_num == 0x04) {
							IMU2_join_data();
							IMU_timer = millis();
						} else if (message_num == 0x0a) {
							//PERF_join_data();
						} else {
							Serial.print("Invalid message number = ");
							Serial.println(message_num,DEC);
						}
					} else {
						Serial.println("XXX Checksum error");	//bad checksum
						//imu_checksum_error_count++;
					} 						 
					// Variable initialization
					IMU_step = 0;
					payload_counter = 0;
					ck_a = 0;
					ck_b = 0;
					IMU_timer = millis(); //Restarting timer...
					break;
			}
		}// End for...
	}
	// If we don't receive GPS packets in 2 seconds => Bad FIX state
	if ((millis() - GPS_timer) > 3000){
		Fix = 0;
	}
	if (PrintErrors){
		Serial.println("ERR:GPS_TIMEOUT!!");
	}
}

/****************************************************************
 * 
 ****************************************************************/

void GPS_IMU_Class::IMU_join_data(void)
{
	int j;

	
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
	Ground_Course = intUnion.word;
	imu_ok = true;
}

void GPS_IMU_Class::IMU2_join_data()
{
	int j=0;

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
	Ground_Course = (unsigned int)intUnion.word;
	
	 //Storing airspeed
	intUnion.byte[0] = buffer[j++];
	intUnion.byte[1] = buffer[j++];
	airspeed = intUnion.word;

	imu_ok = true;

}

void GPS_IMU_Class::GPS_join_data(void)
{
	//gps_messages_received++;
	int j = 0;				 

	Longitude = join_4_bytes(&buffer[j]);		// Lat and Lon * 10**7
	j += 4;

	Lattitude = join_4_bytes(&buffer[j]);
	j += 4;

	//Storing GPS Height above the sea level
	intUnion.byte[0] = buffer[j++];
	intUnion.byte[1] = buffer[j++];
	Altitude = (long)intUnion.word * 10;		 //	Altitude in meters * 100 

	//Storing Speed (3-D) 
	intUnion.byte[0] = buffer[j++];
	intUnion.byte[1] = buffer[j++];
	Speed_3d = Ground_Speed = (float)intUnion.word;			// Speed in M/S * 100
	
	//We skip the gps ground course because we use yaw value from the IMU for ground course
	j += 2;
	Time = join_4_bytes(&buffer[j]);		//	Time of Week in milliseconds
    j +=4;
    imu_health = buffer[j++];
	
	NewData = 1;
	Fix = 1;

}


/****************************************************************
 * 
 ****************************************************************/
 // Join 4 bytes into a long
long GPS_IMU_Class::join_4_bytes(unsigned char Buffer[])
{
	longUnion.byte[0] = *Buffer;
	longUnion.byte[1] = *(Buffer+1);
	longUnion.byte[2] = *(Buffer+2);
	longUnion.byte[3] = *(Buffer+3);
	return(longUnion.dword);
}


/****************************************************************
 * 
 ****************************************************************/
// checksum algorithm
void GPS_IMU_Class::checksum(byte IMU_data)
{
	ck_a+=IMU_data;
	ck_b+=ck_a; 
}

GPS_IMU_Class GPS;
