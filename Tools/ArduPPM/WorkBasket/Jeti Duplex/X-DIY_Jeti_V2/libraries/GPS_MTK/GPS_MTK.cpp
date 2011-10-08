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
		Init() : GPS Initialization
		Read() : Call this funcion as often as you want to ensure you read the incomming gps data
		
	Properties:
		Lattitude : Lattitude * 10,000,000 (long value)
		Longitude : Longitude * 10,000,000 (long value)
		Altitude :  Altitude * 100 (meters) (long value)
		Ground_speed : Speed (m/s) * 100 (long value)
		Ground_course : Course (degrees) * 100 (long value)
		NewData : 1 when a new data is received.
		          You need to write a 0 to NewData when you read the data
		Fix : 0: GPS NO FIX or 2D FIX, 1: 3D FIX.
			
*/

#include "GPS_MTK.h"
#include <avr/interrupt.h>
#include "WProgram.h"


// Constructors ////////////////////////////////////////////////////////////////
GPS_MTK_Class::GPS_MTK_Class()
{
}


// Public Methods //////////////////////////////////////////////////////////////
void GPS_MTK_Class::Init(void)
{
	delay(200);
	ck_a=0;
	ck_b=0;
	UBX_step=0;
	NewData=0;
	Fix=0;
	PrintErrors=0;
	GPS_timer=millis();   //Restarting timer...
	// Initialize serial port
	#if defined(__AVR_ATmega1280__)
		Serial1.begin(38400);         // Serial port 1 on ATMega1280
	#else
		Serial.begin(38400);
	#endif
	Serial1.print("$PGCMD,16,0,0,0,0,0*6A\r\n");
	//Serial.println("sent config string");
}

// optimization : This code don´t wait for data, only proccess the data available
// We can call this function on the main loop (50Hz loop)
// If we get a complete packet this function calls parse_ubx_gps() to parse and update the GPS info.
void GPS_MTK_Class::Read(void)
{
  static unsigned long GPS_timer=0;
  byte data;
  int numc;
  
  #if defined(__AVR_ATmega1280__)    // If AtMega1280 then Serial port 1...
	numc = Serial1.available();
  #else
	numc = Serial.available();
  #endif
  if (numc > 0)
    for (int i=0;i<numc;i++)  // Process bytes received
      {
	  #if defined(__AVR_ATmega1280__)
        data = Serial1.read();
      #else
		data = Serial.read();
	  #endif
      switch(UBX_step)     //Normally we start from zero. This is a state machine
      {
      case 0:  
        if(data==0xB5)  // UBX sync char 1
          UBX_step++;   //OH first data packet is correct, so jump to the next step
        break; 
      case 1:  
        if(data==0x62)  // UBX sync char 2
          UBX_step++;   //ooh! The second data packet is correct, jump to the step 2
        else 
          UBX_step=0;   //Nop, is not correct so restart to step zero and try again.     
        break;
      case 2:
        UBX_class=data;
        ubx_checksum(UBX_class);
        UBX_step++;
        break;
      case 3:
        UBX_id=data;
        UBX_step=4;
        UBX_payload_length_hi=26;
		UBX_payload_length_lo=0;
		UBX_payload_counter=0;

        ubx_checksum(UBX_id);
        
        break;
      case 4:
	if (UBX_payload_counter < UBX_payload_length_hi)  // We stay in this state until we reach the payload_length
        {
          UBX_buffer[UBX_payload_counter] = data;
          ubx_checksum(data);
          UBX_payload_counter++;
          if (UBX_payload_counter==UBX_payload_length_hi)
            UBX_step++;
        }
        break;
      case 5:
        UBX_ck_a=data;   // First checksum byte
        UBX_step++;
        break;
      case 6:
        UBX_ck_b=data;   // Second checksum byte
       
	  // We end the GPS read...
        if((ck_a==UBX_ck_a)&&(ck_b==UBX_ck_b))   // Verify the received checksum with the generated checksum.. 
	  		parse_ubx_gps();               // Parse the new GPS packet
        else
		  {
		  if (PrintErrors)
			Serial.println("ERR:GPS_CHK!!");
		  }
        // Variable initialization
        UBX_step=0;
        ck_a=0;
        ck_b=0;
        GPS_timer=millis(); //Restarting timer...
        break;
	  }
    }    // End for...
  // If we don´t receive GPS packets in 2 seconds => Bad FIX state
  if ((millis() - GPS_timer)>2000)
    {
	Fix = 0;
	if (PrintErrors)
	  Serial.println("ERR:GPS_TIMEOUT!!");
    }
}

/****************************************************************
 * 
 ****************************************************************/
// Private Methods //////////////////////////////////////////////////////////////
void GPS_MTK_Class::parse_ubx_gps(void)
{
  int j;
//Verifing if we are in class 1, you can change this "IF" for a "Switch" in case you want to use other UBX classes.. 
//In this case all the message im using are in class 1, to know more about classes check PAGE 60 of DataSheet.
  if(UBX_class==0x01) 
  {
    switch(UBX_id)//Checking the UBX ID
    {


    case 0x05: //ID Custom
      j=0;
      Lattitude= join_4_bytes(&UBX_buffer[j]) * 10; // lon*10,000,000
      j+=4;
      Longitude = join_4_bytes(&UBX_buffer[j]) * 10; // lat*10000000
      j+=4;
      Altitude = join_4_bytes(&UBX_buffer[j]);  // MSL
      j+=4;
	  Ground_Speed = join_4_bytes(&UBX_buffer[j]);
      j+=4;
	  Ground_Course = join_4_bytes(&UBX_buffer[j]) / 10000; // Heading 2D
      j+=4;
	  NumSats=UBX_buffer[j];
	  j++;
	  Fix=UBX_buffer[j];
	  if (Fix==3)
		Fix = 1;
	  else
		Fix = 0;
      j++;
	  Time = join_4_bytes(&UBX_buffer[j]);
      NewData=1;
      break;

      }
    }   
}


/****************************************************************
 * 
 ****************************************************************/
 // Join 4 bytes into a long
long GPS_MTK_Class::join_4_bytes(unsigned char Buffer[])
{
  union long_union {
	int32_t dword;
	uint8_t  byte[4];
} longUnion;

  longUnion.byte[3] = *Buffer;
  longUnion.byte[2] = *(Buffer+1);
  longUnion.byte[1] = *(Buffer+2);
  longUnion.byte[0] = *(Buffer+3);
  return(longUnion.dword);
}

/****************************************************************
 * 
 ****************************************************************/
// checksum algorithm
void GPS_MTK_Class::ubx_checksum(byte ubx_data)
{
  ck_a+=ubx_data;
  ck_b+=ck_a; 
}

GPS_MTK_Class GPS;