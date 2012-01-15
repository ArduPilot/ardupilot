/*
	GPS_NMEA.cpp - Generic NMEA GPS library for Arduino
	Code by Jordi Muñoz and Jose Julio. DIYDrones.com
	This code works with boards based on ATMega168/328 and ATMega1280 (Serial port 1)

	This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

	GPS configuration : NMEA protocol
	Baud rate : 38400
	NMEA Sentences : 
		$GPGGA : Global Positioning System Fix Data
		$GPVTG : Ttack and Ground Speed
		
	Methods:
		Init() : GPS Initialization
		Read() : Call this funcion as often as you want to ensure you read the incomming gps data
		
	Properties:
		Lattitude : Lattitude * 10000000 (long value)
		Longitude : Longitude * 10000000 (long value)
		Altitude :  Altitude * 1000 (milimeters) (long value)
		Ground_speed : Speed (m/s) * 100 (long value)
		Ground_course : Course (degrees) * 100 (long value)
		Type : 2 (This indicate that we are using the Generic NMEA library)
		NewData : 1 when a new data is received.
		          You need to write a 0 to NewData when you read the data
		Fix : >=1: GPS FIX, 0: No Fix (normal logic)
		Quality : 0 = No Fix
		           1 = Bad (Num sats < 5)
				   2 = Poor
				   3 = Medium
				   4 = Good

   NOTE : This code has been tested on a Locosys 20031 GPS receiver (MTK chipset)
*/

#include "GPS_NMEA.h"

#include <avr/interrupt.h>
#include "WProgram.h"


// Constructors ////////////////////////////////////////////////////////////////
GPS_NMEA_Class::GPS_NMEA_Class()
{
}

// Public Methods //////////////////////////////////////////////////////////////
void GPS_NMEA_Class::Init(void)
{
	Type = 2;
	GPS_checksum_calc = false;
	bufferidx = 0;
	NewData=0;
	Fix=0;
	Quality=0;
	PrintErrors=0;	
	// Initialize serial port
	#if defined(__AVR_ATmega1280__)
		Serial1.begin(38400);         // Serial port 1 on ATMega1280
	#else
		Serial.begin(38400);
	#endif
}

// This code don´t wait for data, only proccess the data available on serial port
// We can call this function on the main loop (50Hz loop)
// If we get a complete packet this function call parse_nmea_gps() to parse and update the GPS info.
void GPS_NMEA_Class::Read(void)
{
  char c;
  int numc;
  int i;
 
  
  #if defined(__AVR_ATmega1280__)    // If AtMega1280 then Serial port 1...
	numc = Serial1.available();    
  #else
	numc = Serial.available();
  #endif
  if (numc > 0)
    for (i=0;i<numc;i++){
      #if defined(__AVR_ATmega1280__)    // If AtMega1280 then Serial port 1...
	  c = Serial1.read();   
      #else
	  c = Serial.read();
      #endif
      if (c == '$'){                      // NMEA Start
        bufferidx = 0;
        buffer[bufferidx++] = c;
        GPS_checksum = 0;
        GPS_checksum_calc = true;
        continue;
        }
      if (c == '\r'){                     // NMEA End
        buffer[bufferidx++] = 0;
		parse_nmea_gps();
        }
      else {
        if (bufferidx < (GPS_BUFFERSIZE-1)){
          if (c == '*')
            GPS_checksum_calc = false;    // Checksum calculation end
          buffer[bufferidx++] = c;
          if (GPS_checksum_calc)
            GPS_checksum ^= c;            // XOR 
          }
		else
		  bufferidx=0;   // Buffer overflow : restart
        }
    }   
}

/****************************************************************
 * 
 ****************************************************************/
// Private Methods //////////////////////////////////////////////////////////////
void GPS_NMEA_Class::parse_nmea_gps(void)
{
  byte NMEA_check;
  long aux_deg;
  long aux_min;
  char *parseptr;

  
  if (strncmp(buffer,"$GPGGA",6)==0){        // Check if sentence begins with $GPGGA
    if (buffer[bufferidx-4]=='*'){           // Check for the "*" character
      NMEA_check = parseHex(buffer[bufferidx-3])*16 + parseHex(buffer[bufferidx-2]);    // Read the checksums characters
      if (GPS_checksum == NMEA_check){      // Checksum validation
        //Serial.println("buffer");
		NewData = 1;  // New GPS Data
        parseptr = strchr(buffer, ',')+1;
        //parseptr = strchr(parseptr, ',')+1;
		Time = parsenumber(parseptr,2);          // GPS UTC time hhmmss.ss
		parseptr = strchr(parseptr, ',')+1;
		//
        aux_deg = parsedecimal(parseptr,2);      // degrees
        aux_min = parsenumber(parseptr+2,4);     // minutes (sexagesimal) => Convert to decimal
        Lattitude = aux_deg*10000000 + (aux_min*50)/3;   // degrees + minutes/0.6  (*10000000) (0.6 = 3/5)
        parseptr = strchr(parseptr, ',')+1;
		//
		if (*parseptr=='S')
		  Lattitude = -1*Lattitude;              // South Lattitudes are negative
		//
        parseptr = strchr(parseptr, ',')+1;
        // W Longitudes are Negative
        aux_deg = parsedecimal(parseptr,3);      // degrees
        aux_min = parsenumber(parseptr+3,4);     // minutes (sexagesimal)
        Longitude = aux_deg*10000000 + (aux_min*50)/3;  // degrees + minutes/0.6 (*10000000)
        //Longitude = -1*Longitude;                   // This Assumes that we are in W longitudes...
        parseptr = strchr(parseptr, ',')+1;
		//
		if (*parseptr=='W')
		  Longitude = -1*Longitude;              // West Longitudes are negative
		//
        parseptr = strchr(parseptr, ',')+1;
        Fix = parsedecimal(parseptr,1);
        parseptr = strchr(parseptr, ',')+1;
        NumSats = parsedecimal(parseptr,2);
        parseptr = strchr(parseptr, ',')+1; 
        HDOP = parsenumber(parseptr,1);          // HDOP * 10
        parseptr = strchr(parseptr, ',')+1;
        Altitude = parsenumber(parseptr,1)*100;  // Altitude in decimeters*100 = milimeters
		if (Fix < 1)
		  Quality = 0;      // No FIX
		else if(NumSats<5)
		  Quality = 1;      // Bad (Num sats < 5)
		else if(HDOP>30)
		  Quality = 2;      // Poor (HDOP > 30)
		else if(HDOP>25)
		  Quality = 3;      // Medium (HDOP > 25)
		else
		  Quality = 4;      // Good (HDOP < 25)
        }
	  else
	    {
		if (PrintErrors)
	      Serial.println("GPSERR: Checksum error!!");
	    }
      }
    }
  else if (strncmp(buffer,"$GPVTG",6)==0){        // Check if sentence begins with $GPVTG
    //Serial.println(buffer);
    if (buffer[bufferidx-4]=='*'){           // Check for the "*" character
      NMEA_check = parseHex(buffer[bufferidx-3])*16 + parseHex(buffer[bufferidx-2]);    // Read the checksums characters
      if (GPS_checksum == NMEA_check){      // Checksum validation
        parseptr = strchr(buffer, ',')+1;
        Ground_Course = parsenumber(parseptr,2);      // Ground course in degrees * 100
        parseptr = strchr(parseptr, ',')+1;
        parseptr = strchr(parseptr, ',')+1;
        parseptr = strchr(parseptr, ',')+1;
        parseptr = strchr(parseptr, ',')+1;
        parseptr = strchr(parseptr, ',')+1;
        parseptr = strchr(parseptr, ',')+1;
        Ground_Speed = parsenumber(parseptr,2)*10/36; // Convert Km/h to m/s (*100)
        //GPS_line = true;
        }
	  else
	    {
		if (PrintErrors)
	      Serial.println("GPSERR: Checksum error!!");
	    }
    }
  }
  else
    {
	bufferidx = 0;
	if (PrintErrors)
	  Serial.println("GPSERR: Bad sentence!!");
    }
}


/****************************************************************
 * 
 ****************************************************************/
 // Parse hexadecimal numbers
byte GPS_NMEA_Class::parseHex(char c) {
    if (c < '0')
      return (0);
    if (c <= '9')
      return (c - '0');
    if (c < 'A')
       return (0);
    if (c <= 'F')
       return ((c - 'A')+10);
}

// Decimal number parser
long GPS_NMEA_Class::parsedecimal(char *str,byte num_car) {
  long d = 0;
  byte i;
  
  i = num_car;
  while ((str[0] != 0)&&(i>0)) {
     if ((str[0] > '9') || (str[0] < '0'))
       return d;
     d *= 10;
     d += str[0] - '0';
     str++;
     i--;
     }
  return d;
}

// Function to parse fixed point numbers (numdec=number of decimals)
long GPS_NMEA_Class::parsenumber(char *str,byte numdec) {
  long d = 0;
  byte ndec = 0;
  
  while (str[0] != 0) {
     if (str[0] == '.'){
       ndec = 1;
     }
     else {
       if ((str[0] > '9') || (str[0] < '0'))
         return d;
       d *= 10;
       d += str[0] - '0';
       if (ndec > 0)
         ndec++;
       if (ndec > numdec)   // we reach the number of decimals...
         return d;
       }
     str++;
     }
  return d;
}

GPS_NMEA_Class GPS; 