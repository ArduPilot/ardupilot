/*
 www.ArduCopter.com - www.DIYDrones.com
 Copyright (c) 2010.  All rights reserved.
 An Open Source Arduino based multicopter.
 
 File     : Arducopter.pde
 Version  : v1.0, Aug 27, 2010
 Author(s): ArduCopter Team
             Ted Carancho (aeroquad), Jose Julio, Jordi Muñoz,
             Jani Hirvinen, Ken McEwans, Roberto Navoni,          
             Sandro Benigno, Chris Anderson
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program. If not, see <http://www.gnu.org/licenses/>.

/* ********************************************************************** */
/* Hardware : ArduPilot Mega + Sensor Shield (Production versions)        */
/* Mounting position : RC connectors pointing backwards                   */
/* This code use this libraries :                                         */
/*   APM_RC : Radio library (with InstantPWM)                             */
/*   APM_ADC : External ADC library                                       */
/*   DataFlash : DataFlash log library                                    */
/*   APM_BMP085 : BMP085 barometer library                                */
/*   APM_Compass : HMC5843 compass library [optional]                     */
/*   GPS_UBLOX or GPS_NMEA or GPS_MTK : GPS library    [optional]         */
/* ********************************************************************** */

/* ********************************************************************** *
ChangeLog:


* *********************************************************************** *
TODO:


* *********************************************************************** */


/* ************************************************************ */
/* **************** MAIN PROGRAM - MODULES ******************** */
/* ************************************************************ */

/* User definable modules */

// Comment out with   // modules that you are not using

//#define IsGPS       // Do we have a GPS connected
//#define IsNEWMTEK   // Do we have MTEK with new firmware
//#define IsMAG         // Do we have a Magnetometer connected, if have remember to activate it from Configurator
//#define IsTEL       // Do we have a telemetry connected, eg. XBee connected on Telemetry port
//#define IsAM          // Do we have motormount LED's. AM = Atraction Mode


// Serial data, do we have FTDI cable or Xbee on Telemetry port as our primary command link
#define Ser0          // FTDI/USB Port  Either one
//#define Ser3          // Telemetry port

//#define CONFIGURATOR  // Do se use Configurator or normal text output over serial link


/**********************************************/
// Not in use yet, starting to work with battery monitors and pressure sensors. 
// Added 19-08-2010

//#define UseAirspeed
//#define UseBMP
//#define BATTERY_EVENT 1   // (boolean) 0 = don't read battery, 1 = read battery voltage (only if you have it wired up!)

/**********************************************/


// Frame build condiguration
#define FLIGHT_MODE_+    // Traditional "one arm as nose" frame configuration
//#define FLIGHT_MODE_X  // Frame orientation 45 deg to CCW, nose between two arms





/* ************************************************************ */
/* **************** MAIN PROGRAM - INCLUDES ******************* */
/* ************************************************************ */

#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <math.h>
#include <APM_RC.h> 		// ArduPilot Mega RC Library
#include <APM_ADC.h>		// ArduPilot Mega Analog to Digital Converter Library 
#include <APM_BMP085.h> 	// ArduPilot Mega BMP085 Library 
#include <DataFlash.h>		// ArduPilot Mega Flash Memory Library
#include <APM_Compass.h>	// ArduPilot Mega Magnetometer Library
//#include "defines.h"
//#include "console.h"
#include <Wire.h>               // I2C Communication library

#ifdef UseBMP
#include <APM_BMP085.h> 	// ArduPilot Mega BMP085 Library 
#endif

//#include <GPS_IMU.h>		// ArduPilot IMU/SIM GPS Library
#include <GPS_MTK.h>		// ArduPilot MTK GPS Library
//#include <GPS_UBLOX.h>	// ArduPilot Ublox GPS Library
//#include <GPS_NMEA.h> 	// ArduPilot NMEA GPS library

// EEPROM storage for user configurable values
#include <EEPROM.h>             // EEPROM 
#include "Arducopter.h"
#include "ArduUser.h"

/* Software version */
#define VER 1.40    // Current software version (only numeric values)




/* ************************************************************ */
/* ************* MAIN PROGRAM - DECLARATIONS ****************** */
/* ************************************************************ */





/* ************************************************************ */
/* **************** MAIN PROGRAM - SETUP ********************** */
/* ************************************************************ */
void setup() {

  //APM_Init_IO(); 
  APM_Init();

  //APM_Init_ADC();
  //APM_Init_Radio();
  //APM_Init_Serial();
  //APM_Init_xx

  
  
}



/* ************************************************************ */
/* ************** MAIN PROGRAM - MAIN LOOP ******************** */
/* ************************************************************ */
void loop() {
	// We want this to execute at 500Hz if possible
	// -------------------------------------------
	if (millis()-fast_loopTimer > 5) {
		deltaMiliSeconds 	= millis() - fast_loopTimer;
		G_Dt 			= (float)deltaMiliSeconds / 1000.f;
		fast_loopTimer		= millis();
		mainLoop_count++;

		// Execute the fast loop
		// ---------------------
		// fast_loop();
                // - PWM Updates
                // - Stabilization
                // - Altitude correction
		
		// Execute the medium loop 
		// -----------------------
		// medium_loop();
                // - Radio read
                // - GPS read
                // - Drift correction

		// Execute the slow loop 
		// -----------------------
		// slow_loop();
                // - Battery usage
                // - GCS updates
                // - Garbage management

		if (millis()- perf_mon_timer > 20000) {
			if (mainLoop_count != 0) {
	
				//send_message(MSG_PERF_REPORT);
				#if LOG_PM
					Log_Write_Performance();
				#endif
				resetPerfData();
			 }
		}
	}  
}

