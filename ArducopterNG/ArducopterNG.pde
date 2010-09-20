/*
 www.ArduCopter.com - www.DIYDrones.com
 Copyright (c) 2010.  All rights reserved.
 An Open Source Arduino based multicopter.
 
 File     : Arducopter.pde
 Version  : v0.1, October 2010
 Author(s): ArduCopter Team
             Ted Carancho (AeroQuad), Jose Julio, Jordi Muñoz,
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
//#define UseAirspeed
//#define UseBMP
//#define BATTERY_EVENT 1   // (boolean) 0 = don't read battery, 1 = read battery voltage (only if you have it wired up!)

// Serial data, do we have FTDI cable or Xbee on Telemetry port as our primary command link
#define Ser0          // FTDI/USB Port  Either one
//#define Ser3          // Telemetry port

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
#include <Wire.h>               // I2C Communication library
#include <APM_BMP085.h> 	// ArduPilot Mega BMP085 Library 
#include <EEPROM.h>             // EEPROM 
#include "Arducopter.h"
#include "ArduUser.h"

// GPS
#include <GPS_MTK.h>		// ArduPilot MTK GPS Library
//#include <GPS_IMU.h>		// ArduPilot IMU/SIM GPS Library
//#include <GPS_UBLOX.h>	// ArduPilot Ublox GPS Library
//#include <GPS_NMEA.h> 	// ArduPilot NMEA GPS library

/* Software version */
#define VER 0.1    // Current software version (only numeric values)

/* ************************************************************ */
/* ************* MAIN PROGRAM - DECLARATIONS ****************** */
/* ************************************************************ */
#define ROLL 0
#define PITCH 1
#define YAW 2
#define XAXIS 0
#define YAXIS 1
#define ZAXIS 2

#define GYROZ 0
#define GYROX 1
#define GYROY 2
#define ACCELX 3
#define ACCELY 4
#define ACCELZ 5
#define LASTSENSOR 6
int rawADC[6];
int zeroADC[6];
int dataADC[6];
byte channel;
int sensorSign[6] = {1, 1, 1, 1, 1, 1}; // GYROZ, GYROX, GYROY, ACCELX, ACCELY, ACCELZ

#define STABLE 0
#define ACRO 1
byte flightMode;

unsigned long currentTime, previousTime, deltaTime;
unsigned long sensorLoop = 0;
unsigned long controlLoop = 0;
unsigned long radioLoop = 0;
unsigned long motorLoop = 0;

/* ************************************************************ */
/* **************** MAIN PROGRAM - SETUP ********************** */
/* ************************************************************ */
void setup() {
  APM_Init();
  //APM_Init_ADC();
  //APM_Init_Radio();
  //APM_Init_Serial();
  //APM_Init_xx

  pinMode(LED_Yellow,OUTPUT); //Yellow LED A  (PC1)
  pinMode(LED_Red,OUTPUT);    //Red LED B     (PC2)
  pinMode(LED_Green,OUTPUT);  //Green LED C   (PC0)
  pinMode(SW1_pin,INPUT);     //Switch SW1 (pin PG0)
  pinMode(RELE_pin,OUTPUT);   // Rele output
  digitalWrite(RELE_pin,LOW);
  
  // APM Radio initialization
  APM_RC.Init();
  // RC channels Initialization (Quad motors)  
  APM_RC.OutputCh(0,MIN_THROTTLE);  // Motors stoped
  APM_RC.OutputCh(1,MIN_THROTTLE);
  APM_RC.OutputCh(2,MIN_THROTTLE);
  APM_RC.OutputCh(3,MIN_THROTTLE);

  for(i = 0; i <= 50; i++) {
    digitalWrite(LED_Green, HIGH);
    digitalWrite(LED_Yellow, HIGH);
    digitalWrite(LED_Red, HIGH);
    delay(20);
    digitalWrite(LED_Green, LOW);
    digitalWrite(LED_Yellow, LOW);
    digitalWrite(LED_Red, LOW);
    delay(20);
  }

  APM_ADC.Init();            // APM ADC library initialization

  DataFlash.Init();          // DataFlash log initialization
  DataFlash.StartWrite(1);   // Start a write session on page 1
  // Check if we enable the DataFlash log Read Mode (switch)
  // If we press switch 1 at startup we read the Dataflash eeprom
  while (digitalRead(SW1_pin)==0) {
    SerPriln("Entering Log Read Mode...");
    Log_Read(1,2000);
    delay(30000);
  }

  #ifdef IsGPS  
    GPS.Init();                // GPS Initialization
    #ifdef IsNEWMTEK  
      delay(250);
      // DIY Drones MTEK GPS needs binary sentences activated if you upgraded to latest firmware.
      // If your GPS shows solid blue but LED C (Red) does not go on, your GPS is on NMEA mode
      Serial1.print("$PGCMD,16,0,0,0,0,0*6A\r\n"); 
    #endif
  #endif

  readUserConfig(); // Load user configurable items from EEPROM

  if (MAGNETOMETER == 1)
    APM_Compass.Init();  // I2C initialization

  calibrateSensors();
  
  previousTime = millis();
  motorArmed = 0;
  digitalWrite(LED_Green,HIGH);     // Ready to go...  
}



/* ************************************************************ */
/* ************** MAIN PROGRAM - MAIN LOOP ******************** */
/* ************************************************************ */

// fast rate
// read sensors
// update attitude
// motor control

// medium rate
// read transmitter
// magnetometer
// barometer

// slow rate
// external command/telemetry
// GPS


void loop() {
  currentTime = millis();
  deltaTime = currentTime - previousTime;
  G_Dt = deltaTime / 1000.0;
  previousTime = currentTime;
  
  // Read Sensors **************************************************
  if (currentTime > sensorLoop + 2) { // 500Hz (every 2ms)
    for (channel = GYROZ; channel < LASTSENSOR; channel++) {
      dataADC[channel] = readADC(channel); // defined in Sensors.pde
    }
    sensorLoop = currentTime;
  }
  
  // Update ArduCopter Control *************************************
  if (currentTime > controlLoop + 5) { // 200Hz (every 5ms)
    if(flightMode == STABLE) {        // Changed for variable
      gled_speed = 1200;
      Attitude_control_v3();
    }
    else {
      gled_speed = 400;
      Rate_control_v2();
      // Reset yaw, so if we change to stable mode we continue with the actual yaw direction
      command_rx_yaw = ToDeg(yaw);
    }
    controlLoop = currentTime;
  }
  
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

