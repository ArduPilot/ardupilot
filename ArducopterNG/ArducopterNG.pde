/*
 www.ArduCopter.com - www.DIYDrones.com
 Copyright (c) 2010.  All rights reserved.
 An Open Source Arduino based multicopter.
 
 File     : Arducopter.pde
 Version  : v1.0, Aug 27, 2010
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

// Just add this in now and edit later
  int i, j;
  float aux_float[3];

  pinMode(LED_Yellow,OUTPUT); //Yellow LED A  (PC1)
  pinMode(LED_Red,OUTPUT);    //Red LED B     (PC2)
  pinMode(LED_Green,OUTPUT);  //Green LED C   (PC0)

  pinMode(SW1_pin,INPUT);     //Switch SW1 (pin PG0)

  pinMode(RELE_pin,OUTPUT);   // Rele output
  digitalWrite(RELE_pin,LOW);
  
  APM_RC.Init();             // APM Radio initialization
  // RC channels Initialization (Quad motors)  
  APM_RC.OutputCh(0,MIN_THROTTLE);  // Motors stoped
  APM_RC.OutputCh(1,MIN_THROTTLE);
  APM_RC.OutputCh(2,MIN_THROTTLE);
  APM_RC.OutputCh(3,MIN_THROTTLE);

  //  delay(1000); // Wait until frame is not moving after initial power cord has connected
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

  // Safety measure for Channel mids
  if(roll_mid < 1400 || roll_mid > 1600) roll_mid = 1500;
  if(pitch_mid < 1400 || pitch_mid > 1600) pitch_mid = 1500;
  if(yaw_mid < 1400 || yaw_mid > 1600) yaw_mid = 1500;

  if (MAGNETOMETER == 1)
    APM_Compass.Init();  // I2C initialization

  DataFlash.StartWrite(1);   // Start a write session on page 1

  SerBeg(SerBau);                      // Initialize SerialXX.port, IsXBEE define declares which port
#ifndef CONFIGURATOR  
  SerPri("ArduCopter Quadcopter v");
  SerPriln(VER)
  SerPri("Serial ready on port: ");    // Printout greeting to selecter serial port
  SerPriln(SerPor);                    // Printout serial port name
#endif

  // Check if we enable the DataFlash log Read Mode (switch)
  // If we press switch 1 at startup we read the Dataflash eeprom
  while (digitalRead(SW1_pin)==0)
  {
    SerPriln("Entering Log Read Mode...");
    Log_Read(1,2000);
    delay(30000);
  }

  Read_adc_raw();
  delay(10);

  // Offset values for accels and gyros...
  AN_OFFSET[3] = acc_offset_x;
  AN_OFFSET[4] = acc_offset_y;
  AN_OFFSET[5] = acc_offset_z;
  aux_float[0] = gyro_offset_roll;
  aux_float[1] = gyro_offset_pitch;
  aux_float[2] = gyro_offset_yaw;

  j = 0;
  // Take the gyro offset values
  for(i=0;i<300;i++)
  {
    Read_adc_raw();
    for(int y=0; y<=2; y++)   // Read initial ADC values for gyro offset.
    {
      aux_float[y]=aux_float[y]*0.8 + AN[y]*0.2;
      //SerPri(AN[y]);
      //SerPri(",");
    }
    //SerPriln();
    Log_Write_Sensor(AN[0],AN[1],AN[2],AN[3],AN[4],AN[5],ch_throttle);
    delay(10);
    
    // Runnings lights effect to let user know that we are taking mesurements
    if(j == 0) {
      digitalWrite(LED_Green, HIGH);
      digitalWrite(LED_Yellow, LOW);
      digitalWrite(LED_Red, LOW);
    } 
    else if (j == 1) {
      digitalWrite(LED_Green, LOW);
      digitalWrite(LED_Yellow, HIGH);
      digitalWrite(LED_Red, LOW);
    } 
    else {
      digitalWrite(LED_Green, LOW);
      digitalWrite(LED_Yellow, LOW);
      digitalWrite(LED_Red, HIGH);
    }
    if((i % 5) == 0) j++;
    if(j >= 3) j = 0;
  }
  digitalWrite(LED_Green, LOW);
  digitalWrite(LED_Yellow, LOW);
  digitalWrite(LED_Red, LOW);

  for(int y=0; y<=2; y++)   
    AN_OFFSET[y]=aux_float[y];

  //  Neutro_yaw = APM_RC.InputCh(3); // Take yaw neutral radio value
#ifndef CONFIGURATOR
  for(i=0;i<6;i++)
  {
    SerPri("AN[]:");
    SerPriln(AN_OFFSET[i]);
  }
  SerPri("Yaw neutral value:");
  //  SerPriln(Neutro_yaw);
  SerPri(yaw_mid);
#endif
  delay(1000);

  DataFlash.StartWrite(1);   // Start a write session on page 1
  timer = millis();
  tlmTimer = millis();
  Read_adc_raw();        // Initialize ADC readings...
  delay(20);

#ifdef IsAM
  // Switch Left & Right lights on
  digitalWrite(RI_LED, HIGH);
  digitalWrite(LE_LED, HIGH); 
#endif

  motorArmed = 0;
  digitalWrite(LED_Green,HIGH);     // Ready to go...
  
}



/* ************************************************************ */
/* ************** MAIN PROGRAM - MAIN LOOP ******************** */
/* ************************************************************ */
void loop() {
  // We want this to execute at 500Hz if possible
  // -------------------------------------------
  if (millis()-fast_loopTimer > 5) {
    deltaMiliSeconds = millis() - fast_loopTimer;
    G_Dt = (float)deltaMiliSeconds / 1000.0f;
    fast_loopTimer = millis();
    mainLoop_count++;
    // Execute the fast loop
    // ---------------------
    // fast_loop();
    // - PWM Updates
    // - Stabilization
    // - Altitude correction
    if(FL_mode == 0) {        // Changed for variable
      gled_speed = 1200;
      Attitude_control_v3();
    }
    else {
      gled_speed = 400;
      Rate_control_v2();
      // Reset yaw, so if we change to stable mode we continue with the actual yaw direction
      command_rx_yaw = ToDeg(yaw);
    }
		
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

