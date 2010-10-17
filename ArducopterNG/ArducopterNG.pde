/*
 www.ArduCopter.com - www.DIYDrones.com
 Copyright (c) 2010.  All rights reserved.
 An Open Source Arduino based multicopter.
 
 File     : ArducopterNG.pde
 Version  : v1.0, 11 October 2010
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
/*   GPS_MTK or GPS_UBLOX or GPS_NMEA : GPS library    [optional]         */
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

#define CONFIGURATOR

// Serial data, do we have FTDI cable or Xbee on Telemetry port as our primary command link
// If we are using normal FTDI/USB port as our telemetry/configuration, comment out next line
//#define SerXbee

//#define Ser0          // FTDI/USB Port  Either one
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

// GPS library (Include only one library)
#include <GPS_MTK.h>		// ArduPilot MTK GPS Library
//#include <GPS_IMU.h>		// ArduPilot IMU/SIM GPS Library
//#include <GPS_UBLOX.h>	// ArduPilot Ublox GPS Library
//#include <GPS_NMEA.h> 	// ArduPilot NMEA GPS library

/* Software version */
#define VER 1.5    // Current software version (only numeric values)

/* ************************************************************ */
/* ************* MAIN PROGRAM - DECLARATIONS ****************** */
/* ************************************************************ */

byte flightMode;

unsigned long currentTime, previousTime;
unsigned long mainLoop = 0;
unsigned long mediumLoop = 0;
unsigned long slowLoop = 0;

/* ************************************************************ */
/* **************** MAIN PROGRAM - SETUP ********************** */
/* ************************************************************ */
void setup() {
  
  APM_Init();    // APM Hardware initialization (in System.pde)
  
  mainLoop = millis();              // Initialize timers
  mediumLoop = mainLoop;
  GPS_timer = mainLoop;
  motorArmed = 0;
  Read_adc_raw();                   // Initialize ADC readings...
  delay(10);
  digitalWrite(LED_Green,HIGH);     // Ready to go...  
}


/* ************************************************************ */
/* ************** MAIN PROGRAM - MAIN LOOP ******************** */
/* ************************************************************ */

// Sensor reading loop is inside APM_ADC and runs at 400Hz (based on Timer2 interrupt)

// * fast rate loop => Main loop => 200Hz
// read sensors
// IMU : update attitude
// motor control
// Asyncronous task : read transmitter
// * medium rate loop (60Hz)
// Asyncronous task : read GPS
// * slow rate loop (10Hz)
// magnetometer
// barometer (20Hz)
// external command/telemetry
// Battery monitor

void loop()
{
  //int aux;
  //int i;
  //float aux_float;

  currentTime = millis();
  
  // Main loop at 200Hz (IMU + control)
  if ((currentTime-mainLoop) > 5)    // 200Hz (every 5ms)
    {
    G_Dt = (currentTime-mainLoop)*0.001;   // Microseconds!!!
    mainLoop = currentTime;

    //IMU DCM Algorithm
    Read_adc_raw();       // Read sensors raw data
    Matrix_update(); 
    Normalize();          
    Drift_correction();
    Euler_angles();

    // Read radio values (if new data is available)
    if (APM_RC.GetState() == 1)   // New radio frame?
      read_radio();

    // Attitude control
    if(flightMode == STABLE_MODE) {    // STABLE Mode
      gled_speed = 1200;
      if (AP_mode == AP_NORMAL_MODE)    // Normal mode
        Attitude_control_v3(command_rx_roll,command_rx_pitch,command_rx_yaw);
      else                        // Automatic mode : GPS position hold mode
        Attitude_control_v3(command_rx_roll+command_gps_roll,command_rx_pitch+command_gps_pitch,command_rx_yaw);
      }
    else {                 // ACRO Mode
      gled_speed = 400;
      Rate_control_v2();
      // Reset yaw, so if we change to stable mode we continue with the actual yaw direction
      command_rx_yaw = ToDeg(yaw);
      }

    // Send output commands to motor ESCs...
    motor_output();
    
    // Autopilot mode functions
    if (AP_mode == AP_AUTOMATIC_MODE)
      {
      if (target_position)
        {
        if (GPS.NewData)     // New GPS info?
          {
          read_GPS_data();
          Position_control(target_lattitude,target_longitude);     // Call GPS position hold routine
          }
        }
      else   // First time we enter in GPS position hold we capture the target position as the actual position
        {
        if (GPS.Fix){   // We need a GPS Fix to capture the actual position...
          target_lattitude = GPS.Lattitude;
          target_longitude = GPS.Longitude;
          target_position=1;
          //target_sonar_altitude = sonar_value;
          target_baro_altitude = press_alt;
          Initial_Throttle = ch_throttle;
          Reset_I_terms_navigation();  // Reset I terms (in Navigation.pde)
          }
        command_gps_roll=0;
        command_gps_pitch=0;
        }
      }
    else
      target_position=0;
    }
    
    // Medium loop (about 60Hz) 
    if ((currentTime-mediumLoop)>=17){
      mediumLoop = currentTime;
      GPS.Read();     // Read GPS data 
      // Each of the six cases executes at 10Hz
      switch (medium_loopCounter){
        case 0:   // Magnetometer reading (10Hz)
          medium_loopCounter++;
          slowLoop++;
          #ifdef IsMAG
          if (MAGNETOMETER == 1) {
            APM_Compass.Read();     // Read magnetometer
            APM_Compass.Calculate(roll,pitch);  // Calculate heading
          }
          #endif
          break;
        case 1:  // Barometer reading (2x10Hz = 20Hz)
          medium_loopCounter++;
          #ifdef UseBMP
          if (APM_BMP085.Read()){
            read_baro();
            Baro_new_data = 1;
            }
          #endif
          break;
        case 2:  // Send serial telemetry (10Hz)
          medium_loopCounter++;
          #ifdef CONFIGURATOR
          sendSerialTelemetry();
          #endif
          break;
        case 3:  // Read serial telemetry (10Hz)
          medium_loopCounter++;
          #ifdef CONFIGURATOR
          readSerialCommand();
          #endif
          break;
        case 4:  // second Barometer reading (2x10Hz = 20Hz)
          medium_loopCounter++;
          #ifdef UseBMP
          if (APM_BMP085.Read()){
            read_baro();
            Baro_new_data = 1;
            }
          #endif
          break;
        case 5:  //  Battery monitor (10Hz)
          medium_loopCounter=0;
          #if BATTERY_EVENT == 1
          read_battery();         // Battery monitor
          #endif
	  break;	
      }
    }
}
 
