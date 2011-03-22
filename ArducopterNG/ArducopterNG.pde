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
/*   AP_ADC : External ADC library                                       */
/*   DataFlash : DataFlash log library                                    */
/*   APM_BMP085 : BMP085 barometer library                                */
/*   AP_Compass : HMC5843 compass library [optional]                     */
/*   GPS_MTK or GPS_UBLOX or GPS_NMEA : GPS library    [optional]         */
/* ********************************************************************** */

/* ************************************************************ */
/* **************** MAIN PROGRAM - MODULES ******************** */
/* ************************************************************ */

/* ************************************************************ */
// User MODULES
//
// Please check your modules settings for every new software downloads you have.
// Also check repository / ArduCopter wiki pages for ChangeLogs and software notes
//
// Comment out with // modules that you are not using
//
// Do check ArduUser.h settings file too !!
//
///////////////////////////////////////
//  Modules Config
// --------------------------

#define IsGPS       // Do we have a GPS connected
#define IsNEWMTEK   // Do we have MTEK with new firmware
#define IsMAG       // Do we have a Magnetometer connected, if have remember to activate it from Configurator
//#define IsAM        // Do we have motormount LED's. AM = Atraction Mode
//#define IsCAM       // Do we have camera stabilization in use, If you activate, check OUTPUT pins from ArduUser.h

//#define UseAirspeed  // Quads don't use AirSpeed... Legacy, jp 19-10-10
#define UseBMP         // Use pressure sensor
//#define BATTERY_EVENT 1   // (boolean) 0 = don't read battery, 1 = read battery voltage (only if you have it _wired_ up!)
//#define IsSONAR        // are we using a Sonar for altitude hold?  use this or "UseBMP" not both!
//#define IsRANGEFINDER  // are we using range finders for obstacle avoidance?

#define CONFIGURATOR

///////////////////////////////////////
// GPS Selection

#define GPSDEVICE GPSDEV_DIYMTEK    // For DIY Drones MediaTek
//#define GPSDEVICE  GPSDEV_DIYUBLOX   // For DIY Drones uBlox GPS
//#define GPSDEVICE  GPSDEV_FPUBLOX    // For Fah Pah Special ArduCopter GPS
//#define GPSDEVICE  GPSDEV_NMEA       // For general NMEA compatible GPSEs
//#dedine GPSDEVICE  GPSDEV_IMU        // For IMU Simulations only


////////////////////////////////////////
// Frame / Motor / ESC definitions

// Introducing new frame / Motor / ESC definitions for future expansion. Currently these are not in 
// use but they need to be here so implementation work can continue.

                             // New frame model definitions. (not in use yet, 28-11-10 jp)
#define FRAME_MODEL QUAD     // Quad frame model 
//#define FRAME_MODEL HEXA     // Quad frame model 
//#define FRAME_MODEL OCTO     // Quad frame model 


                             // New motor definition for different frame type (not in use yet, 28-11-10 jp)
#define MAX_MOTORS  4        // Are we using more motors than 4, possible choises are 4, 6, 8
                             // This has to be on main .pde to get included on all other header etc files

                             // Not in use yet, 28-11-10 jp
#define MOTORTYPE  PWM       // Traditional PWM ESC's controlling motors
//#define MOTORTYPE  I2C     // I2C style ESC's controlling motors
//#define MOTORTYPE UART     // UART style ESC's controlling motors




////////////////////
// Serial ports & speeds

// Serial data, do we have FTDI cable or Xbee on Telemetry port as our primary command link
// If we are using normal FTDI/USB port as our telemetry/configuration, keep next line disabled
//#define SerXbee

// Telemetry port speed, default is 115200
//#define SerBau  19200
//#define SerBau  38400
//#define SerBau  57600
#define SerBau  115200


// For future use, for now don't activate any!
// Serial1 speed for GPS, mostly 38.4k, done from libraries
//#define GpsBau  19200
//#define GpsBau  38400
//#define GpsBau  57600
//#define GpsBau  115200


/* ************************************************* */
// Radio modes
#define RADIOMODE  MODE2    // Most users have this eg: left stick: Throttle/Rudder, right stick: Elevator/Aileron
//#define RADIOMODE  MODE1  // Only if you are sure that you have Mode 1 radio. 

// NOTE! MODE1 is not working yet, we need to have input from users to be sure of channel orders.  03-11-10, jp



/* ************************************************* */
// Flight & Electronics orientation

// Frame build condiguration
//#define FLIGHT_MODE_+    // Traditional "one arm as nose" frame configuration
//#define FLIGHT_MODE_X  // Frame orientation 45 deg to CCW, nose between two arms
// 19-10-10 by JP

// This feature has been disabled for now, if you want to change between flight orientations
// just use DIP switch for that. DIP1 down = X, DIP1 up = +

// Magneto orientation and corrections.
// If you don't have magneto activated, It is safe to ignore these
//#ifdef IsMAG
//#define MAGORIENTATION  AP_COMPASS_COMPONENTS_UP_PINS_FORWARD       // This is default solution for ArduCopter
//#define MAGORIENTATION  AP_COMPASS_COMPONENTS_UP_PINS_BACK        // Alternative orientation for ArduCopter
#define MAGORIENTATION  AP_COMPASS_COMPONENTS_DOWN_PINS_FORWARD    // If you have soldered Magneto to IMU shield in WIki pictures shows

// To get Magneto offsets, switch to CLI mode and run offset calibration. During calibration
// you need to roll/bank/tilt/yaw/shake etc your ArduCopter. Don't kick like Jani always does :)
//#define MAGOFFSET 0,0,0
//#define MAGOFFSET -27.50,23.00,81.00

// Obsolete, Magnetometer offset are moved to CLI

// Declination is a correction factor between North Pole and real magnetic North. This is different on every location
// IF you want to use really accurate headholding and future navigation features, you should update this
// You can check Declination to your location from http://www.magnetic-declination.com/
#define DECLINATION 0.0

//#define DECLINATION 0.61

// And remember result from NOAA website is in form of DEGREES°MINUTES'. Degrees you can use directly but Minutes you need to 
// recalculate due they one degree is 60 minutes.. For example Jani's real declination is 0.61, correct way to calculate this is
// 37 / 60 = 0.61 and for Helsinki it would be 7°44' eg 7. and then 44/60 = 0.73 so declination for Helsinki/South Finland would be 7.73

// East values are positive
// West values are negative

// Some of devel team's Declinations and their Cities
//#define DECLINATION 0.61      // Jani, Bangkok, 0°37' E  (Due I live almost at Equator, my Declination is rather small)
//#define DECLINATION 7.73      // Jani, Helsinki,7°44' E  (My "summer" home back at Finland)
//#define DECLINATION -20.68    // Sandro, Belo Horizonte, 22°08' W  (Whoah... Sandro is really DECLINED)
//#define DECLINATION 7.03      // Randy, Tokyo, 7°02'E
//#define DECLINATION 8.91      // Doug, Denver, 8°55'E
//#define DECLINATION -6.08     // Jose, Canary Islands, 6°5'W
//#define DECLINATION 0.73      // Tony, Minneapolis, 0°44'E

//#endif





/* ************************************************************ */
/* **************** MAIN PROGRAM - INCLUDES ******************* */
/* ************************************************************ */

//#include <AP_GPS.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <math.h>
#include <APM_RC.h> 		// ArduPilot Mega RC Library
#include <AP_ADC.h>		// ArduPilot Mega Analog to Digital Converter Library 
#include <APM_BMP085.h> 	// ArduPilot Mega BMP085 Library 
#include <DataFlash.h>		// ArduPilot Mega Flash Memory Library
#include <AP_Compass.h>	        // ArduPilot Mega Magnetometer Library
#include <Wire.h>               // I2C Communication library
#include <EEPROM.h>             // EEPROM 
#include <AP_RangeFinder.h>     // RangeFinders (Sonars, IR Sensors)
//#include <AP_GPS.h>
#include "Arducopter.h"
#include "ArduUser.h"

#ifdef IsGPS
// GPS library (Include only one library)
#include <GPS_MTK.h>            // ArduPilot MTK GPS Library
//#include <GPS_IMU.h>            // ArduPilot IMU/SIM GPS Library
//#include <GPS_UBLOX.h>  // ArduPilot Ublox GPS Library
//#include <GPS_NMEA.h>   // ArduPilot NMEA GPS library
#endif

#if AIRFRAME == HELI
#include "Heli.h"
#endif

/* Software version */
#define VER 1.54    // Current software version (only numeric values)

// Sensors - declare one global instance
AP_ADC_ADS7844		adc;
APM_BMP085_Class	APM_BMP085;
AP_Compass_HMC5843	AP_Compass;
#ifdef IsSONAR
AP_RangeFinder_MaxsonarXL  AP_RangeFinder_down;  // Default sonar for altitude hold
//AP_RangeFinder_MaxsonarLV  AP_RangeFinder_down;  // Alternative sonar is AP_RangeFinder_MaxsonarLV
#endif
#ifdef IsRANGEFINDER
AP_RangeFinder_MaxsonarLV  AP_RangeFinder_frontRight;
AP_RangeFinder_MaxsonarLV  AP_RangeFinder_backRight;
AP_RangeFinder_MaxsonarLV  AP_RangeFinder_backLeft;
AP_RangeFinder_MaxsonarLV  AP_RangeFinder_frontLeft;
#endif

/* ************************************************************ */
/* ************* MAIN PROGRAM - DECLARATIONS ****************** */
/* ************************************************************ */

byte flightMode;

unsigned long currentTime;  // current time in milliseconds
unsigned long currentTimeMicros = 0, previousTimeMicros = 0;  // current and previous loop time in microseconds
unsigned long mainLoop = 0;
unsigned long mediumLoop = 0;
unsigned long slowLoop = 0;

/* ************************************************************ */
/* **************** MAIN PROGRAM - SETUP ********************** */
/* ************************************************************ */
void setup() {

  APM_Init();                // APM Hardware initialization (in System.pde)

  mainLoop = millis();       // Initialize timers
  mediumLoop = mainLoop;
  GPS_timer = mainLoop;
  motorArmed = 0;
  
  GEOG_CORRECTION_FACTOR = 0;   // Geographic correction factor will be automatically calculated

  Read_adc_raw();            // Initialize ADC readings...
  
#ifdef SerXbee
  Serial.begin(SerBau);
  Serial.print("ArduCopter v");
  Serial.println(VER);
  Serial.println("Serial data on Telemetry port");
  Serial.println("No commands or output on this serial, check your Arducopter.pde if needed to change.");
  Serial.println();
  Serial.println("General info:");
  if(!SW_DIP1) Serial.println("Flight mode: + ");
  if(SW_DIP1) Serial.println("Flight mode: x ");
#endif 


  delay(10);
  digitalWrite(LED_Green,HIGH);     // Ready to go...  
}


/* ************************************************************ */
/* ************** MAIN PROGRAM - MAIN LOOP ******************** */
/* ************************************************************ */

// Sensor reading loop is inside AP_ADC and runs at 400Hz (based on Timer2 interrupt)

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
// sonar (20Hz)
// obstacle avoidance (20Hz)
// external command/telemetry
// Battery monitor



/* ***************************************************** */
// Main loop 
void loop()
{
  
  currentTimeMicros = micros();
  currentTime = currentTimeMicros / 1000;

  // Main loop at 200Hz (IMU + control)
  if ((currentTime-mainLoop) > 5)    // about 200Hz (every 5ms)
  {
    G_Dt = (currentTimeMicros-previousTimeMicros) * 0.000001;   // Microseconds!!!
    mainLoop = currentTime;
    previousTimeMicros = currentTimeMicros;

    //IMU DCM Algorithm
    Read_adc_raw();       // Read sensors raw data
    Matrix_update(); 
    Normalize();          
    Drift_correction();
    Euler_angles();

    // Read radio values (if new data is available)
    if (APM_RC.GetState() == 1) {  // New radio frame?
#if AIRFRAME == QUAD    
      read_radio();
#endif
#if AIRFRAME == HELI
      heli_read_radio();
#endif
    }

    // Attitude control
    if(flightMode == STABLE_MODE) {    // STABLE Mode
      gled_speed = 1200;
      if (AP_mode == AP_NORMAL_MODE) {   // Normal mode
#if AIRFRAME == QUAD
        Attitude_control_v3(command_rx_roll,command_rx_pitch,command_rx_yaw);
#endif        
#if AIRFRAME == HELI
        heli_attitude_control(command_rx_roll,command_rx_pitch,command_rx_collective,command_rx_yaw);
#endif
      }else{                        // Automatic mode : GPS position hold mode
#if AIRFRAME == QUAD      
        Attitude_control_v3(command_rx_roll+command_gps_roll+command_RF_roll,command_rx_pitch+command_gps_pitch+command_RF_pitch,command_rx_yaw);
#endif      
#if AIRFRAME == HELI
        heli_attitude_control(command_rx_roll+command_gps_roll,command_rx_pitch+command_gps_pitch,command_rx_collective,command_rx_yaw);
#endif
      }
    }
    else {                 // ACRO Mode
      gled_speed = 400;
      Rate_control_v2();
      // Reset yaw, so if we change to stable mode we continue with the actual yaw direction
      command_rx_yaw = ToDeg(yaw);
    }

    // Send output commands to motor ESCs...
#if AIRFRAME == QUAD     // we update the heli swashplate at about 60hz
    motor_output();
#endif    

#ifdef IsCAM
  // Do we have cameras stabilization connected and in use?
  if(!SW_DIP2) camera_output();
#endif

    // Autopilot mode functions - GPS Hold, Altitude Hold + object avoidance
    if (AP_mode == AP_AUTOMATIC_MODE)
    {
      digitalWrite(LED_Yellow,HIGH);      // Yellow LED ON : GPS Position Hold MODE
      
      // Do GPS Position hold (lattitude & longitude)
      if (target_position) 
      {
        #ifdef IsGPS
        if (GPS.NewData)     // New GPS info?
        {
          if (GPS.Fix)
          {
            read_GPS_data();    // In Navigation.pde
            //Position_control(target_lattitude,target_longitude);     // Call GPS position hold routine
            Position_control_v2(target_lattitude,target_longitude);     // V2 of GPS Position holdCall GPS position hold routine
          }else{
            command_gps_roll=0;
            command_gps_pitch=0;
          }
        }
        #endif
      } else {  // First time we enter in GPS position hold we capture the target position as the actual position
        #ifdef IsGPS
        if (GPS.Fix){   // We need a GPS Fix to capture the actual position...
          target_lattitude = GPS.Lattitude;
          target_longitude = GPS.Longitude;
          target_position=1;
        }
        #endif
        command_gps_roll=0;
        command_gps_pitch=0;
        Reset_I_terms_navigation();  // Reset I terms (in Navigation.pde)
      }
      
      // Switch on altitude control if we have a barometer or Sonar
      #if (defined(UseBMP) || defined(IsSONAR))
      if( altitude_control_method == ALTITUDE_CONTROL_NONE ) 
      {
          // by default turn on altitude hold using barometer
          #ifdef UseBMP
          if( press_baro_altitude != 0 ) 
          {
            altitude_control_method = ALTITUDE_CONTROL_BARO;  
            target_baro_altitude = press_baro_altitude;  
            baro_altitude_I = 0;  // don't carry over any I values from previous times user may have switched on altitude control
          }
          #endif
          
          // use sonar if it's available
          #ifdef IsSONAR
          if( sonar_status == SONAR_STATUS_OK && press_sonar_altitude != 0 ) 
          {
            altitude_control_method = ALTITUDE_CONTROL_SONAR;
            target_sonar_altitude = constrain(press_sonar_altitude,AP_RangeFinder_down.min_distance*3,sonar_threshold);
          }
          sonar_altitude_I = 0;  // don't carry over any I values from previous times user may have switched on altitude control          
          #endif
          
          // capture current throttle to use as base for altitude control
          initial_throttle = ch_throttle;
          ch_throttle_altitude_hold = ch_throttle;
      }
 
      // Sonar Altitude Control
      #ifdef IsSONAR 
      if( sonar_new_data ) // if new sonar data has arrived
      {
        // Allow switching between sonar and barometer
        #ifdef UseBMP
        
        // if SONAR become invalid switch to barometer
        if( altitude_control_method == ALTITUDE_CONTROL_SONAR && sonar_valid_count <= -3  )
        {
          // next target barometer altitude to current barometer altitude + user's desired change over last sonar altitude (i.e. keeps up the momentum)
          altitude_control_method = ALTITUDE_CONTROL_BARO;
          target_baro_altitude = press_baro_altitude;// + constrain((target_sonar_altitude - press_sonar_altitude),-50,50);
        }
   
        // if SONAR becomes valid switch to sonar control
        if( altitude_control_method == ALTITUDE_CONTROL_BARO && sonar_valid_count >= 3  )
        {
          altitude_control_method = ALTITUDE_CONTROL_SONAR;
          if( target_sonar_altitude == 0 ) {  // if target sonar altitude hasn't been intialised before..
            target_sonar_altitude = press_sonar_altitude;// + constrain((target_baro_altitude - press_baro_altitude),-50,50);  // maybe this should just use the user's last valid target sonar altitude         
          }
          // ensure target altitude is reasonable
          target_sonar_altitude = constrain(target_sonar_altitude,AP_RangeFinder_down.min_distance*3,sonar_threshold);
        }      
        #endif  // defined(UseBMP)
      
        // main Sonar control
        if( altitude_control_method == ALTITUDE_CONTROL_SONAR )
        {
          if( sonar_status == SONAR_STATUS_OK ) {
            ch_throttle_altitude_hold = Altitude_control_Sonar(press_sonar_altitude,target_sonar_altitude);  // calculate throttle to maintain altitude
          } else {
            // if sonar_altitude becomes invalid we return control to user temporarily
            ch_throttle_altitude_hold = ch_throttle;
          }            
  
          // modify the target altitude if user moves stick more than 100 up or down
          if( abs(ch_throttle-initial_throttle)>100 ) 
          {
            target_sonar_altitude += (ch_throttle-initial_throttle)/25;
            if( target_sonar_altitude < AP_RangeFinder_down.min_distance*3 )
                target_sonar_altitude = AP_RangeFinder_down.min_distance*3;
            #if !defined(UseBMP)   // limit the upper altitude if no barometer used
            if( target_sonar_altitude > sonar_threshold)
                target_sonar_altitude = sonar_threshold;
            #endif
          }
        }
        sonar_new_data = 0;  // record that we've consumed the sonar data
      }  // new sonar data
      #endif // #ifdef IsSONAR
      
      // Barometer Altitude control
      #ifdef UseBMP
      if( baro_new_data && altitude_control_method == ALTITUDE_CONTROL_BARO )   // New altitude data?
      {
        ch_throttle_altitude_hold = Altitude_control_baro(press_baro_altitude,target_baro_altitude);   // calculate throttle to maintain altitude
        baro_new_data=0;  // record that we have consumed the new data
        
        // modify the target altitude if user moves stick more than 100 up or down
        if (abs(ch_throttle-initial_throttle)>100) 
        {
          target_baro_altitude += (ch_throttle-initial_throttle)/25;  // Change in stick position => altitude ascend/descend rate control
        }
      }
      #endif
      #endif // defined(UseBMP) || defined(IsSONAR)
      
      // Object avoidance
      #ifdef IsRANGEFINDER
      if( RF_new_data ) 
      {
        Obstacle_avoidance(RF_SAFETY_ZONE);  // main obstacle avoidance function
        RF_new_data = 0;  // record that we have consumed the rangefinder data
      }
      #endif
    }else{
      digitalWrite(LED_Yellow,LOW);
      target_position=0;
      if( altitude_control_method != ALTITUDE_CONTROL_NONE )
      {
        altitude_control_method = ALTITUDE_CONTROL_NONE;  // turn off altitude control        
      }
    } // if (AP_mode == AP_AUTOMATIC_MODE)
  }  

  // Medium loop (about 60Hz) 
  if ((currentTime-mediumLoop)>=17){
    mediumLoop = currentTime;
#ifdef IsGPS
    GPS.Read();     // Read GPS data 
#endif

#if AIRFRAME == HELI    
    // Send output commands to heli swashplate...
    heli_moveSwashPlate();
#endif

    // Each of the six cases executes at 10Hz
    switch (medium_loopCounter){
    case 0:   // Magnetometer reading (10Hz)
      medium_loopCounter++;
      slowLoop++;
#ifdef IsMAG
      if (MAGNETOMETER == 1) {
        AP_Compass.read();     // Read magnetometer
        AP_Compass.calculate(roll,pitch);  // Calculate heading
      }
#endif
      break;
    case 1:  // Barometer + RangeFinder reading (2x10Hz = 20Hz)
      medium_loopCounter++;
#ifdef UseBMP
      if (APM_BMP085.Read()){
        read_baro();
        baro_new_data = 1;
      }
#endif
#ifdef IsSONAR
      read_Sonar(); 
      sonar_new_data = 1;  // process sonar values at 20Hz     
#endif   
#ifdef IsRANGEFINDER
      read_RF_Sensors();
      RF_new_data = 1;      
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
    case 4:  // second Barometer + RangeFinder reading (2x10Hz = 20Hz)
      medium_loopCounter++;
#ifdef UseBMP
      if (APM_BMP085.Read()){
        read_baro();
        baro_new_data = 1;
      }
#endif
#ifdef IsSONAR
      read_Sonar(); 
      sonar_new_data = 1;  // process sonar values at 20Hz     
#endif 
#ifdef IsRANGEFINDER
      read_RF_Sensors();
      RF_new_data = 1;
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

  // AM and Mode status LED lights
  if(millis() - gled_timer > gled_speed) {
    gled_timer = millis();
    if(gled_status == HIGH) { 
      digitalWrite(LED_Green, LOW);
#ifdef IsAM      
      digitalWrite(RE_LED, LOW);
#endif
      gled_status = LOW;
//      SerPrln("L");
    } 
    else {
      digitalWrite(LED_Green, HIGH);
#ifdef IsAM
      if(motorArmed) digitalWrite(RE_LED, HIGH);
#endif
      gled_status = HIGH;
    } 
  }

}


