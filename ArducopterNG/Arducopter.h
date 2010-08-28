/*
 www.ArduCopter.com - www.DIYDrones.com
 Copyright (c) 2010.  All rights reserved.
 An Open Source Arduino based multicopter.
 
 File     : Arducopter.h
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

* ************************************************************** *
ChangeLog:


* ************************************************************** *
TODO:


* ************************************************************** */

#include "WProgram.h"


/* APM Hardware definitions */
#define LED_Yellow 36
#define LED_Red 35
#define LED_Green 37
#define RELE_pin 47
#define SW1_pin 41
#define SW2_pin 40

//#define VDIV1 AN1
//#define VDIV2 AN2
//#define VDIV3 AN3
//#define VDIV4 AN4

//#define AN5
//#define AN6

// Hardware Parameters
#define SLIDE_SWITCH_PIN 40
#define PUSHBUTTON_PIN 41

#define A_LED_PIN 37			//36 = B,	37 = A,	35 = C
#define B_LED_PIN 36
#define C_LED_PIN 35


#define EE_LAST_LOG_PAGE 0xE00
#define EE_LAST_LOG_NUM 0xE02
#define EE_LOG_1_START 0xE04

// Serial ports
#define SERIAL0_BAUD 38400		// this is the main USB out 38400 57600 115200
#define SERIAL1_BAUD 115200
#define SERIAL2_BAUD 115200
#define SERIAL3_BAUD 115200


#ifdef Ser3
#define SerPr Serial3.print
#define SerPrln Serial3.println
#define SerRe Serial3.read
#define SerAv Serial3.available
#endif


#ifndef SerPr
#define Ser10
#endif
#ifdef Ser10
#define SerPr Serial.print
#define SerPrln Serial.println
#define SerRe Serial.read
#define SerAv Serial.available
#endif


	
								
/****************************************************/
/*Logging Stuff	- These should be 1 (on) or 0 (off)*/
/****************************************************/	
#define LOG_ATTITUDE 1	        // Logs basic attitude info
#define LOG_GPS 1		// Logs GPS info
#define LOG_PM 1		// Logs IMU performance monitoring info£
#define LOG_CTUN 0		// Logs control loop tuning info
#define LOG_NTUN 0		// Logs navigation loop tuning info
#define LOG_MODE 1		// Logs mode changes
#define LOG_RAW 0		// Logs raw accel/gyro data
#define LOG_SEN 1               // Logs sensor data

//  GCS Message ID's
#define MSG_ACKNOWLEDGE 0x00
#define MSG_HEARTBEAT 0x01
#define MSG_ATTITUDE 0x02
#define MSG_LOCATION 0x03
#define MSG_PRESSURE 0x04
#define MSG_STATUS_TEXT 0x05
#define MSG_PERF_REPORT 0x06
#define MSG_COMMAND 0x22
#define MSG_VALUE 0x32
#define MSG_PID 0x42
#define MSG_TRIMS 0x50
#define MSG_MINS 0x51
#define MSG_MAXS 0x52
#define MSG_IMU_OUT 0x53

#define SEVERITY_LOW 1
#define SEVERITY_MEDIUM 2
#define SEVERITY_HIGH 3
#define SEVERITY_CRITICAL 4

// Debug options - set only one of these options to 1 at a time, set the others to 0
#define DEBUG_SUBSYSTEM 0 		// 0 = no debug
								// 1 = Debug the Radio input
								// 2 = Debug the Servo output
								// 3 = Debug the Sensor input
								// 4 = Debug the GPS input
								// 5 = Debug the GPS input - RAW HEX OUTPUT
								// 6 = Debug the IMU
								// 7 = Debug the Control Switch
								// 8 = Debug the Servo DIP switches
								// 9 = Debug the Relay out
								// 10 = Debug the Magnetometer
								// 11 = Debug the ABS pressure sensor
								// 12 = Debug the stored waypoints
								// 13 = Debug the Throttle
								// 14 = Debug the Radio Min Max
								// 15 = Debug the EEPROM - Hex Dump
								

#define DEBUG_LEVEL SEVERITY_LOW
								// SEVERITY_LOW
								// SEVERITY_MEDIUM
								// SEVERITY_HIGH
								// SEVERITY_CRITICAL


// Sensor: GYROX, GYROY, GYROZ, ACCELX, ACCELY, ACCELZ
uint8_t sensors[6] = {1, 2, 0, 4, 5, 6};  // For ArduPilot Mega Sensor Shield Hardware

// Sensor: GYROX, GYROY, GYROZ,   ACCELX, ACCELY, ACCELZ,     MAGX, MAGY, MAGZ
int SENSOR_SIGN[]={
  1, -1, 1,    -1, 1, 1,     -1, -1, -1}; 
 //{-1,1,-1,1,-1,1,-1,-1,-1};
/* APM Hardware definitions, END */

/* General definitions */

#define TRUE 1
#define FALSE 0
#define ON 1
#define OFF 0


// ADC : Voltage reference 3.3v / 12bits(4096 steps) => 0.8mV/ADC step
// ADXL335 Sensitivity(from datasheet) => 330mV/g, 0.8mV/ADC step => 330/0.8 = 412
// Tested value : 408
#define GRAVITY 408 //this equivalent to 1G in the raw data coming from the accelerometer 
#define Accel_Scale(x) x*(GRAVITY/9.81)//Scaling the raw data of the accel to actual acceleration in meters for seconds square

#define ToRad(x) (x*0.01745329252)  // *pi/180
#define ToDeg(x) (x*57.2957795131)  // *180/pi

// IDG500 Sensitivity (from datasheet) => 2.0mV/º/s, 0.8mV/ADC step => 0.8/3.33 = 0.4
// Tested values : 
#define Gyro_Gain_X 0.4  //X axis Gyro gain
#define Gyro_Gain_Y 0.41 //Y axis Gyro gain
#define Gyro_Gain_Z 0.41 //Z axis Gyro gain
#define Gyro_Scaled_X(x) x*ToRad(Gyro_Gain_X) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) x*ToRad(Gyro_Gain_Y) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) x*ToRad(Gyro_Gain_Z) //Return the scaled ADC raw data of the gyro in radians for second

/*For debugging purposes*/
#define OUTPUTMODE 1  //If value = 1 will print the corrected data, 0 will print uncorrected data of the gyros (with drift), 2 Accel only data

int AN[6]; //array that store the 6 ADC channels
int AN_OFFSET[6]; //Array that store the Offset of the gyros and accelerometers
int gyro_temp;


float G_Dt=0.02;                  // Integration time for the gyros (DCM algorithm)
float Accel_Vector[3]= {0, 0, 0}; //Store the acceleration in a vector
float Accel_Vector_unfiltered[3]= {0, 0, 0}; //Store the acceleration in a vector
float Gyro_Vector[3]= {0, 0, 0};  //Store the gyros rutn rate in a vector
float Omega_Vector[3]= {0, 0, 0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0, 0, 0};      //Omega Proportional correction
float Omega_I[3]= {0, 0, 0};      //Omega Integrator
float Omega[3]= {0, 0, 0};
//float Accel_magnitude;
//float Accel_weight;

float errorRollPitch[3] = {0, 0, 0};
float errorYaw[3] = {0, 0, 0};
float errorCourse = 0;
float COGX = 0; //Course overground X axis
float COGY = 1; //Course overground Y axis

float roll = 0;
float pitch = 0;
float yaw = 0;

unsigned int counter = 0;

float DCM_Matrix[3][3]= {
  { 1,0,0 },
  { 0,1,0 },
  { 0,0,1 }}; 
float Update_Matrix[3][3]={
  { 0,1,2 },
  { 3,4,5 },
  { 6,7,8 }}; //Gyros here

float Temporary_Matrix[3][3]={
  { 0,0,0 },
  { 0,0,0 },
  { 0,0,0 }};

// GPS variables
float speed_3d=0;
int GPS_ground_speed=0;

// Main timers
long timer=0; 
long timer_old;
long GPS_timer;
long GPS_timer_old;
float GPS_Dt=0.2;   // GPS Dt

// Attitude control variables
float command_rx_roll=0;        // User commands
float command_rx_roll_old;
float command_rx_roll_diff;
float command_rx_pitch=0;
float command_rx_pitch_old;
float command_rx_pitch_diff;
float command_rx_yaw=0;
float command_rx_yaw_diff;
int control_roll;           // PID control results
int control_pitch;
int control_yaw;
float K_aux;

// Attitude PID controls
float roll_I=0;
float roll_D;
float err_roll;
float pitch_I=0;
float pitch_D;
float err_pitch;
float yaw_I=0;
float yaw_D;
float err_yaw;

//Position control
#ifdef IsGPS
long target_longitude;
long target_lattitude;
byte target_position;
#endif
float gps_err_roll;
float gps_err_roll_old;
float gps_roll_D;
float gps_roll_I=0;
float gps_err_pitch;
float gps_err_pitch_old;
float gps_pitch_D;
float gps_pitch_I=0;
float command_gps_roll;
float command_gps_pitch;

//Altitude control
int Initial_Throttle;
int target_sonar_altitude;
int err_altitude;
int err_altitude_old;
float command_altitude;
float altitude_I;
float altitude_D;

//Pressure Sensor variables
#ifdef UseBMP
unsigned long abs_press 	        = 0;    
unsigned long abs_press_filt            = 0;
unsigned long abs_press_gnd             = 0;
int 	ground_temperature	        = 0;    // 
int 	temp_unfilt			= 0;
long 	ground_alt			= 0;	// Ground altitude from gps at startup in centimeters
long 	press_alt			= 0;	// Pressure altitude 
#endif


#define BATTERY_VOLTAGE(x) (x*(INPUT_VOLTAGE/1024.0))*VOLT_DIV_RATIO

#define AIRSPEED_PIN 1		// Need to correct value
#define BATTERY_PIN 1		// Need to correct value
#define RELAY_PIN 47
#define LOW_VOLTAGE	11.4    // Pack voltage at which to trigger alarm
#define INPUT_VOLTAGE 5.2	// (Volts) voltage your power regulator is feeding your ArduPilot to have an accurate pressure and battery level readings. (you need a multimeter to measure and set this of course)
#define VOLT_DIV_RATIO 1.0	//  Voltage divider ratio set with thru-hole resistor (see manual)

float 	battery_voltage 	= LOW_VOLTAGE * 1.05;		// Battery Voltage, initialized above threshold for filter


// Sonar variables
int Sonar_value=0;
#define SonarToCm(x) (x*1.26)   // Sonar raw value to centimeters
int Sonar_Counter=0;

// AP_mode : 1=> Position hold  2=>Stabilization assist mode (normal mode)
byte AP_mode = 2;  

// Mode LED timers and variables, used to blink LED_Green
byte gled_status = HIGH;
long gled_timer;
int gled_speed;

long t0;
int num_iter;
float aux_debug;

// Radio definitions
int roll_mid;
int pitch_mid;
int yaw_mid;

int Neutro_yaw;
int ch_roll;
int ch_pitch;
int ch_throttle;
int ch_yaw;
int ch_aux;
int ch_aux2;

int frontMotor;
int backMotor;
int leftMotor;
int rightMotor;
byte motorArmed = 0;
int minThrottle = 0;

// Serial communication
char queryType;
long tlmTimer = 0;

// Arming/Disarming
uint8_t Arming_counter=0;
uint8_t Disarming_counter=0;

// Performance monitoring
// ----------------------
long 	perf_mon_timer 				= 0;
float 	imu_health 					= 0;		//Metric based on accel gain deweighting
int 	G_Dt_max 					= 0;		//Max main loop cycle time in milliseconds
byte 	gyro_sat_count 				= 0;
byte 	adc_constraints 			= 0;
byte 	renorm_sqrt_count 			= 0;
byte 	renorm_blowup_count 		        = 0;
int 	gps_fix_count				= 0;

byte	gcs_messages_sent			= 0;

// System Timers
// --------------
unsigned long fast_loopTimer		= 0;		// Time in miliseconds of main control loop
unsigned long medium_loopTimer		= 0;		// Time in miliseconds of navigation control loop
byte medium_loopCounter				= 0;		// Counters for branching from main control loop to slower loops
byte slow_loopCounter				= 0;		// 
unsigned long deltaMiliSeconds 		= 0;		// Delta Time in miliseconds
unsigned long dTnav					= 0;		// Delta Time in milliseconds for navigation computations
int mainLoop_count 					= 0;
unsigned long elapsedTime			= 0;		// for doing custom events
//unsigned int GPS_timer				= 0;


