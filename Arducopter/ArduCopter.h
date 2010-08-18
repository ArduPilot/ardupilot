/*
  ArduCopter 1.3 - Aug 2010
  www.ArduCopter.com
  Copyright (c) 2010. All rights reserved.
  An Open Source Arduino based multicopter.
 
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
*/

#include "WProgram.h"
#include "UserConfig.h"

/*******************************************************************/
// ArduPilot Mega specific hardware and software settings
// 
// DO NOT EDIT unless you are absolytely sure of your doings. 
// User configurable settings are on UserConfig.h
/*******************************************************************/


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

// Sensor: GYROX, GYROY, GYROZ, ACCELX, ACCELY, ACCELZ
uint8_t sensors[6] = {1, 2, 0, 4, 5, 6};  // For ArduPilot Mega Sensor Shield Hardware

// Sensor: GYROX, GYROY, GYROZ,   ACCELX, ACCELY, ACCELZ,     MAGX, MAGY, MAGZ
int SENSOR_SIGN[]={
  1, -1, -1,         // GYROX, GYROY, GYROZ
 -1, 1, 1,           // ACCELX, ACCELY, ACCELZ
 -1, -1, -1};        // MAGNETOX, MAGNETOY, MAGNETOZ
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


float G_Dt=0.02;    // Integration time for the gyros (DCM algorithm)
float Accel_Vector[3]= {0, 0, 0}; //Store the acceleration in a vector
float Accel_Vector_unfiltered[3]= {0, 0, 0}; //Store the acceleration in a vector
float Gyro_Vector[3]= {0, 0, 0};//Store the gyros rutn rate in a vector
float Omega_Vector[3]= {0, 0, 0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0, 0, 0};//Omega Proportional correction
float Omega_I[3]= {0, 0, 0};//Omega Integrator
float Omega[3]= {0, 0, 0};
//float Accel_magnitude;
//float Accel_weight;

float errorRollPitch[3]= {0, 0, 0};
float errorYaw[3]= {0, 0, 0};
float errorCourse=0;
float COGX=0; //Course overground X axis
float COGY=1; //Course overground Y axis

float roll=0;
float pitch=0;
float yaw=0;

unsigned int counter=0;

float DCM_Matrix[3][3]= {
  {
    1,0,0  }
  ,{
    0,1,0  }
  ,{
    0,0,1  }
}; 
float Update_Matrix[3][3]={
  {
    0,1,2  }
  ,{
    3,4,5  }
  ,{
    6,7,8  }
}; //Gyros here

float Temporary_Matrix[3][3]={
  {
    0,0,0  }
  ,{
    0,0,0  }
  ,{
    0,0,0  }
};

// GPS variables
float speed_3d=0;
int GPS_ground_speed=0;

long timer=0; //general porpuse timer 
long timer_old;

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
long target_longitude;
long target_lattitude;
byte target_position;
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



/*****************************************************/
// APM Specific Memory variables

// Following variables stored in EEPROM
float KP_QUAD_ROLL;
float KD_QUAD_ROLL;
float KI_QUAD_ROLL;
float KP_QUAD_PITCH;
float KD_QUAD_PITCH;
float KI_QUAD_PITCH;
float KP_QUAD_YAW;
float KD_QUAD_YAW;
float KI_QUAD_YAW;
float STABLE_MODE_KP_RATE;
float KP_GPS_ROLL;
float KD_GPS_ROLL;
float KI_GPS_ROLL;
float KP_GPS_PITCH;
float KD_GPS_PITCH;
float KI_GPS_PITCH;
float GPS_MAX_ANGLE;
float KP_ALTITUDE;
float KD_ALTITUDE;
float KI_ALTITUDE;
int acc_offset_x;
int acc_offset_y;
int acc_offset_z;
int gyro_offset_roll;
int gyro_offset_pitch;
int gyro_offset_yaw;
float Kp_ROLLPITCH;
float Ki_ROLLPITCH;
float Kp_YAW;
float Ki_YAW;
float GEOG_CORRECTION_FACTOR;
int MAGNETOMETER;
float Kp_RateRoll;
float Ki_RateRoll;
float Kd_RateRoll;
float Kp_RatePitch;
float Ki_RatePitch;
float Kd_RatePitch;
float Kp_RateYaw;
float Ki_RateYaw;
float Kd_RateYaw;
float xmitFactor;

// EEPROM storage addresses
#define KP_QUAD_ROLL_ADR 0
#define KD_QUAD_ROLL_ADR 4
#define KI_QUAD_ROLL_ADR 8
#define KP_QUAD_PITCH_ADR 12
#define KD_QUAD_PITCH_ADR 16
#define KI_QUAD_PITCH_ADR 20
#define KP_QUAD_YAW_ADR 24
#define KD_QUAD_YAW_ADR 28
#define KI_QUAD_YAW_ADR 32
#define STABLE_MODE_KP_RATE_ADR 36
#define KP_GPS_ROLL_ADR 40
#define KD_GPS_ROLL_ADR 44
#define KI_GPS_ROLL_ADR 48
#define KP_GPS_PITCH_ADR 52
#define KD_GPS_PITCH_ADR 56
#define KI_GPS_PITCH_ADR 60
#define GPS_MAX_ANGLE_ADR 64
#define KP_ALTITUDE_ADR 68
#define KD_ALTITUDE_ADR 72
#define KI_ALTITUDE_ADR 76
#define acc_offset_x_ADR 80
#define acc_offset_y_ADR 84
#define acc_offset_z_ADR 88
#define gyro_offset_roll_ADR 92
#define gyro_offset_pitch_ADR 96
#define gyro_offset_yaw_ADR 100
#define Kp_ROLLPITCH_ADR 104
#define Ki_ROLLPITCH_ADR 108
#define Kp_YAW_ADR 112
#define Ki_YAW_ADR 116
#define GEOG_CORRECTION_FACTOR_ADR 120
#define MAGNETOMETER_ADR 124
#define XMITFACTOR_ADR 128
#define KP_RATEROLL_ADR 132
#define KI_RATEROLL_ADR 136
#define KD_RATEROLL_ADR 140
#define KP_RATEPITCH_ADR 144
#define KI_RATEPITCH_ADR 148
#define KD_RATEPITCH_ADR 152
#define KP_RATEYAW_ADR 156
#define KI_RATEYAW_ADR 160
#define KD_RATEYAW_ADR 164
#define CHROLL_MID 168
#define CHPITCH_MID 172
#define CHYAW_MID 176

// Utilities for writing and reading from the EEPROM
float readEEPROM(int address) {
  union floatStore {
    byte floatByte[4];
    float floatVal;
  } floatOut;
  
  for (int i = 0; i < 4; i++) 
    floatOut.floatByte[i] = EEPROM.read(address + i);
  return floatOut.floatVal;
}

void writeEEPROM(float value, int address) {
  union floatStore {
    byte floatByte[4];
    float floatVal;
  } floatIn;
  
  floatIn.floatVal = value;
  for (int i = 0; i < 4; i++) 
    EEPROM.write(address + i, floatIn.floatByte[i]);
}

void readUserConfig() {
  KP_QUAD_ROLL = readEEPROM(KP_QUAD_ROLL_ADR);
  KD_QUAD_ROLL = readEEPROM(KD_QUAD_ROLL_ADR);
  KI_QUAD_ROLL = readEEPROM(KI_QUAD_ROLL_ADR);
  KP_QUAD_PITCH = readEEPROM(KP_QUAD_PITCH_ADR);
  KD_QUAD_PITCH = readEEPROM(KD_QUAD_PITCH_ADR);
  KI_QUAD_PITCH = readEEPROM(KI_QUAD_PITCH_ADR);
  KP_QUAD_YAW = readEEPROM(KP_QUAD_YAW_ADR);
  KD_QUAD_YAW = readEEPROM(KD_QUAD_YAW_ADR);
  KI_QUAD_YAW = readEEPROM(KI_QUAD_YAW_ADR);
  STABLE_MODE_KP_RATE = readEEPROM(STABLE_MODE_KP_RATE_ADR);
  KP_GPS_ROLL = readEEPROM(KP_GPS_ROLL_ADR);
  KD_GPS_ROLL = readEEPROM(KD_GPS_ROLL_ADR);
  KI_GPS_ROLL = readEEPROM(KI_GPS_ROLL_ADR);
  KP_GPS_PITCH = readEEPROM(KP_GPS_PITCH_ADR);
  KD_GPS_PITCH = readEEPROM(KD_GPS_PITCH_ADR);
  KI_GPS_PITCH = readEEPROM(KI_GPS_PITCH_ADR);
  GPS_MAX_ANGLE = readEEPROM(GPS_MAX_ANGLE_ADR);
  KP_ALTITUDE = readEEPROM(KP_ALTITUDE_ADR);
  KD_ALTITUDE = readEEPROM(KD_ALTITUDE_ADR);
  KI_ALTITUDE = readEEPROM(KI_ALTITUDE_ADR);
  acc_offset_x = readEEPROM(acc_offset_x_ADR);
  acc_offset_y = readEEPROM(acc_offset_y_ADR);
  acc_offset_z = readEEPROM(acc_offset_z_ADR);
  gyro_offset_roll = readEEPROM(gyro_offset_roll_ADR);
  gyro_offset_pitch = readEEPROM(gyro_offset_pitch_ADR);
  gyro_offset_yaw = readEEPROM(gyro_offset_yaw_ADR);
  Kp_ROLLPITCH = readEEPROM(Kp_ROLLPITCH_ADR);
  Ki_ROLLPITCH = readEEPROM(Ki_ROLLPITCH_ADR);
  Kp_YAW = readEEPROM(Kp_YAW_ADR);
  Ki_YAW = readEEPROM(Ki_YAW_ADR);
  GEOG_CORRECTION_FACTOR = readEEPROM(GEOG_CORRECTION_FACTOR_ADR);
  MAGNETOMETER = readEEPROM(MAGNETOMETER_ADR);
  Kp_RateRoll = readEEPROM(KP_RATEROLL_ADR);
  Ki_RateRoll = readEEPROM(KI_RATEROLL_ADR);
  Kd_RateRoll = readEEPROM(KD_RATEROLL_ADR);
  Kp_RatePitch = readEEPROM(KP_RATEPITCH_ADR);
  Ki_RatePitch = readEEPROM(KI_RATEPITCH_ADR);
  Kd_RatePitch = readEEPROM(KD_RATEPITCH_ADR);
  Kp_RateYaw = readEEPROM(KP_RATEYAW_ADR);
  Ki_RateYaw = readEEPROM(KI_RATEYAW_ADR);
  Kd_RateYaw = readEEPROM(KD_RATEYAW_ADR);
  xmitFactor = readEEPROM(XMITFACTOR_ADR);
  roll_mid = readEEPROM(CHROLL_MID);
  pitch_mid = readEEPROM(CHPITCH_MID);
  yaw_mid = readEEPROM(CHYAW_MID);

}
