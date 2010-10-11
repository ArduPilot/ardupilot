/*
 www.ArduCopter.com - www.DIYDrones.com
 Copyright (c) 2010.  All rights reserved.
 An Open Source Arduino based multicopter.
 
 File     : UserDefines.pde
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


/*************************************************************/
// Safety & Security 

// Arm & Disarm delays
#define ARM_DELAY 200      // milliseconds of how long you need to keep rudder to max right for arming motors
#define DISARM_DELAY 100   // milliseconds of how long you need to keep rudder to max left for disarming motors


/*************************************************************/
// AM Mode & Flight information 

/* AM PIN Definitions */
/* Will be moved in future to AN extension ports */
/* due need to have PWM pins free for sonars and servos */

#define FR_LED 3  // Mega PE4 pin, OUT7
#define RE_LED 2  // Mega PE5 pin, OUT6
#define RI_LED 7  // Mega PH4 pin, OUT5
#define LE_LED 8  // Mega PH5 pin, OUT4

/* AM PIN Definitions - END */


/*************************************************************/
// Radio related definitions

// If you don't know these values, you can activate RADIO_TEST_MODE below
// and check your mid values

//#define RADIO_TEST_MODE

#define ROLL_MID 1500    // Radio Roll channel mid value
#define PITCH_MID 1500    // Radio Pitch channel mid value
#define YAW_MID 1500    // Radio Yaw channel mid value
#define THROTTLE_MID 1505    // Radio Throttle channel mid value
#define AUX_MID 1500

#define CHANN_CENTER 1500       // Channel center, legacy
#define MIN_THROTTLE 1040       // Throttle pulse width at minimun...

// Following variables stored in EEPROM
float KP_QUAD_ROLL;
float KI_QUAD_ROLL;
float STABLE_MODE_KP_RATE_ROLL;
float KP_QUAD_PITCH;
float KI_QUAD_PITCH;
float STABLE_MODE_KP_RATE_PITCH;
float KP_QUAD_YAW;
float KI_QUAD_YAW;
float STABLE_MODE_KP_RATE_YAW;
float STABLE_MODE_KP_RATE;       //NOT USED NOW
float KP_GPS_ROLL;
float KI_GPS_ROLL;
float KD_GPS_ROLL;
float KP_GPS_PITCH;
float KI_GPS_PITCH;
float KD_GPS_PITCH;
float GPS_MAX_ANGLE;
float KP_ALTITUDE;
float KI_ALTITUDE;
float KD_ALTITUDE;
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
float ch_roll_slope = 1;
float ch_pitch_slope = 1;
float ch_throttle_slope = 1;
float ch_yaw_slope = 1;
float ch_aux_slope = 1;
float ch_aux2_slope = 1;
float ch_roll_offset = 0;
float ch_pitch_offset = 0;
float ch_throttle_offset = 0;
float ch_yaw_offset = 0;
float ch_aux_offset = 0;
float ch_aux2_offset = 0;

// This function call contains the default values that are set to the ArduCopter
// when a "Default EEPROM Value" command is sent through serial interface
void defaultUserConfig() {
  KP_QUAD_ROLL = 4.0;
  KI_QUAD_ROLL = 0.15;
  STABLE_MODE_KP_RATE_ROLL = 1.2;
  KP_QUAD_PITCH = 4.0;
  KI_QUAD_PITCH = 0.15;
  STABLE_MODE_KP_RATE_PITCH = 1.2;
  KP_QUAD_YAW = 3.0;
  KI_QUAD_YAW = 0.15;
  STABLE_MODE_KP_RATE_YAW = 2.4;
  STABLE_MODE_KP_RATE = 0.2;     // NOT USED NOW
  KP_GPS_ROLL = 0.015;
  KI_GPS_ROLL = 0.005;
  KD_GPS_ROLL = 0.01;
  KP_GPS_PITCH = 0.015;
  KI_GPS_PITCH = 0.005;
  KD_GPS_PITCH = 0.01;
  GPS_MAX_ANGLE = 22;
  KP_ALTITUDE = 0.8;
  KI_ALTITUDE = 0.2;
  KD_ALTITUDE = 0.2;
  acc_offset_x = 2073;
  acc_offset_y = 2056;
  acc_offset_z = 2010;
  gyro_offset_roll = 1659;
  gyro_offset_pitch = 1618;
  gyro_offset_yaw = 1673;
  Kp_ROLLPITCH = 0.0014;
  Ki_ROLLPITCH = 0.00000015;
  Kp_YAW = 1.2;
  Ki_YAW = 0.00005;
  GEOG_CORRECTION_FACTOR = 0.87;
  MAGNETOMETER = 0;
  Kp_RateRoll = 1.95;
  Ki_RateRoll = 0.0;
  Kd_RateRoll = 0.0;
  Kp_RatePitch = 1.95;
  Ki_RatePitch = 0.0;
  Kd_RatePitch = 0.0;  
  Kp_RateYaw = 3.2;
  Ki_RateYaw = 0.0;
  Kd_RateYaw = 0.0;
  xmitFactor = 0.32;
  roll_mid = 1500;
  pitch_mid = 1500;
  yaw_mid = 1500;
  ch_roll_slope = 1;
  ch_pitch_slope = 1;
  ch_throttle_slope = 1;
  ch_yaw_slope = 1;
  ch_aux_slope = 1;
  ch_aux2_slope = 1;
  ch_roll_offset = 0;
  ch_pitch_offset = 0;
  ch_throttle_offset = 0;
  ch_yaw_offset = 0;
  ch_aux_offset = 0;
  ch_aux2_offset = 0;
}

// EEPROM storage addresses
#define KP_QUAD_ROLL_ADR 0
#define KI_QUAD_ROLL_ADR 8
#define STABLE_MODE_KP_RATE_ROLL_ADR 4
#define KP_QUAD_PITCH_ADR 12
#define KI_QUAD_PITCH_ADR 20
#define STABLE_MODE_KP_RATE_PITCH_ADR 16
#define KP_QUAD_YAW_ADR 24
#define KI_QUAD_YAW_ADR 32
#define STABLE_MODE_KP_RATE_YAW_ADR 28
#define STABLE_MODE_KP_RATE_ADR 36      // NOT USED NOW
#define KP_GPS_ROLL_ADR 40
#define KI_GPS_ROLL_ADR 48
#define KD_GPS_ROLL_ADR 44
#define KP_GPS_PITCH_ADR 52
#define KI_GPS_PITCH_ADR 60
#define KD_GPS_PITCH_ADR 56
#define GPS_MAX_ANGLE_ADR 64
#define KP_ALTITUDE_ADR 68
#define KI_ALTITUDE_ADR 76
#define KD_ALTITUDE_ADR 72
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
#define ch_roll_slope_ADR 180
#define ch_pitch_slope_ADR 184
#define ch_throttle_slope_ADR 188
#define ch_yaw_slope_ADR 192
#define ch_aux_slope_ADR 196
#define ch_aux2_slope_ADR 200
#define ch_roll_offset_ADR 204
#define ch_pitch_offset_ADR 208
#define ch_throttle_offset_ADR 212
#define ch_yaw_offset_ADR 216
#define ch_aux_offset_ADR 220
#define ch_aux2_offset_ADR 224

