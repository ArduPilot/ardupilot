/*
  ArduCopter v1.2
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
}
