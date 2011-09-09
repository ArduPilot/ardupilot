/*
 www.ArduCopter.com - www.DIYDrones.com
 Copyright (c) 2010.  All rights reserved.
 An Open Source Arduino based multicopter.
 
 File     : EEPROM.pde
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
//  eeprom_counter = readEEPROM(eeprom_counter_ADR);
//  eeprom_checker = readEEPROM(eeprom_checker_ADR);
  KP_QUAD_ROLL = readEEPROM(KP_QUAD_ROLL_ADR);
  KI_QUAD_ROLL = readEEPROM(KI_QUAD_ROLL_ADR);
  STABLE_MODE_KP_RATE_ROLL = readEEPROM(STABLE_MODE_KP_RATE_ROLL_ADR);
  KP_QUAD_PITCH = readEEPROM(KP_QUAD_PITCH_ADR);
  KI_QUAD_PITCH = readEEPROM(KI_QUAD_PITCH_ADR);
  STABLE_MODE_KP_RATE_PITCH = readEEPROM(STABLE_MODE_KP_RATE_PITCH_ADR);
  KP_QUAD_YAW = readEEPROM(KP_QUAD_YAW_ADR);
  KI_QUAD_YAW = readEEPROM(KI_QUAD_YAW_ADR);
  STABLE_MODE_KP_RATE_YAW = readEEPROM(STABLE_MODE_KP_RATE_YAW_ADR);
  STABLE_MODE_KP_RATE = readEEPROM(STABLE_MODE_KP_RATE_ADR);          // NOT USED NOW
  KP_GPS_ROLL = readEEPROM(KP_GPS_ROLL_ADR);
  KI_GPS_ROLL = readEEPROM(KI_GPS_ROLL_ADR);
  KD_GPS_ROLL = readEEPROM(KD_GPS_ROLL_ADR);
  KP_GPS_PITCH = readEEPROM(KP_GPS_PITCH_ADR);
  KI_GPS_PITCH = readEEPROM(KI_GPS_PITCH_ADR);
  KD_GPS_PITCH = readEEPROM(KD_GPS_PITCH_ADR);
  GPS_MAX_ANGLE = readEEPROM(GPS_MAX_ANGLE_ADR);
  KP_ALTITUDE = readEEPROM(KP_ALTITUDE_ADR);
  KI_ALTITUDE = readEEPROM(KI_ALTITUDE_ADR);
  KD_ALTITUDE = readEEPROM(KD_ALTITUDE_ADR);
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
  ch_roll_slope = readEEPROM(ch_roll_slope_ADR);
  ch_pitch_slope = readEEPROM(ch_pitch_slope_ADR);
  ch_throttle_slope = readEEPROM(ch_throttle_slope_ADR);
  ch_yaw_slope = readEEPROM(ch_yaw_slope_ADR);
  ch_aux_slope = readEEPROM(ch_aux_slope_ADR);
  ch_aux2_slope = readEEPROM(ch_aux2_slope_ADR);
  ch_roll_offset = readEEPROM(ch_roll_offset_ADR);
  ch_pitch_offset = readEEPROM(ch_pitch_offset_ADR);
  ch_throttle_offset = readEEPROM(ch_throttle_offset_ADR);
  ch_yaw_offset = readEEPROM(ch_yaw_offset_ADR);
  ch_aux_offset = readEEPROM(ch_aux_offset_ADR);
  ch_aux2_offset = readEEPROM(ch_aux2_offset_ADR);
  cam_mode = readEEPROM(cam_mode_ADR);
  mag_orientation = readEEPROM(mag_orientation_ADR);
  mag_declination = readEEPROM(mag_declination_ADR);
  mag_offset_x = readEEPROM(mag_offset_x_ADR);
  mag_offset_y = readEEPROM(mag_offset_y_ADR);
  mag_offset_z = readEEPROM(mag_offset_z_ADR);
  MIN_THROTTLE = readEEPROM(MIN_THROTTLE_ADR);
  KP_RF_ROLL = readEEPROM(KP_RF_ROLL_ADR);
  KD_RF_ROLL = readEEPROM(KD_RF_ROLL_ADR);
  KI_RF_ROLL = readEEPROM(KI_RF_ROLL_ADR);
  KP_RF_PITCH = readEEPROM(KP_RF_PITCH_ADR);
  KD_RF_PITCH = readEEPROM(KD_RF_PITCH_ADR);
  KI_RF_PITCH = readEEPROM(KI_RF_PITCH_ADR);
  RF_MAX_ANGLE = readEEPROM(RF_MAX_ANGLE_ADR);
  RF_SAFETY_ZONE = readEEPROM(RF_SAFETY_ZONE_ADR);
  KP_SONAR_ALTITUDE = readEEPROM(KP_SONAR_ALTITUDE_ADR);
  KI_SONAR_ALTITUDE = readEEPROM(KI_SONAR_ALTITUDE_ADR);
  KD_SONAR_ALTITUDE = readEEPROM(KD_SONAR_ALTITUDE_ADR);
}

void writeUserConfig() {
//  eeprom_counter = readEEPROM(eeprom_counter_ADR);
//  if(eeprom_counter > 0) eeprom_counter = 0;
//  eeprom_counter++;
//  writeEEPROM(eeprom_counter, eeprom_counter_ADR);
  writeEEPROM(KP_QUAD_ROLL, KP_QUAD_ROLL_ADR);
  writeEEPROM(KI_QUAD_ROLL, KI_QUAD_ROLL_ADR);
  writeEEPROM(STABLE_MODE_KP_RATE_ROLL, STABLE_MODE_KP_RATE_ROLL_ADR);
  writeEEPROM(KP_QUAD_PITCH, KP_QUAD_PITCH_ADR);
  writeEEPROM(KI_QUAD_PITCH, KI_QUAD_PITCH_ADR);
  writeEEPROM(STABLE_MODE_KP_RATE_PITCH, STABLE_MODE_KP_RATE_PITCH_ADR);
  writeEEPROM(KP_QUAD_YAW, KP_QUAD_YAW_ADR);
  writeEEPROM(KI_QUAD_YAW, KI_QUAD_YAW_ADR);
  writeEEPROM(STABLE_MODE_KP_RATE_YAW, STABLE_MODE_KP_RATE_YAW_ADR);
  writeEEPROM(STABLE_MODE_KP_RATE, STABLE_MODE_KP_RATE_ADR);  // NOT USED NOW
  writeEEPROM(KP_GPS_ROLL, KP_GPS_ROLL_ADR);
  writeEEPROM(KD_GPS_ROLL, KD_GPS_ROLL_ADR);
  writeEEPROM(KI_GPS_ROLL, KI_GPS_ROLL_ADR);
  writeEEPROM(KP_GPS_PITCH, KP_GPS_PITCH_ADR);
  writeEEPROM(KD_GPS_PITCH, KD_GPS_PITCH_ADR);
  writeEEPROM(KI_GPS_PITCH, KI_GPS_PITCH_ADR);
  writeEEPROM(GPS_MAX_ANGLE, GPS_MAX_ANGLE_ADR);
  writeEEPROM(KP_ALTITUDE, KP_ALTITUDE_ADR);
  writeEEPROM(KD_ALTITUDE, KD_ALTITUDE_ADR);
  writeEEPROM(KI_ALTITUDE, KI_ALTITUDE_ADR);
  writeEEPROM(acc_offset_x, acc_offset_x_ADR);
  writeEEPROM(acc_offset_y, acc_offset_y_ADR);
  writeEEPROM(acc_offset_z, acc_offset_z_ADR);
  writeEEPROM(gyro_offset_roll, gyro_offset_roll_ADR);
  writeEEPROM(gyro_offset_pitch, gyro_offset_pitch_ADR);
  writeEEPROM(gyro_offset_yaw, gyro_offset_yaw_ADR);
  writeEEPROM(Kp_ROLLPITCH, Kp_ROLLPITCH_ADR);
  writeEEPROM(Ki_ROLLPITCH, Ki_ROLLPITCH_ADR);
  writeEEPROM(Kp_YAW, Kp_YAW_ADR);
  writeEEPROM(Ki_YAW, Ki_YAW_ADR);
  writeEEPROM(GEOG_CORRECTION_FACTOR, GEOG_CORRECTION_FACTOR_ADR);
  writeEEPROM(MAGNETOMETER, MAGNETOMETER_ADR);
  writeEEPROM(Kp_RateRoll, KP_RATEROLL_ADR);
  writeEEPROM(Ki_RateRoll, KI_RATEROLL_ADR);
  writeEEPROM(Kd_RateRoll, KD_RATEROLL_ADR);
  writeEEPROM(Kp_RatePitch, KP_RATEPITCH_ADR);
  writeEEPROM(Ki_RatePitch, KI_RATEPITCH_ADR);
  writeEEPROM(Kd_RatePitch, KD_RATEPITCH_ADR);
  writeEEPROM(Kp_RateYaw, KP_RATEYAW_ADR);
  writeEEPROM(Ki_RateYaw, KI_RATEYAW_ADR);
  writeEEPROM(Kd_RateYaw, KD_RATEYAW_ADR);
  writeEEPROM(xmitFactor, XMITFACTOR_ADR);
  writeEEPROM(roll_mid, CHROLL_MID);
  writeEEPROM(pitch_mid, CHPITCH_MID);
  writeEEPROM(yaw_mid, CHYAW_MID);
  writeEEPROM(ch_roll_slope, ch_roll_slope_ADR);
  writeEEPROM(ch_pitch_slope, ch_pitch_slope_ADR);
  writeEEPROM(ch_throttle_slope, ch_throttle_slope_ADR);
  writeEEPROM(ch_yaw_slope, ch_yaw_slope_ADR);
  writeEEPROM(ch_aux_slope, ch_aux_slope_ADR);
  writeEEPROM(ch_aux2_slope, ch_aux2_slope_ADR);
  writeEEPROM(ch_roll_offset, ch_roll_offset_ADR);
  writeEEPROM(ch_pitch_offset, ch_pitch_offset_ADR);
  writeEEPROM(ch_throttle_offset, ch_throttle_offset_ADR);
  writeEEPROM(ch_yaw_offset, ch_yaw_offset_ADR);
  writeEEPROM(ch_aux_offset, ch_aux_offset_ADR);
  writeEEPROM(ch_aux2_offset, ch_aux2_offset_ADR);
  writeEEPROM(cam_mode, cam_mode_ADR);
  writeEEPROM(mag_orientation, mag_orientation_ADR);
  writeEEPROM(mag_declination, mag_declination_ADR);
  writeEEPROM(mag_offset_x, mag_offset_x_ADR);
  writeEEPROM(mag_offset_y, mag_offset_y_ADR);
  writeEEPROM(mag_offset_z, mag_offset_z_ADR);
  writeEEPROM(MIN_THROTTLE, MIN_THROTTLE_ADR);
  writeEEPROM(KP_RF_ROLL,KP_RF_ROLL_ADR);
  writeEEPROM(KI_RF_ROLL,KI_RF_ROLL_ADR);
  writeEEPROM(KD_RF_ROLL,KD_RF_ROLL_ADR);
  writeEEPROM(KP_RF_PITCH,KP_RF_PITCH_ADR);
  writeEEPROM(KI_RF_PITCH,KI_RF_PITCH_ADR);  
  writeEEPROM(KD_RF_PITCH,KD_RF_PITCH_ADR);
  writeEEPROM(RF_MAX_ANGLE,RF_MAX_ANGLE_ADR);
  writeEEPROM(RF_SAFETY_ZONE,RF_SAFETY_ZONE_ADR);
  writeEEPROM(KP_SONAR_ALTITUDE,KP_SONAR_ALTITUDE_ADR);
  writeEEPROM(KI_SONAR_ALTITUDE,KI_SONAR_ALTITUDE_ADR);
  writeEEPROM(KD_SONAR_ALTITUDE,KD_SONAR_ALTITUDE_ADR);  
}
