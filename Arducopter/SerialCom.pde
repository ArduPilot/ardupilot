/*
 ArduCopter v1.3 - August 2010
 www.ArduCopter.com
 Copyright (c) 2010.  All rights reserved.
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

void readSerialCommand() {
  // Check for serial message
  if (Serial.available()) {
    queryType = Serial.read();
    switch (queryType) {
    case 'A': // Stable PID
      KP_QUAD_ROLL = readFloatSerial();
      KI_QUAD_ROLL = readFloatSerial();
      STABLE_MODE_KP_RATE_ROLL = readFloatSerial();
      KP_QUAD_PITCH = readFloatSerial();
      KI_QUAD_PITCH = readFloatSerial();
      STABLE_MODE_KP_RATE_PITCH = readFloatSerial();
      KP_QUAD_YAW = readFloatSerial();
      KI_QUAD_YAW = readFloatSerial();
      STABLE_MODE_KP_RATE_YAW = readFloatSerial();
      STABLE_MODE_KP_RATE = readFloatSerial();   // NOT USED NOW
      MAGNETOMETER = readFloatSerial();
      break;
    case 'C': // Receive GPS PID
      KP_GPS_ROLL = readFloatSerial();
      KI_GPS_ROLL = readFloatSerial();
      KD_GPS_ROLL = readFloatSerial();
      KP_GPS_PITCH = readFloatSerial();
      KI_GPS_PITCH = readFloatSerial();
      KD_GPS_PITCH = readFloatSerial();
      GPS_MAX_ANGLE = readFloatSerial();
      GEOG_CORRECTION_FACTOR = readFloatSerial();
      break;
    case 'E': // Receive altitude PID
      KP_ALTITUDE = readFloatSerial();
      KD_ALTITUDE = readFloatSerial();
      KI_ALTITUDE = readFloatSerial();
      break;
    case 'G': // Receive drift correction PID
      Kp_ROLLPITCH = readFloatSerial();
      Ki_ROLLPITCH = readFloatSerial();
      Kp_YAW = readFloatSerial();
      Ki_YAW = readFloatSerial();
      break;
    case 'I': // Receive sensor offset
      gyro_offset_roll = readFloatSerial();
      gyro_offset_pitch = readFloatSerial();
      gyro_offset_yaw = readFloatSerial();
      acc_offset_x = readFloatSerial();
      acc_offset_y = readFloatSerial();
      acc_offset_z = readFloatSerial();
      break;
    case 'K': // Spare
      break;
    case 'M': // Receive debug motor commands
      frontMotor = readFloatSerial();
      backMotor = readFloatSerial();
      rightMotor = readFloatSerial();
      leftMotor = readFloatSerial();
      motorArmed = readFloatSerial();
      break;
    case 'O': // Rate Control PID
      Kp_RateRoll = readFloatSerial();
      Ki_RateRoll = readFloatSerial();
      Kd_RateRoll = readFloatSerial();
      Kp_RatePitch = readFloatSerial();
      Ki_RatePitch = readFloatSerial();
      Kd_RatePitch = readFloatSerial();
      Kp_RateYaw = readFloatSerial();
      Ki_RateYaw = readFloatSerial();
      Kd_RateYaw = readFloatSerial();
      xmitFactor = readFloatSerial();
      break;
    case 'W': // Write all user configurable values to EEPROM
      writeUserConfig();
      break;
    case 'Y': // Initialize EEPROM with default values
      defaultUserConfig();
      break;
    case '1': // Receive transmitter calibration values
      ch_roll_slope = readFloatSerial();
      ch_roll_offset = readFloatSerial();
      ch_pitch_slope = readFloatSerial();
      ch_pitch_offset = readFloatSerial();
      ch_yaw_slope = readFloatSerial();
      ch_yaw_offset = readFloatSerial();
      ch_throttle_slope = readFloatSerial();
      ch_throttle_offset = readFloatSerial();
      ch_aux_slope = readFloatSerial();
      ch_aux_offset = readFloatSerial();
      ch_aux2_slope = readFloatSerial();
      ch_aux2_offset = readFloatSerial();
    break;
    }
  }
}

void sendSerialTelemetry() {
  float aux_float[3]; // used for sensor calibration
  switch (queryType) {
  case '=': // Reserved debug command to view any variable from Serial Monitor
/*    Serial.print("throttle =");
    Serial.println(ch_throttle);
    Serial.print("control roll =");
    Serial.println(control_roll-CHANN_CENTER);
    Serial.print("control pitch =");
    Serial.println(control_pitch-CHANN_CENTER);
    Serial.print("control yaw =");
    Serial.println(control_yaw-CHANN_CENTER);
    Serial.print("front left yaw =");
    Serial.println(frontMotor);
    Serial.print("front right yaw =");
    Serial.println(rightMotor);
    Serial.print("rear left yaw =");
    Serial.println(leftMotor);
    Serial.print("rear right motor =");
    Serial.println(backMotor);
    Serial.println();

    Serial.print("current roll rate =");
    Serial.println(read_adc(0));
    Serial.print("current pitch rate =");
    Serial.println(read_adc(1));
    Serial.print("current yaw rate =");
    Serial.println(read_adc(2));
    Serial.print("command rx yaw =");
    Serial.println(command_rx_yaw);
    Serial.println(); 
    queryType = 'X';*/
    Serial.print(APM_RC.InputCh(0));
    comma();
    Serial.print(ch_roll_slope);
    comma();
    Serial.print(ch_roll_offset);
    comma();
    Serial.println(ch_roll);
    break;
  case 'B': // Send roll, pitch and yaw PID values
    Serial.print(KP_QUAD_ROLL, 3);
    comma();
    Serial.print(KI_QUAD_ROLL, 3);
    comma();
    Serial.print(STABLE_MODE_KP_RATE_ROLL, 3);
    comma();
    Serial.print(KP_QUAD_PITCH, 3);
    comma();
    Serial.print(KI_QUAD_PITCH, 3);
    comma();
    Serial.print(STABLE_MODE_KP_RATE_PITCH, 3);
    comma();
    Serial.print(KP_QUAD_YAW, 3);
    comma();
    Serial.print(KI_QUAD_YAW, 3);
    comma();
    Serial.print(STABLE_MODE_KP_RATE_YAW, 3);
    comma();
    Serial.print(STABLE_MODE_KP_RATE, 3);    // NOT USED NOW
    comma();
    Serial.println(MAGNETOMETER, 3);
    queryType = 'X';
    break;
  case 'D': // Send GPS PID
    Serial.print(KP_GPS_ROLL, 3);
    comma();
    Serial.print(KI_GPS_ROLL, 3);
    comma();
    Serial.print(KD_GPS_ROLL, 3);
    comma();
    Serial.print(KP_GPS_PITCH, 3);
    comma();
    Serial.print(KI_GPS_PITCH, 3);
    comma();
    Serial.print(KD_GPS_PITCH, 3);
    comma();
    Serial.print(GPS_MAX_ANGLE, 3);
    comma();
    Serial.println(GEOG_CORRECTION_FACTOR, 3);
    queryType = 'X';
    break;
  case 'F': // Send altitude PID
    Serial.print(KP_ALTITUDE, 3);
    comma();
    Serial.print(KI_ALTITUDE, 3);
    comma();
    Serial.println(KD_ALTITUDE, 3);
    queryType = 'X';
    break;
  case 'H': // Send drift correction PID
    Serial.print(Kp_ROLLPITCH, 4);
    comma();
    Serial.print(Ki_ROLLPITCH, 7);
    comma();
    Serial.print(Kp_YAW, 4);
    comma();
    Serial.println(Ki_YAW, 6);
    queryType = 'X';
    break;
  case 'J': // Send sensor offset
    Serial.print(gyro_offset_roll);
    comma();
    Serial.print(gyro_offset_pitch);
    comma();
    Serial.print(gyro_offset_yaw);
    comma();
    Serial.print(acc_offset_x);
    comma();
    Serial.print(acc_offset_y);
    comma();
    Serial.println(acc_offset_z);
    AN_OFFSET[3] = acc_offset_x;
    AN_OFFSET[4] = acc_offset_y;
    AN_OFFSET[5] = acc_offset_z;
    queryType = 'X';
    break;
  case 'L': // Spare
    RadioCalibration();
    queryType = 'X';
    break;
  case 'N': // Send magnetometer config
    queryType = 'X';
    break;
  case 'P': // Send rate control PID
    Serial.print(Kp_RateRoll, 3);
    comma();
    Serial.print(Ki_RateRoll, 3);
    comma();
    Serial.print(Kd_RateRoll, 3);
    comma();
    Serial.print(Kp_RatePitch, 3);
    comma();
    Serial.print(Ki_RatePitch, 3);
    comma();
    Serial.print(Kd_RatePitch, 3);
    comma();
    Serial.print(Kp_RateYaw, 3);
    comma();
    Serial.print(Ki_RateYaw, 3);
    comma();
    Serial.print(Kd_RateYaw, 3);
    comma();
    Serial.println(xmitFactor, 3);
    queryType = 'X';
    break;
  case 'Q': // Send sensor data
    Serial.print(read_adc(0));
    comma();
    Serial.print(read_adc(1));
    comma();
    Serial.print(read_adc(2));
    comma();
    Serial.print(read_adc(4));
    comma();
    Serial.print(read_adc(3));
    comma();
    Serial.print(read_adc(5));
    comma();
    Serial.print(err_roll);
    comma();
    Serial.print(err_pitch);
    comma();
    Serial.print(degrees(roll));
    comma();
    Serial.print(degrees(pitch));
    comma();
    Serial.println(degrees(yaw));
    break;
  case 'R': // Send raw sensor data
    break;
  case 'S': // Send all flight data
    Serial.print(timer-timer_old);
    comma();
    Serial.print(read_adc(0));
    comma();
    Serial.print(read_adc(1));
    comma();
    Serial.print(read_adc(2));
    comma();
    Serial.print(ch_throttle);
    comma();
    Serial.print(control_roll);
    comma();
    Serial.print(control_pitch);
    comma();
    Serial.print(control_yaw);
    comma();
    Serial.print(frontMotor); // Front Motor
    comma();
    Serial.print(backMotor); // Back Motor
    comma();
    Serial.print(rightMotor); // Right Motor
    comma();
    Serial.print(leftMotor); // Left Motor
    comma();
    Serial.print(read_adc(4));
    comma();
    Serial.print(read_adc(3));
    comma();
    Serial.println(read_adc(5));
    break;
  case 'T': // Spare
    break;
  case 'U': // Send receiver values
    Serial.print(ch_roll); // Aileron
    comma();
    Serial.print(ch_pitch); // Elevator
    comma();
    Serial.print(ch_yaw); // Yaw
    comma();
    Serial.print(ch_throttle); // Throttle
    comma();
    Serial.print(ch_aux); // AUX1 (Mode)
    comma();
    Serial.print(ch_aux2); // AUX2 
    comma();
    Serial.print(roll_mid); // Roll MID value
    comma();
    Serial.print(pitch_mid); // Pitch MID value
    comma();
    Serial.println(yaw_mid); // Yaw MID Value
    break;
  case 'V': // Spare
    break;
  case 'X': // Stop sending messages
    break;
  case '!': // Send flight software version
    Serial.println(VER);
    queryType = 'X';
    break;
  case '2': // Send transmitter calibration values
    Serial.print(ch_roll_slope);
    comma();
    Serial.print(ch_roll_offset);
    comma();
    Serial.print(ch_pitch_slope);
    comma();
    Serial.print(ch_pitch_offset);
    comma();
    Serial.print(ch_yaw_slope);
    comma();
    Serial.print(ch_yaw_offset);
    comma();
    Serial.print(ch_throttle_slope);
    comma();
    Serial.print(ch_throttle_offset);
    comma();
    Serial.print(ch_aux_slope);
    comma();
    Serial.print(ch_aux_offset);
    comma();
    Serial.print(ch_aux2_slope);
    comma();
    Serial.println(ch_aux2_offset);
    queryType = 'X';
  break;
  case '.': // Modify GPS settings
    Serial1.print("$PGCMD,16,0,0,0,0,0*6A\r\n");
    break;
  }
}

// Used to read floating point values from the serial port
float readFloatSerial() {
  byte index = 0;
  byte timeout = 0;
  char data[128] = "";

  do {
    if (Serial.available() == 0) {
      delay(10);
      timeout++;
    }
    else {
      data[index] = Serial.read();
      timeout = 0;
      index++;
    }
  }  
  while ((data[constrain(index-1, 0, 128)] != ';') && (timeout < 5) && (index < 128));
  return atof(data);
}
