/*
 www.ArduCopter.com - www.DIYDrones.com
 Copyright (c) 2010.  All rights reserved.
 An Open Source Arduino based multicopter.
 
 File     : GCS.pde
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
//
// Function  : send_message()
//
// Parameters: 
//  byte severity   - Debug level
//  char str        - Text to write
//
// Returns   : - none

void send_message(byte severity, const char *str)		// This is the instance of send_message for message 0x05
{
	if(severity >= DEBUG_LEVEL){
		SerPr("MSG: ");
		SerPrln(str);
	}
}


////////////////////////////////////////////////// 
// Function  : readSerialCommand()
//
// Parameters: 
//     - none
//
// Returns   : - none
//
void readSerialCommand() {
  // Check for serial message
  if (SerAv()) {
    queryType = SerRe();
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
/*    SerPr("throttle =");
    SerPrln(ch_throttle);
    SerPr("control roll =");
    SerPrln(control_roll-CHANN_CENTER);
    SerPr("control pitch =");
    SerPrln(control_pitch-CHANN_CENTER);
    SerPr("control yaw =");
    SerPrln(control_yaw-CHANN_CENTER);
    SerPr("front left yaw =");
    SerPrln(frontMotor);
    SerPr("front right yaw =");
    SerPrln(rightMotor);
    SerPr("rear left yaw =");
    SerPrln(leftMotor);
    SerPr("rear right motor =");
    SerPrln(backMotor);
    SerPrln();

    SerPr("current roll rate =");
    SerPrln(read_adc(0));
    SerPr("current pitch rate =");
    SerPrln(read_adc(1));
    SerPr("current yaw rate =");
    SerPrln(read_adc(2));
    SerPr("command rx yaw =");
    SerPrln(command_rx_yaw);
    SerPrln(); 
    queryType = 'X';*/
    SerPr(APM_RC.InputCh(0));
    comma();
    SerPr(ch_roll_slope);
    comma();
    SerPr(ch_roll_offset);
    comma();
    SerPrln(ch_roll);
    break;
  case 'B': // Send roll, pitch and yaw PID values
    SerPr(KP_QUAD_ROLL, 3);
    comma();
    SerPr(KI_QUAD_ROLL, 3);
    comma();
    SerPr(STABLE_MODE_KP_RATE_ROLL, 3);
    comma();
    SerPr(KP_QUAD_PITCH, 3);
    comma();
    SerPr(KI_QUAD_PITCH, 3);
    comma();
    SerPr(STABLE_MODE_KP_RATE_PITCH, 3);
    comma();
    SerPr(KP_QUAD_YAW, 3);
    comma();
    SerPr(KI_QUAD_YAW, 3);
    comma();
    SerPr(STABLE_MODE_KP_RATE_YAW, 3);
    comma();
    SerPr(STABLE_MODE_KP_RATE, 3);    // NOT USED NOW
    comma();
    SerPrln(MAGNETOMETER, 3);
    queryType = 'X';
    break;
  case 'D': // Send GPS PID
    SerPr(KP_GPS_ROLL, 3);
    comma();
    SerPr(KI_GPS_ROLL, 3);
    comma();
    SerPr(KD_GPS_ROLL, 3);
    comma();
    SerPr(KP_GPS_PITCH, 3);
    comma();
    SerPr(KI_GPS_PITCH, 3);
    comma();
    SerPr(KD_GPS_PITCH, 3);
    comma();
    SerPr(GPS_MAX_ANGLE, 3);
    comma();
    SerPrln(GEOG_CORRECTION_FACTOR, 3);
    queryType = 'X';
    break;
  case 'F': // Send altitude PID
    SerPr(KP_ALTITUDE, 3);
    comma();
    SerPr(KI_ALTITUDE, 3);
    comma();
    SerPrln(KD_ALTITUDE, 3);
    queryType = 'X';
    break;
  case 'H': // Send drift correction PID
    SerPr(Kp_ROLLPITCH, 4);
    comma();
    SerPr(Ki_ROLLPITCH, 7);
    comma();
    SerPr(Kp_YAW, 4);
    comma();
    SerPrln(Ki_YAW, 6);
    queryType = 'X';
    break;
  case 'J': // Send sensor offset
    SerPr(gyro_offset_roll);
    comma();
    SerPr(gyro_offset_pitch);
    comma();
    SerPr(gyro_offset_yaw);
    comma();
    SerPr(acc_offset_x);
    comma();
    SerPr(acc_offset_y);
    comma();
    SerPrln(acc_offset_z);
    AN_OFFSET[3] = acc_offset_x;
    AN_OFFSET[4] = acc_offset_y;
    AN_OFFSET[5] = acc_offset_z;
    queryType = 'X';
    break;
  case 'L': // Spare
//    RadioCalibration();
    queryType = 'X';
    break;
  case 'N': // Send magnetometer config
    queryType = 'X';
    break;
  case 'P': // Send rate control PID
    SerPr(Kp_RateRoll, 3);
    comma();
    SerPr(Ki_RateRoll, 3);
    comma();
    SerPr(Kd_RateRoll, 3);
    comma();
    SerPr(Kp_RatePitch, 3);
    comma();
    SerPr(Ki_RatePitch, 3);
    comma();
    SerPr(Kd_RatePitch, 3);
    comma();
    SerPr(Kp_RateYaw, 3);
    comma();
    SerPr(Ki_RateYaw, 3);
    comma();
    SerPr(Kd_RateYaw, 3);
    comma();
    SerPrln(xmitFactor, 3);
    queryType = 'X';
    break;
  case 'Q': // Send sensor data
    SerPr(read_adc(0));
    comma();
    SerPr(read_adc(1));
    comma();
    SerPr(read_adc(2));
    comma();
    SerPr(read_adc(4));
    comma();
    SerPr(read_adc(3));
    comma();
    SerPr(read_adc(5));
    comma();
    SerPr(err_roll);
    comma();
    SerPr(err_pitch);
    comma();
    SerPr(degrees(roll));
    comma();
    SerPr(degrees(pitch));
    comma();
    SerPrln(degrees(yaw));
    break;
  case 'R': // Send raw sensor data
    break;
  case 'S': // Send all flight data
    SerPr(timer-timer_old);
    comma();
    SerPr(read_adc(0));
    comma();
    SerPr(read_adc(1));
    comma();
    SerPr(read_adc(2));
    comma();
    SerPr(ch_throttle);
    comma();
    SerPr(control_roll);
    comma();
    SerPr(control_pitch);
    comma();
    SerPr(control_yaw);
    comma();
    SerPr(frontMotor); // Front Motor
    comma();
    SerPr(backMotor); // Back Motor
    comma();
    SerPr(rightMotor); // Right Motor
    comma();
    SerPr(leftMotor); // Left Motor
    comma();
    SerPr(read_adc(4));
    comma();
    SerPr(read_adc(3));
    comma();
    SerPrln(read_adc(5));
    break;
  case 'T': // Spare
    break;
  case 'U': // Send receiver values
    SerPr(ch_roll); // Aileron
    comma();
    SerPr(ch_pitch); // Elevator
    comma();
    SerPr(ch_yaw); // Yaw
    comma();
    SerPr(ch_throttle); // Throttle
    comma();
    SerPr(ch_aux); // AUX1 (Mode)
    comma();
    SerPr(ch_aux2); // AUX2 
    comma();
    SerPr(roll_mid); // Roll MID value
    comma();
    SerPr(pitch_mid); // Pitch MID value
    comma();
    SerPrln(yaw_mid); // Yaw MID Value
    break;
  case 'V': // Spare
    break;
  case 'X': // Stop sending messages
    break;
  case '!': // Send flight software version
    SerPrln(VER);
    queryType = 'X';
    break;
  case '2': // Send transmitter calibration values
    SerPr(ch_roll_slope);
    comma();
    SerPr(ch_roll_offset);
    comma();
    SerPr(ch_pitch_slope);
    comma();
    SerPr(ch_pitch_offset);
    comma();
    SerPr(ch_yaw_slope);
    comma();
    SerPr(ch_yaw_offset);
    comma();
    SerPr(ch_throttle_slope);
    comma();
    SerPr(ch_throttle_offset);
    comma();
    SerPr(ch_aux_slope);
    comma();
    SerPr(ch_aux_offset);
    comma();
    SerPr(ch_aux2_slope);
    comma();
    SerPrln(ch_aux2_offset);
    queryType = 'X';
  break;
  case '.': // Modify GPS settings, print directly to GPS Port
    Serial1.print("$PGCMD,16,0,0,0,0,0*6A\r\n");
    break;
  }
}

void comma() {
  SerPr(',');
}


// Used to read floating point values from the serial port
float readFloatSerial() {
  byte index = 0;
  byte timeout = 0;
  char data[128] = "";

  do {
    if (SerAv() == 0) {
      delay(10);
      timeout++;
    }
    else {
      data[index] = SerRe();
      timeout = 0;
      index++;
    }
  }  
  while ((data[constrain(index-1, 0, 128)] != ';') && (timeout < 5) && (index < 128));
  return atof(data);
}
