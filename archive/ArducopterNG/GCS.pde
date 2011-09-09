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
    SerPri("MSG: ");
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
  if (SerAva()) {
    queryType = SerRea();
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
      KI_ALTITUDE = readFloatSerial();
      KD_ALTITUDE = readFloatSerial();
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
    case 'K': // Camera mode
              // 1 = Tilt / Roll without 
      cam_mode = readFloatSerial();
      //BATTLOW = readFloatSerial();
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
#if AIRFRAME == HELI
      heli_defaultUserConfig();
#endif      
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
    case '5': // Special debug features


      break;    
    }
  }
}

void sendSerialTelemetry() {
  float aux_float[3]; // used for sensor calibration
  switch (queryType) {
  case '=': // Reserved debug command to view any variable from Serial Monitor
    /*    SerPri(("throttle =");
     SerPrln(ch_throttle);
     SerPri("control roll =");
     SerPrln(control_roll-CHANN_CENTER);
     SerPri("control pitch =");
     SerPrln(control_pitch-CHANN_CENTER);
     SerPri("control yaw =");
     SerPrln(control_yaw-CHANN_CENTER);
     SerPri("front left yaw =");
     SerPrln(frontMotor);
     SerPri("front right yaw =");
     SerPrln(rightMotor);
     SerPri("rear left yaw =");
     SerPrln(leftMotor);
     SerPri("rear right motor =");
     SerPrln(backMotor);
     SerPrln();
     
     SerPri("current roll rate =");
     SerPrln(read_adc(0));
     SerPri("current pitch rate =");
     SerPrln(read_adc(1));
     SerPri("current yaw rate =");
     SerPrln(read_adc(2));
     SerPri("command rx yaw =");
     SerPrln(command_rx_yaw);
     SerPrln(); 
     queryType = 'X';*/
    SerPri(APM_RC.InputCh(0));
    comma();
    SerPri(ch_roll_slope);
    comma();
    SerPri(ch_roll_offset);
    comma();
    SerPrln(ch_roll);
    break;
  case 'B': // Send roll, pitch and yaw PID values
    SerPri(KP_QUAD_ROLL, 3);
    comma();
    SerPri(KI_QUAD_ROLL, 3);
    comma();
    SerPri(STABLE_MODE_KP_RATE_ROLL, 3);
    comma();
    SerPri(KP_QUAD_PITCH, 3);
    comma();
    SerPri(KI_QUAD_PITCH, 3);
    comma();
    SerPri(STABLE_MODE_KP_RATE_PITCH, 3);
    comma();
    SerPri(KP_QUAD_YAW, 3);
    comma();
    SerPri(KI_QUAD_YAW, 3);
    comma();
    SerPri(STABLE_MODE_KP_RATE_YAW, 3);
    comma();
    SerPri(STABLE_MODE_KP_RATE, 3);    // NOT USED NOW
    comma();
    SerPrln(MAGNETOMETER, 3);
    queryType = 'X';
    break;
  case 'D': // Send GPS PID
    SerPri(KP_GPS_ROLL, 3);
    comma();
    SerPri(KI_GPS_ROLL, 3);
    comma();
    SerPri(KD_GPS_ROLL, 3);
    comma();
    SerPri(KP_GPS_PITCH, 3);
    comma();
    SerPri(KI_GPS_PITCH, 3);
    comma();
    SerPri(KD_GPS_PITCH, 3);
    comma();
    SerPri(GPS_MAX_ANGLE, 3);
    comma();
    SerPrln(GEOG_CORRECTION_FACTOR, 3);
    queryType = 'X';
    break;
  case 'F': // Send altitude PID
    SerPri(KP_ALTITUDE, 3);
    comma();
    SerPri(KI_ALTITUDE, 3);
    comma();
    SerPrln(KD_ALTITUDE, 3);
    queryType = 'X';
    break;
  case 'H': // Send drift correction PID
    SerPri(Kp_ROLLPITCH, 4);
    comma();
    SerPri(Ki_ROLLPITCH, 7);
    comma();
    SerPri(Kp_YAW, 4);
    comma();
    SerPrln(Ki_YAW, 6);
    queryType = 'X';
    break;
  case 'J': // Send sensor offset
    SerPri(gyro_offset_roll);
    comma();
    SerPri(gyro_offset_pitch);
    comma();
    SerPri(gyro_offset_yaw);
    comma();
    SerPri(acc_offset_x);
    comma();
    SerPri(acc_offset_y);
    comma();
    SerPrln(acc_offset_z);
    AN_OFFSET[3] = acc_offset_x;
    AN_OFFSET[4] = acc_offset_y;
    AN_OFFSET[5] = acc_offset_z;
    queryType = 'X';
    break;
  case 'L': // Camera settings and 
    SerPri(cam_mode, DEC);
    tab();
    SerPri(BATTLOW, DEC);
    tab();
    queryType = 'X';
    break;
  case 'N': // Send magnetometer config
    queryType = 'X';
    break;
  case 'P': // Send rate control PID
    SerPri(Kp_RateRoll, 3);
    comma();
    SerPri(Ki_RateRoll, 3);
    comma();
    SerPri(Kd_RateRoll, 3);
    comma();
    SerPri(Kp_RatePitch, 3);
    comma();
    SerPri(Ki_RatePitch, 3);
    comma();
    SerPri(Kd_RatePitch, 3);
    comma();
    SerPri(Kp_RateYaw, 3);
    comma();
    SerPri(Ki_RateYaw, 3);
    comma();
    SerPri(Kd_RateYaw, 3);
    comma();
    SerPrln(xmitFactor, 3);
    queryType = 'X';
    break;
  case 'Q': // Send sensor data
    SerPri(read_adc(0));
    comma();
    SerPri(read_adc(1));
    comma();
    SerPri(read_adc(2));
    comma();
    SerPri(read_adc(4));
    comma();
    SerPri(read_adc(3));
    comma();
    SerPri(read_adc(5));
    comma();
    SerPri(err_roll);
    comma();
    SerPri(err_pitch);
    comma();
    SerPri(degrees(roll));
    comma();
    SerPri(degrees(pitch));
    comma();
    SerPrln(degrees(yaw));
    break;
  case 'R': // Send raw sensor data
    break;
  case 'S': // Send all flight data
    SerPri(timer-timer_old);
    comma();
    SerPri(read_adc(0));
    comma();
    SerPri(read_adc(1));
    comma();
    SerPri(read_adc(2));
    comma();
    SerPri(ch_throttle);
    comma();
    SerPri(control_roll);
    comma();
    SerPri(control_pitch);
    comma();
    SerPri(control_yaw);
    comma();
    SerPri(frontMotor); // Front Motor
    comma();
    SerPri(backMotor); // Back Motor
    comma();
    SerPri(rightMotor); // Right Motor
    comma();
    SerPri(leftMotor); // Left Motor
    comma();
    SerPri(read_adc(4));
    comma();
    SerPri(read_adc(3));
    comma();
    SerPri(read_adc(5));
    
     comma();
     SerPri(AP_Compass.heading, 4);
     comma();
     SerPri(AP_Compass.heading_x, 4);
     comma();
     SerPri(AP_Compass.heading_y, 4);
     comma();
     SerPri(AP_Compass.mag_x);
     comma();    
     SerPri(AP_Compass.mag_y);
     comma();
     SerPri(AP_Compass.mag_z);
//     comma();
     
    SerPriln();
    break;
  case 'T': // Spare
    break;
  case 'U': // Send receiver values
    SerPri(ch_roll); // Aileron
    comma();
    SerPri(ch_pitch); // Elevator
    comma();
    SerPri(ch_yaw); // Yaw
    comma();
    SerPri(ch_throttle); // Throttle
    comma();
    SerPri(ch_aux); // AUX1 (Mode)
    comma();
    SerPri(ch_aux2); // AUX2 
    comma();
    SerPri(roll_mid); // Roll MID value
    comma();
    SerPri(pitch_mid); // Pitch MID value
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
    SerPri(ch_roll_slope);
    comma();
    SerPri(ch_roll_offset);
    comma();
    SerPri(ch_pitch_slope);
    comma();
    SerPri(ch_pitch_offset);
    comma();
    SerPri(ch_yaw_slope);
    comma();
    SerPri(ch_yaw_offset);
    comma();
    SerPri(ch_throttle_slope);
    comma();
    SerPri(ch_throttle_offset);
    comma();
    SerPri(ch_aux_slope);
    comma();
    SerPri(ch_aux_offset);
    comma();
    SerPri(ch_aux2_slope);
    comma();
    SerPrln(ch_aux2_offset);
    queryType = 'X';
    break;
  case '3':  // Jani's debugs
    SerPri(yaw);
    tab();
    SerPri(command_rx_yaw);
    tab();
    SerPri(control_yaw);
    tab();
    SerPri(err_yaw);
    tab();
    SerPri(AN[0]);
    tab();
    SerPri(AN[1]);
    tab();
    SerPri(AN[2] - gyro_offset_yaw);
    tab();
    SerPri(gyro_offset_yaw - AN[2]);
    tab();
    SerPri(gyro_offset_yaw);
    tab();
    SerPri(1500 - (gyro_offset_yaw - AN[2]));
    tab();
    SerPriln();
    break;
#ifdef IsGPS
  case '4':  // Jani's debugs
//  Log_Write_GPS(GPS.Time, GPS.Lattitude, GPS.Longitude, GPS.Altitude, GPS.Altitude, GPS.Ground_Speed, GPS.Ground_Course, GPS.Fix, GPS.NumSats);

    SerPri(GPS.Time);
    tab();
    SerPri(GPS.Lattitude);
    tab();
    SerPri(GPS.Longitude);
    tab();
    SerPri(GPS.Altitude);
    tab();
    SerPri(GPS.Ground_Speed);
    tab();
    SerPri(GPS.Ground_Course);
    tab();
    SerPri(GPS.Fix);
    tab();
    SerPri(GPS.NumSats);

    tab();
    SerPriln();
    break;
#endif    
  case '.': // Modify GPS settings, print directly to GPS Port
    Serial1.print("$PGCMD,16,0,0,0,0,0*6A\r\n");
    break;
  }
}

void comma() {
  SerPri(',');
}

void tab() {
  SerPri("\t");
}

// Used to read floating point values from the serial port
float readFloatSerial() {
  byte index = 0;
  byte timeout = 0;
  char data[128] = "";

  do {
    if (SerAva() == 0) {
      delay(10);
      timeout++;
    }
    else {
      data[index] = SerRea();
      timeout = 0;
      index++;
    }
  }  
  while ((data[constrain(index-1, 0, 128)] != ';') && (timeout < 5) && (index < 128));
  return atof(data);
}


