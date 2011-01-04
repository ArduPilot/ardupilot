/*
 www.ArduCopter.com - www.DIYDrones.com
 Copyright (c) 2010.  All rights reserved.
 An Open Source Arduino based multicopter.
 
 File     : Navigation.pde
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
 - initial functions.
 
 * ************************************************************** */

void read_GPS_data()
{
#ifdef IsGPS
  GPS_timer_old=GPS_timer;   // Update GPS timer
  GPS_timer = millis();
  GPS_Dt = (GPS_timer-GPS_timer_old)*0.001;   // GPS_Dt
  GPS.NewData=0;    // We Reset the flag...

  // Write GPS data to DataFlash log
  Log_Write_GPS(GPS.Time, GPS.Lattitude, GPS.Longitude, GPS.Altitude, GPS.Altitude, GPS.Ground_Speed, GPS.Ground_Course, GPS.Fix, GPS.NumSats);

  //if (GPS.Fix >= 2)
  if (GPS.Fix)
    digitalWrite(LED_Red,HIGH);  // GPS Fix => RED LED
  else
    digitalWrite(LED_Red,LOW);
#endif
}

/* GPS based Position control */
void Position_control(long lat_dest, long lon_dest)
{
#ifdef IsGPS
  long Lon_diff;
  long Lat_diff;

  Lon_diff = lon_dest - GPS.Longitude;
  Lat_diff = lat_dest - GPS.Lattitude;

  //If we have not calculated GEOG_CORRECTION_FACTOR we calculate it here as cos(lattitude)
  if (GEOG_CORRECTION_FACTOR==0)
    GEOG_CORRECTION_FACTOR = cos(ToRad(GPS.Lattitude/10000000.0));

  // ROLL
  //Optimization : cos(yaw) = DCM_Matrix[0][0] ;  sin(yaw) = DCM_Matrix[1][0]   [This simplification is valid for low roll angles]
  gps_err_roll = (float)Lon_diff * GEOG_CORRECTION_FACTOR * DCM_Matrix[0][0] - (float)Lat_diff * DCM_Matrix[1][0];

  gps_roll_D = (gps_err_roll-gps_err_roll_old) / GPS_Dt;
  gps_err_roll_old = gps_err_roll;

  gps_roll_I += gps_err_roll * GPS_Dt;
  gps_roll_I = constrain(gps_roll_I, -800, 800);

  command_gps_roll = KP_GPS_ROLL * gps_err_roll + KD_GPS_ROLL * gps_roll_D + KI_GPS_ROLL * gps_roll_I;
  command_gps_roll = constrain(command_gps_roll, -GPS_MAX_ANGLE, GPS_MAX_ANGLE); // Limit max command

  //Log_Write_PID(1,KP_GPS_ROLL*gps_err_roll*10,KI_GPS_ROLL*gps_roll_I*10,KD_GPS_ROLL*gps_roll_D*10,command_gps_roll*10);

  // PITCH
  gps_err_pitch = -(float)Lat_diff * DCM_Matrix[0][0] - (float)Lon_diff * GEOG_CORRECTION_FACTOR * DCM_Matrix[1][0];

  gps_pitch_D = (gps_err_pitch - gps_err_pitch_old) / GPS_Dt;
  gps_err_pitch_old = gps_err_pitch;

  gps_pitch_I += gps_err_pitch * GPS_Dt;
  gps_pitch_I = constrain(gps_pitch_I, -800, 800);

  command_gps_pitch = KP_GPS_PITCH * gps_err_pitch + KD_GPS_PITCH * gps_pitch_D + KI_GPS_PITCH * gps_pitch_I;
  command_gps_pitch = constrain(command_gps_pitch, -GPS_MAX_ANGLE, GPS_MAX_ANGLE); // Limit max command

  //Log_Write_PID(2,KP_GPS_PITCH*gps_err_pitch*10,KI_GPS_PITCH*gps_pitch_I*10,KD_GPS_PITCH*gps_pitch_D*10,command_gps_pitch*10);
#endif  
}

/* GPS based Position control Version 2 - builds up I and D term using lat/lon instead of roll/pitch*/
void Position_control_v2(long lat_dest, long lon_dest)
{
#ifdef IsGPS
  //If we have not calculated GEOG_CORRECTION_FACTOR we calculate it here as cos(lattitude)
  if (GEOG_CORRECTION_FACTOR==0)
    GEOG_CORRECTION_FACTOR = cos(ToRad(GPS.Lattitude/10000000.0));
    
  // store old lat & lon diff for d term?
  gps_err_lon_old = gps_err_lon;
  gps_err_lat_old = gps_err_lat;
  
  // calculate distance from target - for P term
  gps_err_lon = (float)(lon_dest - GPS.Longitude) * GEOG_CORRECTION_FACTOR;
  gps_err_lat = lat_dest - GPS.Lattitude;
  
  // add distance to I term
  gps_lon_I += gps_err_lon;
  gps_lon_I = constrain(gps_lon_I,-1200,1200);  // don't let I get too big
  gps_lat_I += gps_err_lat;
  gps_lat_I = constrain(gps_lat_I,-1200,1200);
  
  // calculate the ground speed - for D term
  gps_lon_D = (gps_err_lon - gps_err_lon_old) / GPS_Dt;
  gps_lat_D = (gps_err_lat - gps_err_lat_old) / GPS_Dt;

  // Now separate lat & lon PID terms into roll & pitch components
  // ROLL
  //Optimization : cos(yaw) = DCM_Matrix[0][0] ;  sin(yaw) = DCM_Matrix[1][0]   [This simplification is valid for low roll angles]
  gps_err_roll = (gps_err_lon * DCM_Matrix[0][0] - gps_err_lat * DCM_Matrix[1][0]);
  gps_roll_I = (gps_lon_I * DCM_Matrix[0][0] - gps_lat_I * DCM_Matrix[1][0]);
  gps_roll_D = (gps_lon_D * DCM_Matrix[0][0] - gps_lat_D * DCM_Matrix[1][0]);  

  command_gps_roll = KP_GPS_ROLL * gps_err_roll + KD_GPS_ROLL * gps_roll_D + KI_GPS_ROLL * gps_roll_I;
  command_gps_roll = constrain(command_gps_roll, -GPS_MAX_ANGLE, GPS_MAX_ANGLE); // Limit max command
  Log_Write_PID(1,KP_GPS_ROLL*gps_err_roll,KI_GPS_ROLL*gps_roll_I,KD_GPS_ROLL*gps_roll_D,command_gps_roll);

  // PITCH
  gps_err_pitch = (-gps_err_lat * DCM_Matrix[0][0] - gps_err_lon * DCM_Matrix[1][0]);
  gps_pitch_I = (-gps_lat_I * DCM_Matrix[0][0] - gps_lon_I * DCM_Matrix[1][0]);
  gps_pitch_D = (-gps_lat_D * DCM_Matrix[0][0] - gps_lon_D * DCM_Matrix[1][0]);

  command_gps_pitch = KP_GPS_PITCH * gps_err_pitch + KD_GPS_PITCH * gps_pitch_D + KI_GPS_PITCH * gps_pitch_I;
  command_gps_pitch = constrain(command_gps_pitch, -GPS_MAX_ANGLE, GPS_MAX_ANGLE); // Limit max command
  Log_Write_PID(2,KP_GPS_PITCH*gps_err_pitch,KI_GPS_PITCH*gps_pitch_I,KD_GPS_PITCH*gps_pitch_D,command_gps_pitch);
  
#endif  
}

void Reset_I_terms_navigation()
{
  gps_roll_I = 0;
  gps_pitch_I = 0;  
  gps_lon_I = 0;  // for position hold ver 2
  gps_lat_I = 0;
}

/* ************************************************************ */
/* Altitude control... (based on barometer) */
int Altitude_control_baro(int altitude, int target_altitude)
{ 
  #define ALTITUDE_CONTROL_BARO_OUTPUT_MIN 40
  #define ALTITUDE_CONTROL_BARO_OUTPUT_MAX 80
  
  // !!!!! REMOVE THIS !!!!!!!
  #define KP_BARO_ALTITUDE 0.25  //0.3
  #define KD_BARO_ALTITUDE 0.09 //0.09
  #define KI_BARO_ALTITUDE 0.1
  
  int command_altitude;
  
  err_altitude_old = err_altitude;
  err_altitude = target_altitude - altitude;  
  baro_altitude_I += (float)err_altitude*0.05;
  baro_altitude_I = constrain(baro_altitude_I,-140,140);
  baro_altitude_D = (float)(err_altitude-err_altitude_old)/0.05;  // 20Hz  
  command_altitude = KP_ALTITUDE*err_altitude + KD_ALTITUDE*baro_altitude_D + KI_ALTITUDE*baro_altitude_I;
  command_altitude = initial_throttle + constrain(command_altitude,-ALTITUDE_CONTROL_BARO_OUTPUT_MIN,ALTITUDE_CONTROL_BARO_OUTPUT_MAX);
  Log_Write_PID(5,KP_ALTITUDE*err_altitude,KI_ALTITUDE*baro_altitude_I,KD_ALTITUDE*baro_altitude_D,command_altitude);  
  return command_altitude;
}

/* ************************************************************ */
/* Altitude control... (based on sonar) */
#define GdT_SONAR_ALTITUDE 0.05
#define ALTITUDE_CONTROL_SONAR_OUTPUT_MIN 60
#define ALTITUDE_CONTROL_SONAR_OUTPUT_MAX 80
int Altitude_control_Sonar(int altitude, int target_altitude)
{
  static int err_altitude = 0;
  int command_altitude;
  int err_altitude_old;
   
  err_altitude_old = err_altitude;
  err_altitude = target_altitude - altitude;  
  sonar_altitude_I += (float)err_altitude*GdT_SONAR_ALTITUDE;
  sonar_altitude_I = constrain(sonar_altitude_I,-1000,1000);
  sonar_altitude_D = (float)(err_altitude-err_altitude_old)/GdT_SONAR_ALTITUDE;  
  command_altitude = KP_SONAR_ALTITUDE*err_altitude + KI_SONAR_ALTITUDE*sonar_altitude_I + KD_SONAR_ALTITUDE*sonar_altitude_D ;
  command_altitude = initial_throttle + constrain(command_altitude,-ALTITUDE_CONTROL_SONAR_OUTPUT_MIN,ALTITUDE_CONTROL_SONAR_OUTPUT_MAX);
  Log_Write_PID(4,KP_SONAR_ALTITUDE*err_altitude,KI_SONAR_ALTITUDE*sonar_altitude_I,KD_SONAR_ALTITUDE*sonar_altitude_D,command_altitude);
  return command_altitude;
}

/* ************************************************************ */
/* Obstacle avoidance routine */
#ifdef IsRANGEFINDER
void Obstacle_avoidance(int safeDistance)
{
  int RF_err_roll = 0;
  int RF_err_pitch = 0;
  int RF_err_throttle = 0;
  float RF_roll_P;
  float RF_roll_D;
  float RF_pitch_P;
  float RF_pitch_D;
  float RF_throttle_P;
  float RF_throttle_D;
  static int RF_err_roll_old;
  static int RF_err_pitch_old;
  static int RF_err_throttle_old;
  int err_temp;
  
  // front right
  err_temp = max(safeDistance - AP_RangeFinder_frontRight.distance,0);
  RF_err_roll += err_temp * AP_RangeFinder_frontRight.orientation_x;
  RF_err_pitch += err_temp * AP_RangeFinder_frontRight.orientation_y;
  RF_err_throttle += err_temp * AP_RangeFinder_frontRight.orientation_z;  

  // back right
  err_temp = max(safeDistance - AP_RangeFinder_backRight.distance,0);
  RF_err_roll += err_temp * AP_RangeFinder_backRight.orientation_x;
  RF_err_pitch += err_temp * AP_RangeFinder_backRight.orientation_y;
  RF_err_throttle += err_temp * AP_RangeFinder_backRight.orientation_z;  

  // back left
  err_temp = max(safeDistance - AP_RangeFinder_backLeft.distance,0);
  RF_err_roll += err_temp * AP_RangeFinder_backLeft.orientation_x;
  RF_err_pitch += err_temp * AP_RangeFinder_backLeft.orientation_y;
  RF_err_throttle += err_temp * AP_RangeFinder_backLeft.orientation_z;  
  
  // front left
  err_temp = max(safeDistance - AP_RangeFinder_frontLeft.distance,0);
  RF_err_roll += err_temp * AP_RangeFinder_frontLeft.orientation_x;
  RF_err_pitch += err_temp * AP_RangeFinder_frontLeft.orientation_y;
  RF_err_throttle += err_temp * AP_RangeFinder_frontLeft.orientation_z;

  // ROLL - P term
  RF_roll_P = RF_err_roll * KP_RF_ROLL;
  RF_roll_P = constrain(RF_roll_P,-RF_MAX_ANGLE,RF_MAX_ANGLE);
  // ROLL - I term
  RF_roll_I += RF_err_roll * 0.05 * KI_RF_ROLL;  
  RF_roll_I = constrain(RF_roll_I,-RF_MAX_ANGLE/2,RF_MAX_ANGLE/2);
  // ROLL - D term
  RF_roll_D = (RF_err_roll-RF_err_roll_old) / 0.05 * KD_RF_ROLL;   // RF_IR frequency is 20Hz (50ms)
  RF_roll_D = constrain(RF_roll_D,-RF_MAX_ANGLE/2,RF_MAX_ANGLE/2);
  RF_err_roll_old = RF_err_roll;
  // ROLL - full comand
  command_RF_roll = RF_roll_P + RF_roll_I + RF_roll_D;
  command_RF_roll = constrain(command_RF_roll,-RF_MAX_ANGLE,RF_MAX_ANGLE); // Limit max command
  
  // PITCH - P term
  RF_pitch_P = RF_err_pitch * KP_RF_PITCH;
  RF_pitch_P = constrain(RF_pitch_P,-RF_MAX_ANGLE,RF_MAX_ANGLE);
  // PITCH - I term
  RF_pitch_I += RF_err_pitch * 0.05 * KI_RF_PITCH;  
  RF_pitch_I = constrain(RF_pitch_I,-RF_MAX_ANGLE/2,RF_MAX_ANGLE/2);
  // PITCH - D term
  RF_pitch_D = (RF_err_pitch-RF_err_pitch_old) / 0.05 * KD_RF_PITCH;   // RF_IR frequency is 20Hz (50ms)
  RF_pitch_D = constrain(RF_pitch_D,-RF_MAX_ANGLE/2,RF_MAX_ANGLE/2);
  RF_err_pitch_old = RF_err_pitch;
  // PITCH - full comand
  command_RF_pitch = RF_pitch_P + RF_pitch_I + RF_pitch_D;
  command_RF_pitch = constrain(command_RF_pitch,-RF_MAX_ANGLE,RF_MAX_ANGLE); // Limit max command
  
  // THROTTLE - not yet implemented
  command_RF_throttle = 0;
}
#endif

