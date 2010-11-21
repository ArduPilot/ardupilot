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
}

void Reset_I_terms_navigation()
{
  altitude_I = 0;
  gps_roll_I = 0;
  gps_pitch_I = 0;  
}

/* ************************************************************ */
/* Altitude control... (based on barometer) */
int Altitude_control_baro(int altitude, int target_altitude)
{ 
  #define ALTITUDE_CONTROL_BARO_OUTPUT_MIN 40
  #define ALTITUDE_CONTROL_BARO_OUTPUT_MAX 80
  
  #define KP_BARO_ALTITUDE 0.25  //0.3
  #define KD_BARO_ALTITUDE 0.09 //0.09
  #define KI_BARO_ALTITUDE 0.1
  
  int command_altitude;
  
  err_altitude_old = err_altitude;
  err_altitude = target_altitude - altitude;  
  altitude_D = (float)(err_altitude-err_altitude_old)/0.05;  // 20Hz
  altitude_I += (float)err_altitude*0.05;
  altitude_I = constrain(altitude_I,-120,120);
  command_altitude = KP_BARO_ALTITUDE*err_altitude + KD_BARO_ALTITUDE*altitude_D + KI_BARO_ALTITUDE*altitude_I;
  command_altitude = Initial_Throttle + constrain(command_altitude,-ALTITUDE_CONTROL_BARO_OUTPUT_MIN,ALTITUDE_CONTROL_BARO_OUTPUT_MAX);
  return command_altitude;
}

/* Altitude control... (based on sonar) */
/* CONTROL PARAMETERS FOR SONAR ALTITUDE CONTROL (TEMPORATLY HERE) */
#define KP_SONAR_ALTITUDE 0.8
#define KD_SONAR_ALTITUDE 0.7
#define KI_SONAR_ALTITUDE 0.3
int Altitude_control_Sonar(int Sonar_altitude, int target_sonar_altitude)
{
  #define ALTITUDE_CONTROL_SONAR_OUTPUT_MIN 60
  #define ALTITUDE_CONTROL_SONAR_OUTPUT_MAX 80
  
  int command_altitude;
   
  err_altitude_old = err_altitude;
  err_altitude = target_sonar_altitude - Sonar_altitude;  
  altitude_D = (float)(err_altitude-err_altitude_old)/0.05;
  altitude_I += (float)err_altitude*0.05;
  altitude_I = constrain(altitude_I,-1000,1000);
  command_altitude = KP_SONAR_ALTITUDE*err_altitude + KD_SONAR_ALTITUDE*altitude_D + KI_SONAR_ALTITUDE*altitude_I;
  return (Initial_Throttle + constrain(command_altitude,-ALTITUDE_CONTROL_SONAR_OUTPUT_MIN,ALTITUDE_CONTROL_SONAR_OUTPUT_MAX));
}


