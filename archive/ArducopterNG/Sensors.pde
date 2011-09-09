/*
 www.ArduCopter.com - www.DIYDrones.com
 Copyright (c) 2010.  All rights reserved.
 An Open Source Arduino based multicopter.
 
 File     : Sensors.pde
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

* ************************************************************** */

/* ******* ADC functions ********************* */
// Read all the ADC channles
void Read_adc_raw(void)
{
  //int temp;
  
  for (int i=0;i<6;i++)
    AN[i] = adc.Ch(sensors[i]);
}

// Returns an analog value with the offset
int read_adc(int select)
{
  if (SENSOR_SIGN[select]<0)
    return (AN_OFFSET[select]-AN[select]);
  else
    return (AN[select]-AN_OFFSET[select]);
}

void calibrateSensors(void) {
  int i;
  int j = 0;
  byte gyro;
  float aux_float[3];
  
  Read_adc_raw();     // Read sensors data
  delay(5);

  // Offset values for accels and gyros...
  AN_OFFSET[3] = acc_offset_x;                // Accel offset values are taken from external calibration (In Configurator)
  AN_OFFSET[4] = acc_offset_y;
  AN_OFFSET[5] = acc_offset_z;
  aux_float[0] = gyro_offset_roll;
  aux_float[1] = gyro_offset_pitch;
  aux_float[2] = gyro_offset_yaw;

  // Take the gyro offset values
  for(i=0;i<600;i++)
  {
    Read_adc_raw();   // Read sensors
    for(gyro = GYROZ; gyro <= GYROY; gyro++)   
      aux_float[gyro] = aux_float[gyro] * 0.8 + AN[gyro] * 0.2;     // Filtering  
    Log_Write_Sensor(AN[0], AN[1], AN[2], AN[3], AN[4], AN[5], 0);

    delay(5);

    RunningLights(j);   // (in Functions.pde)
    // Runnings lights effect to let user know that we are taking mesurements
    if((i % 5) == 0) j++;
    if(j >= 3) j = 0;
  }
  
  // Switch off all ABC lights
  LightsOff();

  for(gyro = GYROZ; gyro <= GYROY; gyro++)  
    AN_OFFSET[gyro] = aux_float[gyro];    // Update sensor OFFSETs from values read
}

#ifdef UseBMP
void read_baro(void)
{
  float tempPresAlt;
  
  tempPresAlt = float(APM_BMP085.Press)/101325.0;
  //tempPresAlt = pow(tempPresAlt, 0.190284);
  //press_alt = (1.0 - tempPresAlt) * 145366.45;
  tempPresAlt = pow(tempPresAlt, 0.190295);
  if (press_baro_altitude == 0)
    press_baro_altitude = (1.0 - tempPresAlt) * 4433000;      // Altitude in cm
  else
    press_baro_altitude = press_baro_altitude * 0.75 + ((1.0 - tempPresAlt) * 4433000)*0.25;  // Altitude in cm (filtered)
}
#endif

#ifdef IsSONAR
/* This function reads in the values from the attached range finders (currently only downward pointing sonar) */
void read_Sonar()
{ 
  // calculate altitude from down sensor
  AP_RangeFinder_down.read();
  
  // translate into an altitude
  press_sonar_altitude = DCM_Matrix[2][2] * AP_RangeFinder_down.distance;
  
  // deal with the unusual case that we're up-side-down
  if( press_sonar_altitude < 0 )
    press_sonar_altitude = 0;  
  
  // set sonar status to OK and update sonar_valid_count which shows reliability of sonar (i.e. are we out of range?)
  if( AP_RangeFinder_down.distance > sonar_threshold ) {
      sonar_status = SONAR_STATUS_BAD;
      if( sonar_valid_count > 0 )
          sonar_valid_count = -1;
      else
          sonar_valid_count--;
  }else{
      sonar_status = SONAR_STATUS_OK;
      if( sonar_valid_count < 0 )
          sonar_valid_count = 1;
      else
          sonar_valid_count++;
  }
  sonar_valid_count = constrain(sonar_valid_count,-10,10);

#if LOG_RANGEFINDER && !defined(IsRANGEFINDER)
    Log_Write_RangeFinder(AP_RangeFinder_down.distance,0,0,0,0,0);
#endif
}
#endif  // IsSONAR

#ifdef IsRANGEFINDER
/* This function reads in the values from the attached range finders (currently only downward pointing sonar) */
void read_RF_Sensors()
{
  AP_RangeFinder_frontRight.read();
  AP_RangeFinder_backRight.read();
  AP_RangeFinder_backLeft.read();
  AP_RangeFinder_frontLeft.read();

#if LOG_RANGEFINDER 
    Log_Write_RangeFinder(AP_RangeFinder_down.distance, AP_RangeFinder_frontRight.distance, AP_RangeFinder_backRight.distance, AP_RangeFinder_backLeft.distance,AP_RangeFinder_frontLeft.distance,0);
#endif
}
#endif  // IsRANGEFINDER

