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
    AN[i] = APM_ADC.Ch(sensors[i]);
}

// Returns an analog value with the offset
int read_adc(int select)
{
  if (SENSOR_SIGN[select]<0)
    return (AN_OFFSET[select]-AN[select]);
  else
    return (AN[select]-AN_OFFSET[select]);
}

int readADC(byte channel) {
  if (sensorSign[channel] < 0)
    return (zeroADC[channel] - APM_ADC.Ch(channel));
  else
    return (APM_ADC.Ch(channel) - zeroADC[channel]);
}

void calibrateSensors(void) {
  int j = 0;
  // Take the gyro offset values
  for(int i=0;i<300;i++) {
    for (channel = GYROZ; channel < LASTSENSOR; channel++) {
      rawADC[channel] = APM_ADC.Ch(channel);
      zeroADC[channel] = (zeroADC[channel] * 0.8) + (rawADC[channel] * 0.2);
      //Log_Write_Sensor(rawADC[GYROZ], rawADC[GYROX], rawADC[GYROY], rawADC[ACCELX], rawADC[ACCELY], rawADC[ACCELZ], receiverData[THROTTLE]);
    }
    delay(5);
    // Runnings lights effect to let user know that we are taking mesurements
    if(j == 0) {
      digitalWrite(LED_Green, HIGH);
      digitalWrite(LED_Yellow, LOW);
      digitalWrite(LED_Red, LOW);
    } 
    else if (j == 1) {
      digitalWrite(LED_Green, LOW);
      digitalWrite(LED_Yellow, HIGH);
      digitalWrite(LED_Red, LOW);
    } 
    else {
      digitalWrite(LED_Green, LOW);
      digitalWrite(LED_Yellow, LOW);
      digitalWrite(LED_Red, HIGH);
    }
    if((i % 5) == 0) j++;
    if(j >= 3) j = 0;
  }
  digitalWrite(LED_Green, LOW);
  digitalWrite(LED_Yellow, LOW);
  digitalWrite(LED_Red, LOW);
}
