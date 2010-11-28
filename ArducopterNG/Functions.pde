/*
 www.ArduCopter.com - www.DIYDrones.com
 Copyright (c) 2010.  All rights reserved.
 An Open Source Arduino based multicopter.
 
 File     : Functions.pde
 Version  : v1.0, Aug 28, 2010
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
 30-10-10 added basic camera stabilization functions with jumptables
 
 * ************************************************************** *
 TODO:
 
 
 * ************************************************************** */


// Flash those A,B,C LEDs on IMU Board
// 
// Function: FullBlink(int, int);
//           int 1 = 
void FullBlink(int count, int blinkdelay) {
  for(int i = 0; i <= count; i++) {
    digitalWrite(LED_Green, HIGH);
    digitalWrite(LED_Yellow, HIGH);
    digitalWrite(LED_Red, HIGH);
    delay(blinkdelay);
    digitalWrite(LED_Green, LOW);
    digitalWrite(LED_Yellow, LOW);
    digitalWrite(LED_Red, LOW);
    delay(blinkdelay);
  }
}


void RunningLights(int LightStep) {

  if(LightStep == 0) {
    digitalWrite(LED_Green, HIGH);
    digitalWrite(LED_Yellow, LOW);
    digitalWrite(LED_Red, LOW);
  } 
  else if (LightStep == 1) {
    digitalWrite(LED_Green, LOW);
    digitalWrite(LED_Yellow, HIGH);
    digitalWrite(LED_Red, LOW);
  } 
  else {
    digitalWrite(LED_Green, LOW);
    digitalWrite(LED_Yellow, LOW);
    digitalWrite(LED_Red, HIGH);
  }

}

void LightsOff() {
  digitalWrite(LED_Green, LOW);
  digitalWrite(LED_Yellow, LOW);
  digitalWrite(LED_Red, LOW);
}

// Funtion to normalize an angle in degrees to -180,180 degrees
float Normalize_angle(float angle)
{
  if (angle > 180)         
    return (angle - 360.0);
  else if (angle < -180)
    return (angle + 360.0);
  else
    return(angle);
}

// Maximun slope filter for radio inputs... (limit max differences between readings)
int channel_filter(int ch, int ch_old)
{
  int diff_ch_old;

  if (ch_old==0)      // ch_old not initialized
    return(ch);
  diff_ch_old = ch - ch_old;      // Difference with old reading
  if (diff_ch_old < 0)
  {
    if (diff_ch_old <- 60)
      return(ch_old - 60);        // We limit the max difference between readings
  }
  else
  {
    if (diff_ch_old > 60)    
      return(ch_old + 60);
  }
  return((ch + ch_old) >> 1);   // Small filtering
  //return(ch);
}


// Special APM PinMode settings and others
void APMPinMode(volatile unsigned char &Port, byte Pin, boolean Set)
{
  if (Set)  {
    Port |=   (1 << Pin);
  } 
  else  {
    Port &=  ~(1 << Pin);
  }
}

boolean APMPinRead(volatile unsigned char &Port, byte Pin)
{
  if(Port   &   (1 << Pin))
    return 1;
  else
    return 0;
}

// Faster and smaller replacement for contrain() function
int limitRange(int data, int minLimit, int maxLimit) {
  if (data < minLimit) return minLimit;
  else if (data > maxLimit) return maxLimit;
  else return data;
}


// Stepping G, Y, R Leds
// Call CLILedStep(); to change led statuses
// Used on CLI as showing that we are in CLI mode
void CLILedStep () {
  
  switch(cli_step) {
  case 1:
        digitalWrite(LED_Green, HIGH);
        digitalWrite(LED_Yellow, LOW);
        digitalWrite(LED_Red, LOW);
  break;
  case 2:
        digitalWrite(LED_Green, LOW);
        digitalWrite(LED_Yellow, HIGH);
        digitalWrite(LED_Red, LOW);
  break;
  case 3:
        digitalWrite(LED_Green, LOW);
        digitalWrite(LED_Yellow, LOW);
        digitalWrite(LED_Red, HIGH);
  break;
  }
  cli_step ++; 
  if(cli_step == 4) cli_step = 1;  
  
}

void LEDAllON() {
        digitalWrite(LED_Green, HIGH);
        digitalWrite(LED_Red, HIGH);
        digitalWrite(LED_Yellow, HIGH);
}

void LEDAllOFF() {
        digitalWrite(LED_Green, LOW);
        digitalWrite(LED_Red, LOW);
        digitalWrite(LED_Yellow, LOW);
}


//
// Camera functions moved to event due it's and event 31-10-10, jp



