/*
   ArduCopter 1.3 - August 2010
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

void RadioCalibration() {
  long command_timer;
  int command; 
  int counter = 5;
  boolean Cmd_ok; 
  long roll_new = 0;
  long pitch_new = 0;
  long yaw_new = 0;

  SerFlu();
  SerPriln("Entering Radio Calibration mode");
  SerPriln("Current channel MID values are:");
  SerPri("ROLL: ");
  SerPri(roll_mid);
  SerPri(" PITCH: ");
  SerPri(pitch_mid);
  SerPri(" YAW: ");
  SerPri(yaw_mid);
  SerPriln();
  SerPriln("Recalibrate Channel MID's [Y/N]?: ");
  command_timer = millis();

  // Start counter loop and wait serial input. If not input for 5 seconds, return to normal mode
  while(millis() - command_timer < 5000) {
    if (SerAva()) {
      queryType = SerRea();
      if(queryType == 'y' || queryType == 'Y') {  
        Cmd_ok = TRUE;
        break;    
      } 
      else {
        Cmd_ok = FALSE;     
        break;
      }
    }
  }
  if(Cmd_ok) {
    // We have a go. Let's do new calibration
    SerPriln("Starting calibration run in 5 seconds. Place all sticks to their middle including trims");
    for(counter = 5; counter >= 0; counter --) {
      command_timer = millis();
      while(millis() - command_timer < 1000) {
      }
      SerPriln(counter);
    }
    // Do actual calibration now
    SerPriln("Measuring average channel values");
    SerPriln("ROLL, PITCH, YAW");

    counter = 0; // Reset counter for just in case. 
    command_timer = millis();
    while(millis() - command_timer < 1000) {

      if (APM_RC.GetState()==1) {  // New radio frame?
        // Commands from radio Rx... 
        ch_roll = channel_filter(APM_RC.InputCh(0), ch_roll);
        ch_pitch = channel_filter(APM_RC.InputCh(1), ch_pitch);
        ch_throttle = channel_filter(APM_RC.InputCh(2), ch_throttle);
        ch_yaw = channel_filter(APM_RC.InputCh(3), ch_yaw);
        ch_aux = APM_RC.InputCh(4);
        ch_aux2 = APM_RC.InputCh(5);

        SerPri(ch_roll);
        comma();
        SerPri(ch_pitch);
        comma();
        SerPri(ch_yaw);
        SerPriln();
        roll_new += ch_roll;
        pitch_new += ch_pitch; 
        yaw_new += ch_yaw;
        counter++;
      }
    }
    SerPri("New samples received: ");
    SerPriln(counter);    
    roll_new = roll_new / counter;
    pitch_new = pitch_new / counter;
    yaw_new = yaw_new / counter;
    SerPri("New values as: ");
    SerPri("ROLL: ");
    SerPri(roll_new);
    SerPri(" PITCH: ");
    SerPri(pitch_new);
    SerPri(" YAW: ");
    SerPri(yaw_new);
    SerPriln();
    SerPriln("Accept & Save values [Y/N]?: ");
    Cmd_ok = FALSE;
    while(millis() - command_timer < 5000) {
      if (SerAva()) {
        queryType = SerRea();
        if(queryType == 'y' || queryType == 'Y') { 
          Cmd_ok = TRUE;
          roll_mid = roll_new;
          pitch_mid = pitch_new;
          yaw_mid = yaw_new;
          SerPriln("Values accepted, remember to save them to EEPROM with 'W' command");
          break;    
        } 
        else {
          Cmd_ok = TRUE;
          break;
        }
      }   
    } 
  } 
  if(queryType == 'n' || queryType == 'N') Cmd_ok = TRUE;
  if(Cmd_ok)
    SerPriln("Returning normal mode...");
  else SerPriln("Command timeout, returning normal mode....");
}

void comma() {
  SerPri(',');
}

#if BATTERY_EVENT == 1
void low_battery_event(void)
{
//	send_message(SEVERITY_HIGH,"Low Battery!");
//	set_mode(RTL);
//	throttle_cruise = THROTTLE_CRUISE;
}
#endif






