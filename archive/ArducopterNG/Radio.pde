/*
 www.ArduCopter.com - www.DIYDrones.com
 Copyright (c) 2010.  All rights reserved.
 An Open Source Arduino based multicopter.
 
 File     : Radio.pde
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

#define STICK_TO_ANGLE_FACTOR 12.0        // To convert stick position to absolute angles
#define YAW_STICK_TO_ANGLE_FACTOR 150.0

void read_radio()
{
    int tempThrottle = 0;
  
    // Commands from radio Rx
    // We apply the Radio calibration parameters (from configurator) except for throttle
    ch_roll = channel_filter(APM_RC.InputCh(CH_ROLL) * ch_roll_slope + ch_roll_offset, ch_roll);
    ch_pitch = channel_filter(APM_RC.InputCh(CH_PITCH) * ch_pitch_slope + ch_pitch_offset, ch_pitch);
    ch_yaw = channel_filter(APM_RC.InputCh(CH_RUDDER) * ch_yaw_slope + ch_yaw_offset, ch_yaw);
    ch_aux = APM_RC.InputCh(CH_5) * ch_aux_slope + ch_aux_offset;
    ch_aux2 = APM_RC.InputCh(CH_6) * ch_aux2_slope + ch_aux2_offset;   
    
    // special checks for throttle
    tempThrottle = APM_RC.InputCh(CH_THROTTLE);

    // throttle safety check
    if( motorSafety == 1 ) {
        if( motorArmed == 1 ) {
            if( ch_throttle > MIN_THROTTLE + 100 ) { // if throttle is now over MIN..
                // if throttle has increased suddenly, disarm motors 
                if( (tempThrottle - ch_throttle) > SAFETY_MAX_THROTTLE_INCREASE  ) {     
                    motorArmed = 0;
                }else{ // if it hasn't increased too quickly turn off the safety
                    motorSafety = 0;
                }
            }
        }
    }else if( ch_throttle < MIN_THROTTLE + 100 ) { // Safety logic: hold throttle low for more than 1 second, safety comes on which stops sudden increases in throttle
        Safety_counter++;
        if( Safety_counter > SAFETY_DELAY ) {
            motorSafety = 1;
            Safety_counter = 0;
        }
    }else{  // throttle is over MIN so make sure to reset Safety_counter
         Safety_counter = 0;
    } 
    // normal throttle filtering.  Note: Transmiter calibration not used on throttle
    ch_throttle = channel_filter(tempThrottle, ch_throttle);
        
    // Flight mode
    if(ch_aux2 > 1300) 
      flightMode = ACRO_MODE;  // Force to Acro mode from radio
    else
      flightMode = STABLE_MODE;  // Stable mode (default)

    // Autopilot mode (only works on Stable mode)
    if (flightMode == STABLE_MODE)
      {
      if(ch_aux < 1300)
        AP_mode = AP_AUTOMATIC_MODE;   // Automatic mode : GPS position hold mode + altitude hold
      else 
        AP_mode = AP_NORMAL_MODE;   // Normal mode
      }
      
    if (flightMode==STABLE_MODE)  // IN STABLE MODE we convert stick positions to absoulte angles
      {
      // In Stable mode stick position defines the desired angle in roll, pitch and yaw
//      #ifdef FLIGHT_MODE_X
      if(flightOrientation) {
        // For X mode we make a mix in the input
        float aux_roll = (ch_roll-roll_mid) / STICK_TO_ANGLE_FACTOR;
        float aux_pitch = (ch_pitch-pitch_mid) / STICK_TO_ANGLE_FACTOR;
        command_rx_roll = aux_roll - aux_pitch;
        command_rx_pitch = aux_roll + aux_pitch;
      } else {
        command_rx_roll = (ch_roll-roll_mid) / STICK_TO_ANGLE_FACTOR;       // Convert stick position to absolute angles
        command_rx_pitch = (ch_pitch-pitch_mid) / STICK_TO_ANGLE_FACTOR;
      }

      // YAW
      if (abs(ch_yaw-yaw_mid)>6)   // Take into account a bit of "dead zone" on yaw
        {
        command_rx_yaw += (ch_yaw-yaw_mid) / YAW_STICK_TO_ANGLE_FACTOR;
        command_rx_yaw = Normalize_angle(command_rx_yaw);    // Normalize angle to [-180,180]
        }
      }
    
    // Write Radio data to DataFlash log
    Log_Write_Radio(ch_roll,ch_pitch,ch_throttle,ch_yaw,ch_aux,ch_aux2);
    
    // Motor arm logic
    if (ch_throttle < (MIN_THROTTLE + 100)) {
      control_yaw = 0;
      command_rx_yaw = ToDeg(yaw);
      
      // Arm motor output : Throttle down and full yaw right for more than 2 seconds
      if (ch_yaw > 1850) {
        if (Arming_counter > ARM_DELAY){
          if(ch_throttle > 800) {
          motorArmed = 1;
          minThrottle = MIN_THROTTLE+60;  // A minimun value for mantain a bit if throttle
          }
        }
        else
          Arming_counter++;
      }
      else
        Arming_counter=0;
        
      // To Disarm motor output : Throttle down and full yaw left for more than 2 seconds
      if (ch_yaw < 1150) {
        if (Disarming_counter > DISARM_DELAY){
          motorArmed = 0;
          minThrottle = MIN_THROTTLE;
        }
        else
          Disarming_counter++;
      }
      else
        Disarming_counter=0;
    }
    else{
      Arming_counter=0;
      Disarming_counter=0;
    }
}


