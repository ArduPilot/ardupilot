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
  if (APM_RC.GetState() == 1)   // New radio frame?
    {
    // Commands from radio Rx
    // We apply the Radio calibration parameters (from configurator) except for throttle
    ch_roll = channel_filter(APM_RC.InputCh(CH_ROLL) * ch_roll_slope + ch_roll_offset, ch_roll);
    ch_pitch = channel_filter(APM_RC.InputCh(CH_PITCH) * ch_pitch_slope + ch_pitch_offset, ch_pitch);
    ch_throttle = channel_filter(APM_RC.InputCh(CH_THROTTLE), ch_throttle); // Transmiter calibration not used on throttle
    ch_yaw = channel_filter(APM_RC.InputCh(CH_RUDDER) * ch_yaw_slope + ch_yaw_offset, ch_yaw);
    ch_aux = APM_RC.InputCh(CH_5) * ch_aux_slope + ch_aux_offset;
    ch_aux2 = APM_RC.InputCh(CH_6) * ch_aux2_slope + ch_aux2_offset;
    
    // Flight mode
    if(ch_aux2 > 1800) 
      flightMode = 1;  // Force to Acro mode from radio
    else
      flightMode = 0;  // Stable mode (default)

    // Autopilot mode (only works on Stable mode)
    if (flightMode == 0)
      {
      if(ch_aux > 1800)
        AP_mode = 1;   // Automatic mode : GPS position hold mode + altitude hold
      else 
        AP_mode = 0;   // Normal mode
      }
      
    if (flightMode==0)  // IN STABLE MODE we convert stick positions to absoulte angles
      {
      // In Stable mode stick position defines the desired angle in roll, pitch and yaw
      #ifdef FLIGHT_MODE_X
        // For X mode we make a mix in the input
        float aux_roll = (ch_roll-roll_mid) / STICK_TO_ANGLE_FACTOR;
        float aux_pitch = (ch_pitch-pitch_mid) / STICK_TO_ANGLE_FACTOR;
        command_rx_roll = aux_roll - aux_pitch;
        command_rx_pitch = aux_roll + aux_pitch;
      #else
        command_rx_roll = (ch_roll-roll_mid) / STICK_TO_ANGLE_FACTOR;       // Convert stick position to absolute angles
        command_rx_pitch = (ch_pitch-pitch_mid) / STICK_TO_ANGLE_FACTOR;
      #endif

      // YAW
      if (abs(ch_yaw-yaw_mid)>6)   // Take into account a bit of "dead zone" on yaw
        {
        command_rx_yaw += (ch_yaw-yaw_mid) / YAW_STICK_TO_ANGLE_FACTOR;
        command_rx_yaw = Normalize_angle(command_rx_yaw);    // Normalize angle to [-180,180]
        }
      }
    
    // Write Radio data to DataFlash log
    Log_Write_Radio(ch_roll,ch_pitch,ch_throttle,ch_yaw,int(K_aux*100),(int)AP_mode);
    }  // END new radio data
}


// Send output commands to ESC´s
void motor_output()
{
  // Quadcopter mix
  if (motorArmed == 1) 
    {   
  #ifdef IsAM
    digitalWrite(FR_LED, HIGH);    // AM-Mode
  #endif
    // Quadcopter output mix
    rightMotor = constrain(ch_throttle - control_roll + control_yaw, minThrottle, 2000);
    leftMotor = constrain(ch_throttle + control_roll + control_yaw, minThrottle, 2000);
    frontMotor = constrain(ch_throttle + control_pitch - control_yaw, minThrottle, 2000);
    backMotor = constrain(ch_throttle - control_pitch - control_yaw, minThrottle, 2000);
    }
  else    // MOTORS DISARMED
    {    
  #ifdef IsAM
    digitalWrite(FR_LED, LOW);    // AM-Mode
  #endif
    digitalWrite(LED_Green,HIGH); // Ready LED on

    rightMotor = MIN_THROTTLE;
    leftMotor = MIN_THROTTLE;
    frontMotor = MIN_THROTTLE;
    backMotor = MIN_THROTTLE;

    // Reset_I_Terms();
    roll_I = 0;     // reset I terms of PID controls
    pitch_I = 0;
    yaw_I = 0; 
    // Initialize yaw command to actual yaw when throttle is down...
    command_rx_yaw = ToDeg(yaw);
    }

  // Send commands to motors
  APM_RC.OutputCh(0, rightMotor);
  APM_RC.OutputCh(1, leftMotor);
  APM_RC.OutputCh(2, frontMotor);
  APM_RC.OutputCh(3, backMotor);

  // InstantPWM => Force inmediate output on PWM signals
  APM_RC.Force_Out0_Out1();
  APM_RC.Force_Out2_Out3();
}

