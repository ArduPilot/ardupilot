/*
 www.ArduCopter.com - www.DIYDrones.com
 Copyright (c) 2010.  All rights reserved.
 An Open Source Arduino based multicopter.
 
 File     : Motors.pde
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



// Send output commands to ESC´s
void motor_output()
{
  if (ch_throttle < (MIN_THROTTLE + 100))  // If throttle is low we disable yaw (neccesary to arm/disarm motors safely)
    control_yaw = 0;
   
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
  
  // Do we have cameras stabilization connected and in use?
  if(SW_DIP2) camera_output();

  
  // InstantPWM => Force inmediate output on PWM signals
  APM_RC.Force_Out0_Out1();
  APM_RC.Force_Out2_Out3();
}

