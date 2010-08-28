/*
 www.ArduCopter.com - www.DIYDrones.com
 Copyright (c) 2010.  All rights reserved.
 An Open Source Arduino based multicopter.
 
 File     : Attitude.pde
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

/* ************************************************************ */

////////////////////////////////////////////////// 
// Function  : Attitude_control_v2()
//
//             Stable flight mode main algoritms
//
// Parameters: 
//     - none
//
// Returns   : - none
//
// Alters    : 
//  err_roll, roll_rate
//
// Relies    :
//  Radio input, Gyro
//

// STABLE MODE
// ROLL, PITCH and YAW PID controls... 
// Input : desired Roll, Pitch and Yaw absolute angles. Output : Motor commands
void Attitude_control_v2()
{
  float err_roll_rate;
  float err_pitch_rate;
  float roll_rate;
  float pitch_rate;

  // ROLL CONTROL    
  if (AP_mode == 2)        // Normal Mode => Stabilization mode
    err_roll = command_rx_roll - ToDeg(roll);
  else
    err_roll = (command_rx_roll + command_gps_roll) - ToDeg(roll);  // Position control

    err_roll = constrain(err_roll, -25, 25);  // to limit max roll command...

  // New control term...
  roll_rate = ToDeg(Omega[0]);  // Omega[] is the raw gyro reading plus Omega_I, so it´s bias corrected
  err_roll_rate = ((ch_roll - roll_mid) >> 1) - roll_rate;

  roll_I += err_roll * G_Dt;
  roll_I = constrain(roll_I, -20, 20);
  // D term implementation => two parts: gyro part and command part
  // To have a better (faster) response we can use the Gyro reading directly for the Derivative term...
  // We also add a part that takes into account the command from user (stick) to make the system more responsive to user inputs
  roll_D = - roll_rate;  // Take into account Angular velocity of the stick (command)

  // PID control
  K_aux = KP_QUAD_ROLL; // Comment this out if you want to use transmitter to adjust gain
  control_roll = K_aux * err_roll + KD_QUAD_ROLL * roll_D + KI_QUAD_ROLL * roll_I + STABLE_MODE_KP_RATE * err_roll_rate;  

  // PITCH CONTROL
  if (AP_mode==2)        // Normal mode => Stabilization mode
    err_pitch = command_rx_pitch - ToDeg(pitch);
  else
    err_pitch = (command_rx_pitch + command_gps_pitch) - ToDeg(pitch);  // Position Control

  err_pitch = constrain(err_pitch, -25, 25);  // to limit max pitch command...

  // New control term...
  pitch_rate = ToDeg(Omega[1]);
  err_pitch_rate = ((ch_pitch - pitch_mid) >> 1) - pitch_rate;

  pitch_I += err_pitch * G_Dt;
  pitch_I = constrain(pitch_I, -20, 20);
  // D term
  pitch_D = - pitch_rate;

  // PID control
  K_aux = KP_QUAD_PITCH; // Comment this out if you want to use transmitter to adjust gain
  control_pitch = K_aux * err_pitch + KD_QUAD_PITCH * pitch_D + KI_QUAD_PITCH * pitch_I + STABLE_MODE_KP_RATE * err_pitch_rate; 

  // YAW CONTROL
  err_yaw = command_rx_yaw - ToDeg(yaw);
  if (err_yaw > 180)    // Normalize to -180,180
      err_yaw -= 360;
  else if(err_yaw < -180)
    err_yaw += 360;

  err_yaw = constrain(err_yaw, -60, 60);  // to limit max yaw command...

  yaw_I += err_yaw * G_Dt;
  yaw_I = constrain(yaw_I, -20, 20);
  yaw_D = - ToDeg(Omega[2]);

  // PID control
  control_yaw = KP_QUAD_YAW * err_yaw + KD_QUAD_YAW * yaw_D + KI_QUAD_YAW * yaw_I;
}


////////////////////////////////////////////////// 
// Function  : Rate_control()
//
//             Acro mode main algoritms
//
// Parameters: 
//     - none
//
// Returns   : - none
//
// Alters    : 
//  err_roll, roll_rate
//
// Relies    :
//  Radio input, Gyro
//


// ACRO MODE
void Rate_control()
{
  static float previousRollRate, previousPitchRate, previousYawRate;
  float currentRollRate, currentPitchRate, currentYawRate;

  // ROLL CONTROL
  currentRollRate = read_adc(0);      // I need a positive sign here

  err_roll = ((ch_roll - roll_mid) * xmitFactor) - currentRollRate;

  roll_I += err_roll * G_Dt;
  roll_I = constrain(roll_I, -20, 20);

  roll_D = currentRollRate - previousRollRate;
  previousRollRate = currentRollRate;

  // PID control
  control_roll = Kp_RateRoll * err_roll + Kd_RateRoll * roll_D + Ki_RateRoll * roll_I; 

  // PITCH CONTROL
  currentPitchRate = read_adc(1);
  err_pitch = ((ch_pitch - pitch_mid) * xmitFactor) - currentPitchRate;

  pitch_I += err_pitch*G_Dt;
  pitch_I = constrain(pitch_I,-20,20);

  pitch_D = currentPitchRate - previousPitchRate;
  previousPitchRate = currentPitchRate;

  // PID control
  control_pitch = Kp_RatePitch*err_pitch + Kd_RatePitch*pitch_D + Ki_RatePitch*pitch_I; 

  // YAW CONTROL
  currentYawRate = read_adc(2);
  err_yaw = ((ch_yaw - yaw_mid) * xmitFactor) - currentYawRate;

  yaw_I += err_yaw*G_Dt;
  yaw_I = constrain(yaw_I, -20, 20);

  yaw_D = currentYawRate - previousYawRate;
  previousYawRate = currentYawRate;

  // PID control
  K_aux = KP_QUAD_YAW; // Comment this out if you want to use transmitter to adjust gain
  control_yaw = Kp_RateYaw*err_yaw + Kd_RateYaw*yaw_D + Ki_RateYaw*yaw_I; 
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
    if (diff_ch_old <- 40)
      return(ch_old - 40);        // We limit the max difference between readings
  }
  else
  {
    if (diff_ch_old > 40)    
      return(ch_old + 40);
  }
  return((ch + ch_old) >> 1);   // Small filtering
  //return(ch);
}
