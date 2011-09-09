/*
 www.ArduCopter.com - www.DIYDrones.com
 Copyright (c) 2010.  All rights reserved.
 An Open Source Arduino based multicopter.
 
 File     : Events.pde
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
  31-10-10 Moved camera stabilization from Functions to here, jp

* ************************************************************** *
TODO:


* ************************************************************** */






/* **************************************************** */
// Camera stabilization
//
// Stabilization for three different camera styles
// 1) Camera mounts that have tilt / pan
// 2) Camera mounts that have tilt / roll
// 3) Camera mounts that have tilt / roll / pan (future)
//
// Original code idea from Max Levine / DIY Drones
// You need to initialize proper camera mode by sending Serial command and then save it
// to EEPROM memory. Possible commands are K1, K2, K3, K4
// Look more about different camera type on ArduCopter Wiki

#ifdef IsCAM
void camera_output() {

  // cam_mode = 1;                        // for debugging 
  // Camera stabilization jump tables
  // SW_DIP1 is a multplier, settings  
  switch ((SW_DIP1 * 4) + cam_mode + (BATTLOW * 10)) {
    // Cases 1 & 4 are stabilization for + Mode flying setup
    // Cases 5 & 8 are stabilization for x Mode flying setup
    
    // Modes 3/4 + 7/8 needs still proper scaling on yaw movement
    // Scaling needs physical test flying with FPV cameras on, 30-10-10 jp

  case 1:
    // Camera stabilization with Roll / Tilt mounts, NO transmitter input
    APM_RC.OutputCh(CAM_TILT_OUT, limitRange((CAM_CENT + (pitch) * CAM_SMOOTHING), 1000, 2000));                            // Tilt correction 
    APM_RC.OutputCh(CAM_ROLL_OUT, limitRange((CAM_CENT + (roll) * CAM_SMOOTHING_ROLL), 1000, 2000));                        // Roll correction
    break;
  case 2:
    // Camera stabilization with Roll / Tilt mounts, transmitter controls basic "zerolevel"
    APM_RC.OutputCh(CAM_TILT_OUT, limitRange((APM_RC.InputCh(CAM_TILT_CH) + (pitch) * CAM_SMOOTHING), 1000, 2000));         // Tilt correction 
    APM_RC.OutputCh(CAM_ROLL_OUT, limitRange((CAM_CENT + (roll) * CAM_SMOOTHING_ROLL), 1000, 2000));                        // Roll correction
    break;
  case 3:
    // Camera stabilization with Yaw / Tilt mounts, NO transmitter input
    APM_RC.OutputCh(CAM_TILT_OUT, limitRange((CAM_CENT - (roll - pitch) * CAM_SMOOTHING), 1000, 2000));                     // Tilt correction 
    APM_RC.OutputCh(CAM_YAW_OUT, limitRange((CAM_CENT - (gyro_offset_yaw - AN[2])), 1000, 2000));                           // Roll correction
    break;
  case 4:
    // Camera stabilization with Yaw / Tilt mounts, transmitter controls basic "zerolevel"
    APM_RC.OutputCh(CAM_TILT_OUT, limitRange((APM_RC.InputCh(CAM_TILT_CH) + (pitch) * CAM_SMOOTHING), 1000, 2000));         // Tilt correction 
    APM_RC.OutputCh(CAM_YAW_OUT, limitRange((CAM_CENT - (gyro_offset_yaw - AN[2])), 1000, 2000));                           // Roll correction
    break;

    // x Mode flying setup  
  case 5:
    // Camera stabilization with Roll / Tilt mounts, NO transmitter input
    APM_RC.OutputCh(CAM_TILT_OUT, limitRange((CAM_CENT - (roll - pitch) * CAM_SMOOTHING), 1000, 2000));                      // Tilt correction 
    APM_RC.OutputCh(CAM_ROLL_OUT, limitRange((CAM_CENT + (roll + pitch) * CAM_SMOOTHING), 1000, 2000));                      // Roll correction
    break;
  case 6:
    // Camera stabilization with Roll / Tilt mounts, transmitter controls basic "zerolevel"
    APM_RC.OutputCh(CAM_TILT_OUT, limitRange((APM_RC.InputCh(CAM_TILT_CH) + (roll - pitch) * CAM_SMOOTHING), 1000, 2000));   // Tilt correction 
    APM_RC.OutputCh(CAM_ROLL_OUT, limitRange((CAM_CENT + (roll + pitch) * CAM_SMOOTHING), 1000, 2000));                      // Roll correction
    break;
  case 7:
    // Camera stabilization with Yaw / Tilt mounts, NO transmitter input
    APM_RC.OutputCh(CAM_TILT_OUT, limitRange((CAM_CENT - (roll - pitch) * CAM_SMOOTHING), 1000, 2000));                      // Tilt correction 
    APM_RC.OutputCh(CAM_YAW_OUT, limitRange((CAM_CENT - (gyro_offset_yaw - AN[2])), 1000, 2000));                            // Roll correction
    break;
  case 8:
    // Camera stabilization with Yaw / Tilt mounts, transmitter controls basic "zerolevel"
    APM_RC.OutputCh(CAM_TILT_OUT, limitRange((APM_RC.InputCh(CAM_TILT_CH) - (roll - pitch) * CAM_SMOOTHING),1000,2000));   // Tilt correction 
    APM_RC.OutputCh(CAM_YAW_OUT, limitRange((CAM_CENT - (gyro_offset_yaw - AN[2])),1000,2000));                            // Yaw correction
    break;

  // Only in case of we have switch values over 10
  default:
    // We should not be here...
    break;
  }
}
#endif


