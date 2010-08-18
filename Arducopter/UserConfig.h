/*
  ArduCopter v1.3 - Aug 2010
 www.ArduCopter.com
 Copyright (c) 2010.  All rights reserved.
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
 
/* *************************************************************
TODO:

  - move all user definable variables from main pde to here
  - comment variables properly


************************************************************* */


/*************************************************************/
// Safety & Security 

// Arm & Disarm delays
#define ARM_DELAY 200
#define DISARM_DELAY 100


/*************************************************************/
// AM Mode & Flight information 

/* AM PIN Definitions */
/* Will be moved in future to AN extension ports */
/* due need to have PWM pins free for sonars and servos */

#define FR_LED 3  // Mega PE4 pin, OUT7
#define RE_LED 2  // Mega PE5 pin, OUT6
#define RI_LED 7  // Mega PH4 pin, OUT5
#define LE_LED 8  // Mega PH5 pin, OUT4

/* AM PIN Definitions - END */


/*************************************************************/
// Radio related definitions

// If you don't know these values, you can activate RADIO_TEST_MODE below
// and check your mid values

//#define RADIO_TEST_MODE

#define ROLL_MID 1478    // Radio Roll channel mid value
#define PITCH_MID 1483    // Radio Pitch channel mid value
#define YAW_MID 1500    // Radio Yaw channel mid value
#define THROTTLE_MID 1502    // Radio Throttle channel mid value
#define AUX_MID 1500

#define CHANN_CENTER 1500       // Channel center, legacy
#define MIN_THROTTLE 1040       // Throttle pulse width at minimun...

