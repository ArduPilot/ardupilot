/*
 www.ArduCopter.com - www.DIYDrones.com
 Copyright (c) 2010.  All rights reserved.
 An Open Source Arduino based multicopter.
 
 File     : UserDefines.pde
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

/*************************************************************/
// Airframe
#define QUAD 0
#define HELI 1

// Note: do not change AIRFRAME to HELI and then load it into a QUAD or you will end up with your engines going to 50% during the initialisation sequence
#define AIRFRAME QUAD

/*************************************************************/
// Safety & Security 

// Arm & Disarm delays
#define ARM_DELAY 50      // how long you need to keep rudder to max right for arming motors (units*0.02, 50=1second)
#define DISARM_DELAY 25   // how long you need to keep rudder to max left for disarming motors
#define SAFETY_DELAY 25   // how long you need to keep throttle to min before safety activates and does not allow sudden throttle increases
#define SAFETY_MAX_THROTTLE_INCREASE 100  // how much of jump in throttle (within a single cycle, 5ms) will cause motors to disarm

/*************************************************************/
// AM Mode & Flight information 

/* AM PIN Definitions */
/* Will be moved in future to AN extension ports */
/* due need to have PWM pins free for sonars and servos */


#define FR_LED 3  // Mega PE4 pin, OUT7
#define RE_LED 2  // Mega PE5 pin, OUT6
#define RI_LED 7  // Mega PH4 pin, OUT5
#define LE_LED 8  // Mega PH5 pin, OUT4

/*
#define FR_LED AN12  // Mega PE4 pin, OUT7
#define RE_LED AN14  // Mega PE5 pin, OUT6
#define RI_LED AN10  // Mega PH4 pin, OUT5
#define LE_LED AN8  // Mega PH5 pin, OUT4
*/


/*************************************************************/
// Special patterns for future use

/*
#define POFF  L1\0x00\0x00\0x05
#define PALL  L1\0xFF\0xFF\0x05

#define GPS_AM_PAT1 L\0x00\0x00\0x05
#define GPS_AM_PAT2 L\0xFF\0xFF\0x05
#define GPS_AM_PAT3 L\0xF0\0xF0\0x05
*/

/* AM PIN Definitions - END */

/*************************************************************/
// Radio related definitions
#define CH_ROLL 0
#define CH_PITCH 1
#define CH_THROTTLE 2
#define CH_RUDDER 3
#define CH_1 0
#define CH_2 1
#define CH_3 2
#define CH_4 3
#define CH_5 4
#define CH_6 5
#define CH_7 6
#define CH_8 7

#define ROLL_MID 1500           // Radio Roll channel mid value
#define PITCH_MID 1500          // Radio Pitch channel mid value
#define YAW_MID 1500            // Radio Yaw channel mid value
#define THROTTLE_MID 1505       // Radio Throttle channel mid value
#define AUX_MID 1500

#define CHANN_CENTER 1500       // Channel center, legacy

                                // legacy, moved to EEPROM
//#define MIN_THROTTLE 1040       // Throttle pulse width at minimun...

/* ******************************************************** */
// Camera related settings

#define CAM_CENT  1500          // Camera center
#define CAM_SMOOTHING 1000      // Camera movement smoothing on pitch axis
#define CAM_SMOOTHING_ROLL -400 // Camera movement smoothing on roll axis

#define CAM_TILT_OUT   4        // OUTx pin for Tilt servo
#define CAM_ROLL_OUT   5        // OUTx pin for Roll servo
#define CAM_YAW_OUT    5        // OUTx pin for Yaw servo (often same as Roll)

#define CAM_TILT_CH  CH_7       // Channel for radio knob to controll tilt "zerolevel" 



/*************************************************************/
// General definitions
//Modes
#define STABLE_MODE 0
#define ACRO_MODE 1
#define AP_NORMAL_MODE 0       // AP disabled => manual flight
#define AP_AUTOMATIC_MODE 1    // AP Enabled  => Automatic mode (GPS position hold)

//Axis
#define ROLL 0
#define PITCH 1
#define YAW 2
#define XAXIS 0
#define YAXIS 1
#define ZAXIS 2

#define GYROZ 0
#define GYROX 1
#define GYROY 2
#define ACCELX 3
#define ACCELY 4
#define ACCELZ 5
#define LASTSENSOR 6

