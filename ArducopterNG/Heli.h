/*
 www.ArduCopter.com - www.DIYDrones.com
 Copyright (c) 2010.  All rights reserved.
 An Open Source Arduino based multicopter.
 
 File     : Arducopter.h
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

#ifndef HELI_H
#define HELI_H

#include <avr/interrupt.h>
#include "WProgram.h"
#include <Wire.h>
#include <EEPROM.h>   // added by Randy
#include <AP_ADC.h>                                   // ArduPilot Mega Analog to Digital Converter Library
#include <APM_RC.h>                                    // ArduPilot Mega RC Library
#include <AP_Compass.h>                               // ArduPilot Mega Compass Library
#include <DataFlash.h>                                 // ArduPilot Mega DataFlash Library.
#include "../AP_Math/AP_Math.h"

/**********************************************************************/
// Channel definitions
#define CHANNEL_FRONT_LEFT 0
#define CHANNEL_FRONT_RIGHT 1
#define CHANNEL_REAR 2
#define CHANNEL_YAW 3

/**********************************************************************/
// EEPROM locations
#define EEPROM_BASE_ADDRESS 300
#define EEPROM_MAGIC_NUMBER_ADDR EEPROM_BASE_ADDRESS
#define FRONT_LEFT_CCPM_MIN_ADDR EEPROM_BASE_ADDRESS+4
#define FRONT_LEFT_CCPM_MAX_ADDR EEPROM_BASE_ADDRESS+8
#define FRONT_RIGHT_CCPM_MIN_ADDR EEPROM_BASE_ADDRESS+12
#define FRONT_RIGHT_CCPM_MAX_ADDR EEPROM_BASE_ADDRESS+16
#define REAR_CCPM_MIN_ADDR EEPROM_BASE_ADDRESS+20
#define REAR_CCPM_MAX_ADDR EEPROM_BASE_ADDRESS+24
#define YAW_MIN_ADDR EEPROM_BASE_ADDRESS+28
#define YAW_MAX_ADDR EEPROM_BASE_ADDRESS+32
#define THROTTLE_MIN_ADDR EEPROM_BASE_ADDRESS+36
#define THROTTLE_MAX_ADDR EEPROM_BASE_ADDRESS+40

#define EEPROM_MAGIC_NUMBER 12345.0

#define YAW_MODE_HEADING_HOLD 0
#define YAW_MODE_RATE 1

#define HELI_STICK_TO_ANGLE_FACTOR 2.0   // To convert ccpm values (-50 ~ 50 ) to absolute angles.  larger number means less lean
#define HELI_YAW_STICK_TO_ANGLE_FACTOR 0.5  // convert yaw (-50 ~ 50) to turn rate in degrees per second.  larger number means slower turn rate

// CCPM Types
#define HELI_CCPM_120_TWO_FRONT_ONE_BACK 0
#define HELI_CCPM_120_ONE_FRONT_TWO_BACK 1

// define which CCPM we have
#define HELI_CCPM HELI_CCPM_120_TWO_FRONT_ONE_BACK

// define DeAllocation matrix(converts radio inputs to roll, pitch and collective
//   for example roll = (inputCh0*Row1Col1) + (inputCh1*Row1Col2) + (inputCh2*Row1Col3)
//               pitch = (inputCh0*Row2Col1) + (inputCh1*Row2Col2) + (inputCh2*Row2Col3)
//               collective = (inputCh0*Row3Col1) + (inputCh1*Row3Col2) + (inputCh2*Row3Col3)
// and Allocation matrix (converts roll, pitch, collective to servo outputs)
//   for example servo0 = (roll*Row1Col1) + (pitch*Row1Col2) + (collective*Row1Col3)
//               servo1 = (roll*Row2Col1) + (pitch*Row2Col2) + (collective*Row2Col3)
//               servo2 = (roll*Row3Col1) + (pitch*Row3Col2) + (collective*Row3Col3)
#if HELI_CCPM == HELI_CCPM_120_TWO_FRONT_ONE_BACK
  #define CCPM_DEALLOCATION   0.5774, -0.5774, 0.0000,  \
                              0.3333, 0.3333, -0.6667,  \
                              0.3333, 0.3333, 0.3333
  #define CCPM_ALLOCATION     0.8660,0.5000,  1.0000,   \
                             -0.8660, 0.5000, 1.0000,   \
                              0.0000, -1.0000, 1.0000
#endif

#if HELI_CCPM == HELI_CCPM_120_ONE_FRONT_TWO_BACK
  #define CCPM_DEALLOCATION   0.5774, -0.5774, 0.0000,  \
                              -0.3333,-0.3333, 0.6667,  \
                              0.3333, 0.3333,  0.3333
  #define CCPM_ALLOCATION     0.8660, -0.5000, 1.0000,  \
                             -0.8660, -0.5000, 1.0000,  \
                              0.0000,  1.0000, 1.0000
#endif

const Matrix3f ccpmDeallocation(CCPM_DEALLOCATION);
const Matrix3f ccpmAllocation(CCPM_ALLOCATION);

/**********************************************************************/
// time variables - we run at a different hertz than quads
unsigned long heli_previousTimeMicros = 0;

//  PWM Input Processing - Variable Definitions
float frontLeftCCPMmin;
float frontLeftCCPMmax;
float frontLeftCCPMslope;
float frontLeftCCPMintercept;

float frontRightCCPMmin;
float frontRightCCPMmax;
float frontRightCCPMslope;
float frontRightCCPMintercept;

float rearCCPMmin;
float rearCCPMmax;
float rearCCPMslope;
float rearCCPMintercept;

float yawMin;
float yawMax;
float yawSlope;
float yawIntercept;                            

Vector3f ccpmPercents;                // Array of ccpm input values, converted to percents
Vector3f rollPitchCollPercent;        // Array containing deallocated roll, pitch and collective percent commands
float ch_collective;
int collective_mid;
float control_collective;
float command_rx_collective;
float yawPercent;
float targetHeading;

// trims
float trim_roll = 0.0;
float trim_pitch = 0.0;
float trim_yaw = 0.0;

// axis under APM control
int roll_control_switch = 1;
int pitch_control_switch = 1;
int yaw_control_switch = 1;
int collective_control_switch = 0;
//int position_control_switch = 0;
//int position_control_engaged = 0;  // we don't have enough buttons so we will turn this on and off with roll + pitch positions
//int position_control_safety = 0;   // if 0 then safety is off.  if 1 then safety is on and position control will not operate

/// for sending values out to servos
Vector3f ccpmPercents_out;            // Array of ccpm input values, converted to percents
Vector3f pitchRollCollPercent_out;    // Array containing deallocated pitch, roll, and collective percent commands

// heli debug
int heli_debug = 0;

/**********************************************************************/
//  Output to Servos
int leftOut;
int rightOut;
int rearOut;
int yawOut;

#endif HELI_H
