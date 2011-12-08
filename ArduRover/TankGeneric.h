/*
 * TankGeneric.h
 *
 *  Created on: Sep 26, 2011
 *      Author: jgoppert
 */

// NOT CURRENTLY WORKING

#ifndef TANKGENERIC_H_
#define TANKGENERIC_H_

// vehicle options
static const AP_Board::options_t options = AP_Board::opt_gps | AP_Board::opt_baro | AP_Board::opt_compass;
static const MAV_TYPE vehicle = UGV_GROUND_ROVER;
//static const apo::AP_Board::mode_e = apo::AP_Board::MODE_HIL_CNTL;
static const apo::AP_Board::mode_e = apo::AP_Board::MODE_LIVE;
static const uint8_t heartBeatTimeout = 0;

// algorithm selection
#define CONTROLLER_CLASS ControllerTank
#define GUIDE_CLASS MavlinkGuide
#define NAVIGATOR_CLASS Navigator_Dcm

// hardware selection
//#define BOARD_TYPE Board_APM1
//#define BOARD_TYPE Board_APM1_2560
#define BOARD_TYPE Board_APM2

// loop rates
static const float loopRate = 150; // attitude nav
static const float loop0Rate = 50; // controller
static const float loop1Rate = 5; 	// pos nav/ gcs fast
static const float loop2Rate = 1; 	// gcs slow
static const float loop3Rate = 0.1;

// gains
const float steeringP = 1.0;
const float steeringI = 0.0;
const float steeringD = 0.0;
const float steeringIMax = 0.0;
const float steeringYMax = 3.0;
const float steeringDFCut = 25;

const float throttleP = 0.0;
const float throttleI = 0.0;
const float throttleD = 0.0;
const float throttleIMax = 0.0;
const float throttleYMax = 0.0;
const float throttleDFCut = 3.0;

// guidance
static const float velCmd = 5;
static const float xt = 10;
static const float xtLim = 90;

#include "ControllerTank.h"

#endif /* TANKGENERIC_H_ */
// vim:ts=4:sw=4:expandtab
