/*
 * SailboatLaser.h
 *
 *  Created on: May 1, 2011
 *      Author: jgoppert
 *
 */

#ifndef SAILBOATLASER_H_
#define SAILBOATLASER_H_

using namespace apo;

// vehicle options
static const AP_Board::options_t options = AP_Board::opt_gps | AP_Board::opt_baro | AP_Board::opt_compass;
static const MAV_TYPE vehicle = UGV_SURFACE_SHIP; 
static const apo::AP_Board::mode_e boardMode = apo::AP_Board::MODE_HIL_CNTL;
//static const apo::AP_Board::mode_e boardMode = apo::AP_Board::MODE_LIVE;
static const uint8_t heartBeatTimeout = 0;

// algorithm selection
#define CONTROLLER_CLASS ControllerSailboat
#define GUIDE_CLASS MavlinkGuide
#define NAVIGATOR_CLASS Navigator_Dcm

// hardware selection
#define BOARD_TYPE Board_APM1
//#define BOARD_TYPE Board_APM1_2560
//#define BOARD_TYPE Board_APM2

// loop rates
static const float loopRate = 150; // attitude nav
static const float loop0Rate = 50; // controller
static const float loop1Rate = 5; 	// pos nav/ gcs fast
static const float loop2Rate = 1; 	// gcs slow
static const float loop3Rate = 0.1;

// gains
static const float steeringP = 0.1;
static const float steeringI = 0.0;
static const float steeringD = 0.1;
static const float steeringIMax = 0.0;
static const float steeringYMax = 1;
static const float steeringDFCut = 25.0;

static const float throttleP = 0.1;
static const float throttleI = 0.0;
static const float throttleD = 0.2;
static const float throttleIMax = 0.0;
static const float throttleYMax = 1;
static const float throttleDFCut = 0.0;

// guidance
static const float velCmd = 2;
static const float xt = 10;
static const float xtLim = 90;

#include "ControllerSailboat.h"

#endif /* SAILBOATLASER_H_ */
// vim:ts=4:sw=4:expandtab
