/*
 * PlaneEasystar.h
 *
 *  Created on: May 1, 2011
 *      Author: jgoppert
 */

#ifndef PLANEEASYSTAR_H_
#define PLANEEASYSTAR_H_

using namespace apo;

// vehicle options
static const AP_Board::options_t options = AP_Board::opt_gps | AP_Board::opt_baro | AP_Board::opt_compass;
static const MAV_TYPE vehicle = MAV_TYPE_FIXED_WING;
//static const apo::AP_Board::mode_e = apo::AP_Board::MODE_HIL_CNTL;
static const apo::AP_Board::mode_e = apo::AP_Board::MODE_LIVE;
static const uint8_t heartBeatTimeout = 0;

// algorithm selection
#define CONTROLLER_CLASS ControllerPlane
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
static const float rdrAilMix = 1.0; // since there are no ailerons

// bank error to roll servo
static const float pidBnkRllP = -1;
static const float pidBnkRllI = 0.0;
static const float pidBnkRllD = 0.0;
static const float pidBnkRllAwu = 0.0;
static const float pidBnkRllLim = 1.0;
static const float pidBnkRllDFCut = 0.0;

// pitch error to pitch servo
static const float pidPitPitP = -1;
static const float pidPitPitI = 0.0;
static const float pidPitPitD = 0.0;
static const float pidPitPitAwu = 0.0;
static const float pidPitPitLim = 1.0;
static const float pidPitPitDFCut = 0.0;

// speed error to pitch command
static const float pidSpdPitP = 0.1;
static const float pidSpdPitI = 0.0;
static const float pidSpdPitD = 0.0;
static const float pidSpdPitAwu = 0.0;
static const float pidSpdPitLim = 1.0;
static const float pidSpdPitDFCut = 0.0;

// yaw rate error to yaw servo
static const float pidYwrYawP = -0.1;
static const float pidYwrYawI = 0.0;
static const float pidYwrYawD = 0.0;
static const float pidYwrYawAwu = 0.0;
static const float pidYwrYawLim = 1.0;
static const float pidYwrYawDFCut = 0.0;

// heading error to bank angle command
static const float pidHdgBnkP = 1.0;
static const float pidHdgBnkI = 0.0;
static const float pidHdgBnkD = 0.0;
static const float pidHdgBnkAwu = 0.0;
static const float pidHdgBnkLim = 0.5;
static const float pidHdgBnkDFCut = 0.0;

// altitude error to throttle command
static const float pidAltThrP = .01;
static const float pidAltThrI = 0.0;
static const float pidAltThrD = 0.0;
static const float pidAltThrAwu = 0.0;
static const float pidAltThrLim = 1;
static const float pidAltThrDFCut = 0.0;

// trim control positions (-1,1)
static const float ailTrim = 0.0;
static const float elvTrim = 0.0;
static const float rdrTrim = 0.0;
static const float thrTrim = 0.5;

// guidance
static const float velCmd = 1; // m/s
static const float xt = 10; // cross track gain
static const float xtLim = 90; // cross track angle limit, deg

#include "ControllerPlane.h"

#endif /* PLANEEASYSTAR_H_ */
// vim:ts=4:sw=4:expandtab
