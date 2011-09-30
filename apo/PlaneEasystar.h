/*
 * PlaneEasystar.h
 *
 *  Created on: May 1, 2011
 *      Author: jgoppert
 */

#ifndef PLANEEASYSTAR_H_
#define PLANEEASYSTAR_H_


// vehicle options
static const apo::vehicle_t vehicle = apo::VEHICLE_PLANE;
//static const apo::halMode_t halMode = apo::MODE_LIVE; // live mode, actual flight
static const apo::halMode_t halMode = apo::MODE_HIL_CNTL; // hardware in the loop, control level
static const apo::board_t board = apo::BOARD_ARDUPILOTMEGA_1280;
static const uint8_t heartBeatTimeout = 3;

// algorithm selection
#define CONTROLLER_CLASS ControllerPlane
#define GUIDE_CLASS MavlinkGuide
#define NAVIGATOR_CLASS DcmNavigator
#define COMMLINK_CLASS MavlinkComm

// hardware selection
#define ADC_CLASS AP_ADC_ADS7844
#define COMPASS_CLASS AP_Compass_HMC5843
#define BARO_CLASS APM_BMP085_Class
#define RANGE_FINDER_CLASS AP_RangeFinder_MaxsonarXL
#define DEBUG_BAUD 57600
#define TELEM_BAUD 57600
#define GPS_BAUD 38400
#define HIL_BAUD 57600

// optional sensors
static bool gpsEnabled = false;
static bool baroEnabled = true;
static bool compassEnabled = true;

static bool rangeFinderFrontEnabled = true;
static bool rangeFinderBackEnabled = true;
static bool rangeFinderLeftEnabled = true;
static bool rangeFinderRightEnabled = true;
static bool rangeFinderUpEnabled = true;
static bool rangeFinderDownEnabled = true;

// loop rates
static const float loop0Rate = 150;
static const float loop1Rate = 100;
static const float loop2Rate = 10;
static const float loop3Rate = 1;
static const float loop4Rate = 0.1;

// gains
static const float rdrAilMix = 1.0; // since there are no ailerons

// bank error to roll servo
static const float pidBnkRllP = -0.5;
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
static const float pidYwrYawP = -0.2;
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

#include "ControllerPlane.h"

#endif /* PLANEEASYSTAR_H_ */
