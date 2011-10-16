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
static const apo::halMode_t halMode = apo::MODE_LIVE;
static const apo::board_t board = apo::BOARD_ARDUPILOTMEGA_2560;
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

// baud rates
static uint32_t debugBaud = 57600; 
static uint32_t telemBaud = 57600; 
static uint32_t gpsBaud = 38400; 
static uint32_t hilBaud = 57600; 

// optional sensors
static const bool gpsEnabled = false;
static const bool baroEnabled = false;
static const bool compassEnabled = true;
static const Matrix3f compassOrientation = AP_COMPASS_COMPONENTS_UP_PINS_FORWARD;
// compass orientation: See AP_Compass_HMC5843.h for possible values

// battery monitoring
static const bool batteryMonitorEnabled = true;
static const uint8_t batteryPin = 0;
static const float batteryVoltageDivRatio = 6;
static const float batteryMinVolt = 10.0;
static const float batteryMaxVolt = 12.4;

static const bool rangeFinderFrontEnabled = false;
static const bool rangeFinderBackEnabled = false;
static const bool rangeFinderLeftEnabled = false;
static const bool rangeFinderRightEnabled = false;
static const bool rangeFinderUpEnabled = false;
static const bool rangeFinderDownEnabled = false;

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

#include "ControllerPlane.h"

#endif /* PLANEEASYSTAR_H_ */
