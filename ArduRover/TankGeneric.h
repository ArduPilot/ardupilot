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
static const apo::vehicle_t vehicle = apo::VEHICLE_TANK;
static const apo::halMode_t halMode = apo::MODE_LIVE;
static const apo::board_t board = apo::BOARD_ARDUPILOTMEGA_1280;
static const uint8_t heartBeatTimeout = 3;

// algorithm selection
#define CONTROLLER_CLASS ControllerTank
#define GUIDE_CLASS MavlinkGuide
#define NAVIGATOR_CLASS DcmNavigator
#define COMMLINK_CLASS MavlinkComm

// hardware selection
#define ADC_CLASS AP_ADC_ADS7844
#define COMPASS_CLASS AP_Compass_HMC5843
#define BARO_CLASS APM_BMP085_Class
#define RANGE_FINDER_CLASS AP_RangeFinder_MaxsonarXL

// baud rates
static const uint32_t debugBaud = 57600;
static const uint32_t telemBaud = 57600;
static const uint32_t gpsBaud = 38400;
static const uint32_t hilBaud = 57600;

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
