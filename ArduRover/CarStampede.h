/*
 * CarStampede.h
 *
 *  Created on: May 1, 2011
 *      Author: jgoppert
 *
 */

#ifndef CARSTAMPEDE_H_
#define CARSTAMPEDE_H_

// vehicle options
static const MAV_TYPE vehicle = UGV_GROUND_ROVER;
//static const apo::halMode_t halMode = apo::MODE_HIL_CNTL;
static const apo::halMode_t halMode = apo::MODE_LIVE;
static const apo::board_t board = apo::BOARD_ARDUPILOTMEGA_1280;
static const uint8_t heartBeatTimeout = 3;

// algorithm selection
#define CONTROLLER_CLASS ControllerCar
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
static const uint32_t hilBaud = 115200;

// optional sensors
static const bool gpsEnabled = true;
static const bool baroEnabled = true;
static const bool compassEnabled = true;
static const Matrix3f compassOrientation = AP_COMPASS_COMPONENTS_UP_PINS_FORWARD;
// compass orientation: See AP_Compass_HMC5843.h for possible values

// battery monitoring
static const bool batteryMonitorEnabled = false;
static const uint8_t batteryPin = 0;
static const float batteryVoltageDivRatio = 6;
static const float batteryMinVolt = 10.0;
static const float batteryMaxVolt = 12.4;

static const bool useForwardReverseSwitch = false;

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
static const float steeringP = 0.1;
static const float steeringI = 0.0;
static const float steeringD = 0.1;
static const float steeringIMax = 0.0;
static const float steeringYMax = 1;
static const float steeringDFCut = 25.0;

static const float throttleP = 0.1;
static const float throttleI = 0.0;
static const float throttleD = 0.0;
static const float throttleIMax = 0.0;
static const float throttleYMax = 1;
static const float throttleDFCut = 0.0;

// guidance
static const float velCmd = 5;
static const float xt = 10;
static const float xtLim = 90;

#include "ControllerCar.h"

#endif /* CARSTAMPEDE_H_ */
// vim:ts=4:sw=4:expandtab
