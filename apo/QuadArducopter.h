/*
 * QuadArducopter.h
 *
 *  Created on: May 1, 2011
 *      Author: jgoppert
 */

#ifndef QUADARDUCOPTER_H_
#define QUADARDUCOPTER_H_

// vehicle options
static const apo::vehicle_t vehicle = apo::VEHICLE_QUAD;
static const apo::halMode_t halMode = apo::MODE_LIVE;
static const apo::board_t board = apo::BOARD_ARDUPILOTMEGA_1280;
static const uint8_t heartBeatTimeout = 3;

// algorithm selection
#define CONTROLLER_CLASS ControllerQuad
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
static const float loopRate = 400; // attitude nav
static const float loop0Rate = 50; // controller
static const float loop1Rate = 10; 	// pos nav/ gcs fast
static const float loop2Rate = 1; 	// gcs slow
static const float loop3Rate = 0.1;

// position control loop
static const float PID_POS_P = 0;
static const float PID_POS_I = 0;
static const float PID_POS_D = 0;
static const float PID_POS_LIM = 0; // about 5 deg
static const float PID_POS_AWU = 0; // about 5 deg
static const float PID_POS_Z_P = 0;
static const float PID_POS_Z_I = 0;
static const float PID_POS_Z_D = 0;
static const float PID_POS_Z_LIM = 0;
static const float PID_POS_Z_AWU = 0;
static const float PID_POS_DFCUT = 10; // cut derivative feedback at 10 hz

// attitude control loop
static const float PID_ATT_P = 0.17;
static const float PID_ATT_I = 0.5;
static const float PID_ATT_D = 0.06;
static const float PID_ATT_LIM = 0.05; // 5 % motors
static const float PID_ATT_AWU = 0.005; // 0.5 %
static const float PID_ATT_DFCUT = 25; // cut derivative feedback at 25 hz
static const float PID_YAWPOS_P = 0;
static const float PID_YAWPOS_I = 0;
static const float PID_YAWPOS_D = 0;
static const float PID_YAWPOS_LIM = 0; // 1 rad/s
static const float PID_YAWPOS_AWU = 0; // 1 rad/s
static const float PID_YAWSPEED_P = 0.5;
static const float PID_YAWSPEED_I = 0;
static const float PID_YAWSPEED_D = 0;
static const float PID_YAWSPEED_LIM = .05; // 5 % motors
static const float PID_YAWSPEED_AWU = 0.0;
static const float PID_YAWSPEED_DFCUT = 3.0; // 3 Radians, about 1 Hz

static const float THRUST_HOVER_OFFSET = 0.475;

#include "ControllerQuad.h"

#endif /* QUADARDUCOPTER_H_ */
