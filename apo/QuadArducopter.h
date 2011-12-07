/*
 * QuadArducopter.h
 *
 *  Created on: May 1, 2011
 *      Author: jgoppert
 */

#ifndef QUADARDUCOPTER_H_
#define QUADARDUCOPTER_H_

using namespace apo;

// vehicle options
static const AP_Board::options_t options = AP_Board::opt_gps | AP_Board::opt_baro | AP_Board::opt_compass;
static const MAV_TYPE vehicle = MAV_QUADROTOR;
//static const apo::AP_Board::mode_e boardMode = apo::AP_Board::MODE_HIL_CNTL;
static const apo::AP_Board::mode_e boardMode = apo::AP_Board::MODE_LIVE;
static const uint8_t heartBeatTimeout = 0;

// algorithm selection
#define CONTROLLER_CLASS ControllerQuad
#define GUIDE_CLASS MavlinkGuide
#define NAVIGATOR_CLASS Navigator_Dcm

//// hardware selection
//#define BOARD_TYPE Board_APM1
//#define BOARD_TYPE Board_APM1_2560
#define BOARD_TYPE Board_APM2

// baud rates
// optional sensors
static const bool gpsEnabled = false;
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

static const bool rangeFinderFrontEnabled = false;
static const bool rangeFinderBackEnabled = false;
static const bool rangeFinderLeftEnabled = false;
static const bool rangeFinderRightEnabled = false;
static const bool rangeFinderUpEnabled = false;
static const bool rangeFinderDownEnabled = false;

// loop rates
static const float loopRate = 250; // attitude nav
static const float loop0Rate = 50; // controller
static const float loop1Rate = 10; 	// pos nav/ gcs fast
static const float loop2Rate = 1; 	// gcs slow
static const float loop3Rate = 0.1;

// position control loop
static const float PID_TILT_P = 0.1;
static const float PID_TILT_I = 0;
static const float PID_TILT_D = 0.1;
static const float PID_TILT_LIM = 0.04; // about 2 deg
static const float PID_TILT_AWU = 0.02; // about 1 deg
static const float PID_TILT_DFCUT = 10; // cut derivative feedback at 10 hz

static const float PID_SPD_P = 1;
static const float PID_SPD_I = 0;
static const float PID_SPD_D = 0.1;
static const float PID_SPD_LIM = 0.04; // about 2 deg
static const float PID_SPD_AWU = 0.02; // about 1 deg
static const float PID_SPD_DFCUT = 10; // cut derivative feedback at 10 hz

static const float PID_POS_Z_P = 0.1;
static const float PID_POS_Z_I = 0;
static const float PID_POS_Z_D = 0.2;
static const float PID_POS_Z_LIM = 0.1;
static const float PID_POS_Z_AWU = 0;
static const float PID_POS_Z_DFCUT = 10; // cut derivative feedback at 10 hz

// attitude control loop
static const float PID_ATT_P = 0.17;
static const float PID_ATT_I = 0.5;
static const float PID_ATT_D = 0.06;
static const float PID_ATT_LIM = 0.05; // 5 % motors
static const float PID_ATT_AWU = 0.005; // 0.5 %
static const float PID_ATT_DFCUT = 25; // cut derivative feedback at 25 hz
static const float PID_YAWPOS_P = 1;
static const float PID_YAWPOS_I = 0;
static const float PID_YAWPOS_D = 0.1;
static const float PID_YAWPOS_LIM = 1; // 1 rad/s
static const float PID_YAWPOS_AWU = 0; // 1 rad/s
static const float PID_YAWSPEED_P = 0.5;
static const float PID_YAWSPEED_I = 0;
static const float PID_YAWSPEED_D = 0;
static const float PID_YAWSPEED_LIM = .05; // 5 % motors
static const float PID_YAWSPEED_AWU = 0.0;
static const float PID_YAWSPEED_DFCUT = 3.0; // 3 Radians, about 1 Hz
static const float THRUST_HOVER_OFFSET = 0.475;

// guidance
static const float velCmd = 0.3; // m/s
static const float xt = 10; // cross track gain
static const float xtLim = 90; // cross track angle limit, deg

#include "ControllerQuad.h"

#endif /* QUADARDUCOPTER_H_ */
// vim:ts=4:sw=4:expandtab
