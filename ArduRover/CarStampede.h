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
static const apo::vehicle_t vehicle = apo::VEHICLE_CAR;
static const apo::halMode_t halMode = apo::MODE_LIVE;
static const apo::board_t board = apo::BOARD_ARDUPILOTMEGA_2560;
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
static const float steeringP = 1.0;
static const float steeringI = 0.0;
static const float steeringD = 0.0;
static const float steeringIMax = 0.0;
static const float steeringYMax = 3.0;

static const float throttleP = 0.0;
static const float throttleI = 0.0;
static const float throttleD = 0.0;
static const float throttleIMax = 0.0;
static const float throttleYMax = 0.0;
static const float throttleDFCut = 3.0;

#include "ControllerCar.h"

#endif /* CARSTAMPEDE_H_ */
