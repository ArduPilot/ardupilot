#ifndef APO_Config_H
#define APO_Config_H

#include "AP_HardwareAbstractionLayer.h"
#include "mikrokopter.h"
#include "constants.h"

// Serial 0: debug      /dev/ttyUSB0
// Serial 1: gps/hil    /dev/ttyUSB1
// Serial 2: gcs        /dev/ttyUSB2

// select hardware absraction mode from
// 	MODE_LIVE, actual flight
// 	TODO: IMPLEMENT --> MODE_HIL_NAV, hardware in the loop with sensors running, tests navigation system and control
// 	MODE_HIL_CNTL, hardware in the loop with only controller running, just tests controller
extern apo::halMode_t halMode;

// select from, BOARD_ARDUPILOTMEGA
extern apo::board_t board;

// select from, VEHICLE_CAR, VEHICLE_QUAD, VEHICLE_PLANE
extern apo::vehicle_t vehicle;

//---------ADVANCED SECTION ----------------//

// loop rates
extern const float loop0Rate;
extern const float loop1Rate;
extern const float loop2Rate;
extern const float loop3Rate;
extern const float loop4Rate;

// max time in seconds to allow flight without ground station comms
// zero will ignore timeout
extern const uint8_t heartbeatTimeout;

//---------HARDWARE CONFIG ----------------//

//Hardware Parameters
#define SLIDE_SWITCH_PIN 40
#define PUSHBUTTON_PIN 41
#define A_LED_PIN 37 //36 = B,3637 = A,363735 = C
#define B_LED_PIN 36
#define C_LED_PIN 35
#define EEPROM_MAX_ADDR	2048
#define RANGE_FINDER_CLASS AP_RangeFinder_MaxsonarLV


#endif
// vim:ts=4:sw=4:expandtab
