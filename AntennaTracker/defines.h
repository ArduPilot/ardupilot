#pragma once

#include "include/mavlink/v2.0/ardupilotmega/mavlink.h"

// Command/Waypoint/Location Options Bitmask
//--------------------
#define MASK_OPTIONS_RELATIVE_ALT       (1<<0)          // 1 = Relative
                                                        // altitude

// Controller modes
// ----------------

// map flight modes to the MAVLink representation
enum ControlMode {
    MANUAL       = TRACKER_FLIGHT_MODE_MANUAL,
    STOP         = TRACKER_FLIGHT_MODE_STOP,
    SCAN         = TRACKER_FLIGHT_MODE_SCAN,
    SERVO_TEST   = TRACKER_FLIGHT_MODE_SERVO_TEST,
    AUTO         = TRACKER_FLIGHT_MODE_AUTO,
    INITIALISING = TRACKER_FLIGHT_MODE_INITIALISING
};

enum ServoType {
    SERVO_TYPE_POSITION=0,
    SERVO_TYPE_ONOFF=1,
    SERVO_TYPE_CR=2
};

enum AltSource {
	ALT_SOURCE_BARO=0,
	ALT_SOURCE_GPS=1,
	ALT_SOURCE_GPS_VEH_ONLY=2
};

//  Filter
#define SERVO_OUT_FILT_HZ               0.1f
#define G_Dt                            0.02f

//  Logging parameters
#define MASK_LOG_ATTITUDE               (1<<0)
#define MASK_LOG_GPS                    (1<<1)
#define MASK_LOG_RCIN                   (1<<2)
#define MASK_LOG_IMU                    (1<<3)
#define MASK_LOG_RCOUT                  (1<<4)
#define MASK_LOG_COMPASS                (1<<5)
#define MASK_LOG_ANY                    0xFFFF

//  Logging messages
#define LOG_V_BAR_MSG                   0x04
#define LOG_V_POS_MSG                   0x05
