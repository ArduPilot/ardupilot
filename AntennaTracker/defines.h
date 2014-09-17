// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef _DEFINES_H
#define _DEFINES_H

#define ToRad(x) radians(x)	// *pi/180
#define ToDeg(x) degrees(x)	// *180/pi

#define EEPROM_MAX_ADDR         4096

// mark a function as not to be inlined
#define NOINLINE __attribute__((noinline))

// InertialSensor driver types
#define CONFIG_INS_OILPAN  1
#define CONFIG_INS_MPU6000 2
#define CONFIG_INS_HIL     3
#define CONFIG_INS_PX4     4
#define CONFIG_INS_FLYMAPLE 5
#define CONFIG_INS_L3G4200D 6

// barometer driver types
#define AP_BARO_BMP085   1
#define AP_BARO_MS5611   2
#define AP_BARO_PX4      3
#define AP_BARO_HIL      4

// compass driver types
#define AP_COMPASS_HMC5843   1
#define AP_COMPASS_PX4       2
#define AP_COMPASS_HIL       3

// In AntennaTracker, EEPROM is used to store waypoint 0 = HOME location, if available
// EEPROM addresses
#define EEPROM_MAX_ADDR         4096
// parameters get the first 1280 bytes of EEPROM, remainder is for waypoints
#define WP_START_BYTE 0x500 // where in memory home WP is stored + all other
                            // WP
#define WP_SIZE 15

// Commands - Note that APM now uses a subset of the MAVLink protocol commands.  See enum MAV_CMD in the GCS_Mavlink library
#define CMD_BLANK 0 // there is no command stored in the mem location requested
#define NO_COMMAND 0
#define WAIT_COMMAND 255

// Command/Waypoint/Location Options Bitmask
//--------------------
#define MASK_OPTIONS_RELATIVE_ALT       (1<<0)          // 1 = Relative
                                                        // altitude
#define MASK_OPTIONS_LOITER_DIRECTION   (1<<2)          // 0 = CW
                                                        // 1 = CCW

// Controller modes
// ----------------

enum ControlMode {
    MANUAL=0,
    STOP=1,
    SCAN=2,
    AUTO=10,
    INITIALISING=16
};

enum ServoType {
    SERVO_TYPE_POSITION=0,
    SERVO_TYPE_ONOFF=1
};

#endif // _DEFINES_H

