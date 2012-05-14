 
 // -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// This file is just a placeholder for your configuration file.  If you wish to change any of the setup parameters from
// their default values, place the appropriate #define statements here.

// For example if you wanted the Port 3 baud rate to be 38400 you would add a statement like the one below (uncommented)
//#define SERIAL3_BAUD        38400
//#define GCS_PROTOCOL        GCS_PROTOCOL_NONE

#define CONFIG_APM_HARDWARE APM_HARDWARE_APM1

#define LITE  DISABLED    // if LITE is ENABLED, you may use an APM1280 or APM2560 CPU only (IMU less) with a GPS MT3329
                          // if LITE is DISABLED, this is for a full APM v1 (Oilpan + GPS MT3329 + Magnetometer HMC5883L) or APM v2

#define CLI_ENABLED         ENABLED
#define CLI_SLIDER_ENABLED  DISABLED
#define CLOSED_LOOP_NAV     ENABLED
#define AUTO_WP_RADIUS      DISABLED

#define TRACE               DISABLED

//#include "APM_Config_HILmode.h"  // for test in HIL mode with AeroSIM Rc 3.83
#include "APM_Config_Rover.h"      // to be used with the real Traxxas model Monster Jam Grinder

// Radio setup:
// APM INPUT (Rec = receiver)
// Rec ch1: Roll 
// Rec ch2: Throttle
// Rec ch3: Pitch
// Rec ch4: Yaw
// Rec ch5: not used
// Rec ch6: 
// Rec ch7: Option channel to 2 positions switch
// Rec ch8: Mode channel to 3 positions switch
// APM OUTPUT
// Ch1: Wheel servo (direction)
// Ch2: not used
// Ch3: to the motor ESC
// Ch4: not used

// The following are the recommended settings for Xplane simulation. Remove the leading "/* and trailing "*/" to enable:

/*
#define HIL_PROTOCOL        HIL_PROTOCOL_MAVLINK
#define HIL_MODE            HIL_MODE_ATTITUDE
#define HIL_PORT            0
#define GCS_PROTOCOL        GCS_PROTOCOL_MAVLINK
#define GCS_PORT            3
*/


