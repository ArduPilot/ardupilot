// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// THIS IS A SAMPLE CONFIGURATION FILE FOR DOING HARDWARE IN THE LOOP TESTING USING THE MAVLINK HIL INTERFACE
// IF YOU WANTED TO USE THIS YOU WOULD COPY THE CONTENTS INTO YOUR APM_Config.h FILE!


// Enable Autopilot Flight Mode
#define FLIGHT_MODE_CHANNEL	8
#define FLIGHT_MODE_1		AUTO
#define FLIGHT_MODE_2		RTL
#define FLIGHT_MODE_3 		FLY_BY_WIRE_A
#define FLIGHT_MODE_4       FLY_BY_WIRE_B
#define FLIGHT_MODE_5       STABILIZE
#define FLIGHT_MODE_6       MANUAL

// HIL_MODE SELECTION
//
// Mavlink supports
// 1. HIL_MODE_ATTITUDE : simulated position, airspeed, and attitude
// 2. HIL_MODE_SENSORS: full sensor simulation
#define HIL_MODE            HIL_MODE_ATTITUDE

// Sensors
// All sensors are supported in all modes.
// The magnetometer is not used in 
// HIL_MODE_ATTITUDE but you may leave it
// enabled if you wish.
#define AIRSPEED_SENSOR     ENABLED
#define MAGNETOMETER        ENABLED
#define AIRSPEED_CRUISE     25
#define THROTTLE_FAILSAFE   ENABLED
