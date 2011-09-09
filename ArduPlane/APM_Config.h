// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// This file is just a placeholder for your configuration file.  If you wish to change any of the setup parameters from
// their default values, place the appropriate #define statements here.

// For example if you wanted the Port 3 baud rate to be 38400 you would add a statement like the one below (uncommented)
//#define SERIAL3_BAUD        38400
//#define GCS_PROTOCOL        GCS_PROTOCOL_NONE


// You may also put an include statement here to point at another configuration file.  This is convenient if you maintain
// different configuration files for different aircraft or HIL simulation.  See the examples below
//#include "APM_Config_mavlink_hil.h"
//#include "Skywalker.h"

// The following are the recommended settings for Xplane simulation. Remove the leading "/* and trailing "*/" to enable:

/*
#define HIL_PROTOCOL        HIL_PROTOCOL_MAVLINK
#define HIL_MODE            HIL_MODE_ATTITUDE
#define HIL_PORT            0
#define GCS_PROTOCOL        GCS_PROTOCOL_MAVLINK
#define GCS_PORT            3
*/

#define MAGNETOMETER ENABLED

// ----- Camera definitions ------
// ------------------------------
#define CAMERA ENABLED
#define CAM_DEBUG DISABLED
// Comment out servos that you do not have
//#define CAM_SERVO		8		// Camera servo channel
#define CAM_ANGLE		30		// Set angle in degrees
//#define CAM_CLICK		45		// This is the position of the servo arm when it actually clicks
//#define OPEN_SERVO		5		// Retraction servo channel - my camera retracts yours might not.

// Camera yaw (left-right)
#define YAW_SERVO		7		// Pan servo channel (can be pitch in stabilization)
#define YAW_REV			1		// output is normal = 1 output is reversed = -1
#define YAW_CENTER		0		// Camera center bearing with relation to airframe forward motion - deg
#define YAW_RANGE		90		// Pan Servo sweep in degrees
#define YAW_RATIO		10.31	// match this to the swing of your pan servo

// Camera pitch (up-down)
#define PITCH_SERVO		6		// Tilt servo channel (can be roll in stabilization)
#define PITCH_REV		1		// output is normal = 1 output is reversed = -1
#define PITCH_CENTER		0		// Camera center bearing with relation to airframe forward motion - deg
#define PITCH_RANGE		90		// Tilt Servo sweep in degrees
#define PITCH_RATIO		10.31	// match this to the swing of your tilt servo

// Camera roll (up-down)
#define ROLL_SERVO		6		// Tilt servo channel (can be roll in stabilization)
#define ROLL_REV		1		// output is normal = 1 output is reversed = -1
#define ROLL_CENTER		0		// Camera center bearing with relation to airframe forward motion - deg
#define ROLL_RANGE		90		// Tilt Servo sweep in degrees
#define ROLL_RATIO		10.31	// match this to the swing of your tilt servo
