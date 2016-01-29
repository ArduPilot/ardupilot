// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef _DEFINES_H
#define _DEFINES_H

// Internal defines, don't edit and expect things to work
// -------------------------------------------------------

#define TRUE 1
#define FALSE 0
#define ToRad(x) radians(x)	// *pi/180
#define ToDeg(x) degrees(x)	// *180/pi

#define DEBUG 0
#define SERVO_MAX 4500	// This value represents 45 degrees and is just an arbitrary representation of servo max travel.

// active altitude sensor
// ----------------------
#define SONAR 0
#define BARO 1

// CH 7 control
enum ch7_option {
    CH7_DO_NOTHING=0,
    CH7_SAVE_WP=1
};

#define T6 1000000
#define T7 10000000

// HIL enumerations
#define HIL_MODE_DISABLED			0
#define HIL_MODE_SENSORS			1

// Auto Pilot modes
// ----------------
enum mode {
    MANUAL=0,
	LEARNING=2,
    STEERING=3,
    HOLD=4,
    AUTO=10,
    RTL=11,
    GUIDED=15,
    INITIALISING=16
};

// types of failsafe events
#define FAILSAFE_EVENT_THROTTLE (1<<0)
#define FAILSAFE_EVENT_GCS      (1<<1)
#define FAILSAFE_EVENT_RC       (1<<2)

//repeating events
#define NO_REPEAT 0
#define CH_5_TOGGLE 1
#define CH_6_TOGGLE 2
#define CH_7_TOGGLE 3
#define CH_8_TOGGLE 4
#define RELAY_TOGGLE 5
#define STOP_REPEAT 10

#define MAV_CMD_CONDITION_YAW 23

//  Logging parameters
#define LOG_CTUN_MSG	        0x01
#define LOG_NTUN_MSG    		0x02
#define LOG_PERFORMANCE_MSG		0x03
#define LOG_STARTUP_MSG 		0x06
#define LOG_SONAR_MSG 		    0x07
#define LOG_ARM_DISARM_MSG      0x08
#define LOG_STEERING_MSG        0x0D

#define TYPE_AIRSTART_MSG		0x00
#define TYPE_GROUNDSTART_MSG	0x01
#define MAX_NUM_LOGS			100

#define MASK_LOG_ATTITUDE_FAST 	(1<<0)
#define MASK_LOG_ATTITUDE_MED 	(1<<1)
#define MASK_LOG_GPS 			(1<<2)
#define MASK_LOG_PM 			(1<<3)
#define MASK_LOG_CTUN 			(1<<4)
#define MASK_LOG_NTUN			(1<<5)
#define MASK_LOG_MODE			(1<<6)
#define MASK_LOG_IMU			(1<<7)
#define MASK_LOG_CMD			(1<<8)
#define MASK_LOG_CURRENT		(1<<9)
#define MASK_LOG_SONAR   		(1<<10)
#define MASK_LOG_COMPASS   		(1<<11)
#define MASK_LOG_CAMERA   		(1<<12)
#define MASK_LOG_STEERING  		(1<<13)
#define MASK_LOG_RC     		(1<<14)
#define MASK_LOG_ARM_DISARM     (1<<15)
#define MASK_LOG_WHEN_DISARMED  (1UL<<16)
#define MASK_LOG_IMU_RAW        (1UL<<19)


// Waypoint Modes
// ----------------
#define ABS_WP 0
#define REL_WP 1

// Command Queues
// ---------------
#define COMMAND_MUST 0
#define COMMAND_MAY 1
#define COMMAND_NOW 2

// Events
// ------
#define EVENT_WILL_REACH_WAYPOINT 1
#define EVENT_SET_NEW_COMMAND_INDEX 2
#define EVENT_LOADED_WAYPOINT 3
#define EVENT_LOOP 4

// Climb rate calculations
#define	ALTITUDE_HISTORY_LENGTH 8	//Number of (time,altitude) points to regress a climb rate from

// sonar
#define MAX_SONAR_XL 0
#define MAX_SONAR_LV 1
#define SonarToCm(x) (x*1.26)   // Sonar raw value to centimeters
#define AN4			4
#define AN5			5

#define SPEEDFILT 400			// centimeters/second; the speed below which a groundstart will be triggered

// convert a boolean (0 or 1) to a sign for multiplying (0 maps to 1, 1 maps to -1)
#define BOOL_TO_SIGN(bvalue) ((bvalue)?-1:1)

#endif // _DEFINES_H
