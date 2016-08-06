// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

// Internal defines, don't edit and expect things to work
// -------------------------------------------------------

#define TRUE 1
#define FALSE 0

#define DEBUG 0
#define SERVO_MAX 4500	// This value represents 45 degrees and is just an arbitrary representation of servo max travel.

// CH 7 control
enum ch7_option {
    CH7_DO_NOTHING=0,
    CH7_SAVE_WP=1
};

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
#define MASK_LOG_IMU_RAW        (1UL<<19)
