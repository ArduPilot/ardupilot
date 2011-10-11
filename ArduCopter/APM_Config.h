// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Example config file. Take a look at config.h. Any term define there can be overridden by defining it here.

// GPS is auto-selected

//#define MAG_ORIENTATION		AP_COMPASS_COMPONENTS_DOWN_PINS_FORWARD
//#define HIL_MODE				HIL_MODE_ATTITUDE

//#define BROKEN_SLIDER		0		// 1 = yes (use Yaw to enter CLI mode)

#define FRAME_CONFIG QUAD_FRAME
	/*
	options:
	QUAD_FRAME
	TRI_FRAME
	HEXA_FRAME
	Y6_FRAME
	OCTA_FRAME
	HELI_FRAME
	*/

#define FRAME_ORIENTATION X_FRAME
	/*
	PLUS_FRAME
	X_FRAME
	V_FRAME
	*/


# define CH7_OPTION		CH7_DO_NOTHING
	/*
	CH7_DO_NOTHING
	CH7_SET_HOVER
	CH7_FLIP
	CH7_SIMPLE_MODE
	CH7_RTL
	CH7_AUTO_TRIM
	CH7_ADC_FILTER (experimental)
	*/

#define ACCEL_ALT_HOLD 0		// disabled by default, work in progress
#define ACCEL_ALT_HOLD_GAIN 12.0
// ACCEL_ALT_HOLD 1 to enable experimental alt_hold_mode

// See the config.h and defines.h files for how to set this up!
//

// lets use Manual throttle during Loiter
//#define LOITER_THR			THROTTLE_MANUAL
# define RTL_YAW 			YAW_HOLD

//#define RATE_ROLL_I 	0.18
//#define RATE_PITCH_I	0.18
