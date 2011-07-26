// Example config file. Take a look at confi.h. Any term define there can be overridden by defining it here.

// GPS is auto-selected

//#define MAG_ORIENTATION		AP_COMPASS_COMPONENTS_DOWN_PINS_FORWARD


#define BROKEN_SLIDER		0		// 1 = yes (use Yaw to enter CLI mode)

//#define FRAME_CONFIG QUAD_FRAME
	/*
	options:
	QUAD_FRAME
	TRI_FRAME
	HEXA_FRAME
	Y6_FRAME
	OCTA_FRAME
	HELI_FRAME
	*/

//#define FRAME_ORIENTATION X_FRAME
	/*
	PLUS_FRAME
	X_FRAME
        V_FRAME
	*/


//#define CHANNEL_6_TUNING  CH6_NONE
	/*
	CH6_NONE
	CH6_STABILIZE_KP
	CH6_STABILIZE_KI
	CH6_RATE_KP
	CH6_RATE_KI
	CH6_THROTTLE_KP
	CH6_THROTTLE_KD
	CH6_YAW_KP
	CH6_YAW_KI
	CH6_YAW_RATE_KP
	CH6_YAW_RATE_KI
	CH6_TOP_BOTTOM_RATIO
	CH6_PMAX
        CH6_RELAY
	*/

// experimental!!
// Yaw is controled by targeting home. you will not have Yaw override.
// flying too close to home may induce spins.
#define SIMPLE_LOOK_AT_HOME 0
