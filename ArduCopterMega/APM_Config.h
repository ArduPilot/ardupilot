// Example config file. Take a look at confi.h. Any term define there can be overridden by defining it here.

// GPS is auto-selected

//#define MAG_ORIENTATION		AP_COMPASS_COMPONENTS_DOWN_PINS_FORWARD


#define BROKEN_SLIDER		0		// 1 = yes (use Yaw to enter CLI mode)

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


#define CHANNEL_6_TUNING CH6_NONE
	/*
	CH6_NONE
	CH6_STABLIZE_KP
	CH6_STABLIZE_KD
	CH6_BARO_KP
	CH6_BARO_KD
	CH6_SONAR_KP
	CH6_SONAR_KD
	CH6_Y6_SCALING
	*/

//#define ACRO_RATE_TRIGGER 4200
// if you want full ACRO mode, set value to 0
// if you want mostly stabilize with flips, set value to 4200

//#define STABILIZE_ROLL_D 		0.11
//#define STABILIZE_PITCH_D 		0.11


// experimental!!
// Yaw is controled by targeting home. you will not have Yaw override.
// flying too close to home may induce spins.
#define SIMPLE_LOOK_AT_HOME 0
#define DYNAMIC_DRIFT 0 	// careful!!! 0 = off, 1 = on
