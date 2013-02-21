// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Example config file. Take a look at config.h. Any term define there can be overridden by defining it here.

# define CONFIG_APM_HARDWARE APM_HARDWARE_APM1
// # define APM2_BETA_HARDWARE

// GPS is auto-selected

//#define MAG_ORIENTATION		AP_COMPASS_COMPONENTS_DOWN_PINS_FORWARD
//#define HIL_MODE				HIL_MODE_ATTITUDE

# define CH7_OPTION		CH7_SAVE_WP
	/*
	CH7_DO_NOTHING
	CH7_SET_HOVER
	CH7_FLIP
	CH7_RTL
	CH7_AUTO_TRIM
	CH7_ADC_FILTER (experimental)
	CH7_SAVE_WP
	*/




#define FLIGHT_MODE_1   AUTO
#define FLIGHT_MODE_2   AUTO
#define FLIGHT_MODE_3   RTL
#define FLIGHT_MODE_4   RTL
#define FLIGHT_MODE_5   FBW //STABILIZE
#define FLIGHT_MODE_6   FBW //STABILIZE

#define USE_WHEEL_LUT DISABLED
