// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

// Example config file. Use APM_Config.h.reference and the wiki to find additional defines to setup your plane.
// Once you upload the code, run the factory "reset" to save all config values to EEPROM.
// After reset, use the setup mode to set your radio limits for CH1-4, and to set your flight modes.

// GPS is auto-selected

#define MAG_ORIENTATION	AP_COMPASS_COMPONENTS_DOWN_PINS_FORWARD

#define SERIAL0_BAUD			115200
#define SERIAL3_BAUD		    115200

// Hardware in the loop  protocol
#if 0
# define HIL_MODE            HIL_MODE_NONE
#else
# define HIL_MODE            HIL_MODE_ATTITUDE
# define HIL_PROTOCOL        HIL_PROTOCOL_MAVLINK
# define HIL_PORT			 0
#endif

// You can set your gps protocol here for your actual
// hardware and leave it without affecting the hardware
// in the loop simulation

// Ground control station comms
#define GCS_PROTOCOL        GCS_PROTOCOL_MAVLINK
#define GCS_PORT            0

//#define RADIO_OVERRIDE_DEFAULTS { 1500, 1500, 1000, 1500, 1000, 1000, 1000, 1815 }

#define FLIGHT_MODE_CHANNEL CH_8

#define FLIGHT_MODE_1			STABILIZE
#define FLIGHT_MODE_2			ALT_HOLD
#define FLIGHT_MODE_3			ACRO
#define FLIGHT_MODE_4			STABILIZE
#define FLIGHT_MODE_5			STABILIZE
#define FLIGHT_MODE_6			STABILIZE

//#define ALWAYS_RESET_RADIO_RANGE 1
//#define ALWAYS_RESET_MODES 1

#define GPS_PROTOCOL  GPS_PROTOCOL_UBLOX

// Use MODE1 arming
#define MOTOR_ARM_CONDITION (g.rc_3.control_in == 0 && g.rc_1.control_in > 2700)
#define MOTOR_DISARM_CONDITION (g.rc_3.control_in == 0 && g.rc_1.control_in < -2700)
