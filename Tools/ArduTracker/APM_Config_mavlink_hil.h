// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

// Enable Autopilot Flight Mode
#define FLIGHT_MODE_CHANNEL	8
#define FLIGHT_MODE_1		AUTO
#define FLIGHT_MODE_2		RTL
#define FLIGHT_MODE_3 		FLY_BY_WIRE_A
#define FLIGHT_MODE_4       FLY_BY_WIRE_B
#define FLIGHT_MODE_5       STABILIZE
#define FLIGHT_MODE_6       MANUAL

// Hardware in the loop  protocol
#define HIL_PROTOCOL        HIL_PROTOCOL_MAVLINK

// HIL_MODE SELECTION
//
// Mavlink supports
// 1. HIL_MODE_ATTITUDE : simulated position, airspeed, and attitude
// 2. HIL_MODE_SENSORS: full sensor simulation
#define HIL_MODE            HIL_MODE_ATTITUDE

// HIL_PORT SELCTION
//
// PORT 1 
// If you would like to run telemetry communications for a groundstation
// while you are running hardware in the loop it is necessary to set
// HIL_PORT to 1. This uses the port that would have been used for the gps
// as the hardware in the loop port. You will have to solder 
// headers onto the gps port connection on the apm
// and connect via an ftdi cable.
// The baud rate is set to 115200 in this mode.
//
// PORT 3
// If you don't require telemetry communication with a gcs while running
// hardware in the loop you may use the telemetry port as the hardware in
// the loop port.
// The buad rate is controlled by SERIAL3_BAUD in this mode.
#define HIL_PORT            1

// You can set your gps protocol here for your actual
// hardware and leave it without affecting the hardware
// in the loop simulation
#define GPS_PROTOCOL 		GPS_PROTOCOL_MTK

// Ground control station comms
#define GCS_PROTOCOL        GCS_PROTOCOL_MAVLINK
#define GCS_PORT            3

// Sensors
// All sensors are supported in all modes.
// The magnetometer is not used in 
// HIL_MODE_ATTITUDE but you may leave it
// enabled if you wish.
#define AIRSPEED_SENSOR     ENABLED
#define MAGNETOMETER        ENABLED
