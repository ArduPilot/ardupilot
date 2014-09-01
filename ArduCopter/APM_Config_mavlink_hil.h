// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// HIL_MODE SELECTION
//
// Mavlink supports
// 1. HIL_MODE_SENSORS: full sensor simulation
#define HIL_MODE            HIL_MODE_SENSORS

// HIL_PORT SELCTION
//
// PORT 1
// If you would like to run telemetry communications for a groundstation
// while you are running hardware in the loop it is necessary to set
// HIL_PORT to 1. This uses the port that would have been used for the gps
// as the hardware in the loop port. You will have to solder
// headers onto the gps port connection on the apm
// and connect via an ftdi cable.
//
// The baud rate is set to 115200 in this mode.
//
// PORT 3
// If you don't require telemetry communication with a gcs while running
// hardware in the loop you may use the telemetry port as the hardware in
// the loop port.  Alternatively, use a telemetry/HIL shim like FGShim
// https://ardupilot-mega.googlecode.com/svn/Tools/trunk/FlightGear
//
// The buad rate is controlled by SERIAL1_BAUD in this mode.

#define HIL_PORT            3

// Sensors
// All sensors are supported in all modes.
#define MAGNETOMETER        ENABLED
