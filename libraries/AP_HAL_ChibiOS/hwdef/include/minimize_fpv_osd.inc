# this is an include file for low-flash boards typically used in
# vehicles used for FPV/OSD flight.  These boards are unlikely to need
# drivers for rare sensors and power systems.

include minimize_common.inc

define AP_OPTICALFLOW_ENABLED 0

define AP_GPS_BACKEND_DEFAULT_ENABLED 0
define AP_GPS_UBLOX_ENABLED 1
define AP_GPS_NMEA_ENABLED 1

define AP_MOTORS_FRAME_DEFAULT_ENABLED 0
define AP_MOTORS_FRAME_QUAD_ENABLED 1

define AP_SBUSOUTPUT_ENABLED 0

define AP_ICENGINE_ENABLED 0

define AP_ADVANCEDFAILSAFE_ENABLED 0

define AC_PRECLAND_ENABLED 0
