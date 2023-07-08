# a set of minimal features on smaller boards that still includes useful things that people ask for
# in particular RC telemetry, VTX and OSD control

# disable GPS and airspeed backends
include minimal.inc

define AC_OAPATHPLANNER_ENABLED 0
define AP_ICENGINE_ENABLED 0
define AP_OPTICALFLOW_ENABLED 0
define HAL_BARO_WIND_COMP_ENABLED 0
define HAL_GENERATOR_ENABLED 0

# these are not on by default on 1Mb boards but get asked for a lot
define AP_TRAMP_ENABLED 1
define HAL_CRSF_TELEM_TEXT_SELECTION_ENABLED 1

# disable emitting nice strings when activating RC channel aux functions:
define AP_RC_CHANNEL_AUX_FUNCTION_STRINGS_ENABLED 0

# disable precision landing
define AC_PRECLAND_ENABLED 0

# low-end boards aren't expected to be used in environments where
# things like satellite communications are required:
define HAL_HIGH_LATENCY2_ENABLED 0

# Gripper isn't a vital feature for smaller boards
define AP_GRIPPER_ENABLED 0

# Sprayer isn't a vital feature for smaller boards
define HAL_SPRAYER_ENABLED 0

# disable use of onboard FFT library:
define HAL_GYROFFT_ENABLED 0

# several notify backends are rare and not supported by default on smaller boards:
define AP_NOTIFY_NCP5623_ENABLED 0

# HOTT telemetry is quite rare, so we don't include it on smaller boards
define HAL_HOTT_TELEM_ENABLED 0

# smaller boards lose SIRF GPS support
define AP_GPS_SIRF_ENABLED 0

# no moving baseline support:
define GPS_MOVING_BASELINE 0

# No LTM telemetry on minimized boards:
define AP_LTM_TELEM_ENABLED 0

# various structures increase the flash size when using >16 servos:
define NUM_SERVO_CHANNELS 16

# no Winch if minimized:
define AP_WINCH_ENABLED 0

# prune out some odd camera backends:
define AP_CAMERA_BACKEND_DEFAULT_ENABLED 0
define AP_CAMERA_RELAY_ENABLED AP_CAMERA_ENABLED
define AP_CAMERA_SERVO_ENABLED AP_CAMERA_ENABLED

# no SLCAN on these boards (use can-over-mavlink if required)
define AP_CAN_SLCAN_ENABLED 0

# no PiccoloCAN:
define HAL_PICCOLO_CAN_ENABLE 0

# no beacon support on minimized boards:
define AP_BEACON_ENABLED 0

# restricted battery backends:
define AP_BATTERY_BACKEND_DEFAULT_ENABLED 0
define AP_BATTERY_ANALOG_ENABLED 1
define AP_BATTERY_ESC_ENABLED 1
define AP_BATTERY_UAVCAN_BATTERYINFO_ENABLED HAL_ENABLE_DRONECAN_DRIVERS

# don't probe for external Barometers:
define AP_BARO_PROBE_EXTERNAL_I2C_BUSES 0

# no mounts:
define HAL_MOUNT_ENABLED 0

# no NMEA output:
define HAL_NMEA_OUTPUT_ENABLED 0

# no Notify Display support:
define HAL_DISPLAY_ENABLED 0

# remove support for killing IMUs at runtime - a developer feature:
define AP_INERTIALSENSOR_KILL_IMU_ENABLED 0

# shortened names in @SYS/taskinfo.txt
define AP_SCHEDULER_EXTENDED_TASKINFO_ENABLED 0
