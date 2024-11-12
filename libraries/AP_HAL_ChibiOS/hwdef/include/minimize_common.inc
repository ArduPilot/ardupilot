# this include file is used to remove features which will never be
#  wanted on any low-flash board in our standard builds.  It is to be
#  included by other minimize_*.inc files and not generally used
#  otherwise.

# disable emitting nice strings when activating RC channel aux functions:
define AP_RC_CHANNEL_AUX_FUNCTION_STRINGS_ENABLED 0

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

# smaller boards lose some GPS support
define AP_GPS_BACKEND_DEFAULT_ENABLED 0
define AP_GPS_UBLOX_ENABLED 1
define AP_GPS_DRONECAN_ENABLED HAL_ENABLE_DRONECAN_DRIVERS
undef HAL_MSP_GPS_ENABLED
define HAL_MSP_GPS_ENABLED HAL_MSP_SENSORS_ENABLED

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
define AP_BATTERY_ESC_ENABLED HAL_WITH_ESC_TELEM
define AP_BATTERY_WATT_MAX_ENABLED 0
define AP_BATTERY_UAVCAN_BATTERYINFO_ENABLED HAL_ENABLE_DRONECAN_DRIVERS
define AP_BATTERY_SUM_ENABLED 1
define AP_BATTERY_SYNTHETIC_CURRENT_ENABLED 1
define AP_BATTERY_SMBUS_ENABLED 1
define AP_BATTERY_SMBUS_GENERIC_ENABLED AP_BATTERY_SMBUS_ENABLED
define AP_BATTERY_SMBUS_NEODESIGN_ENABLED AP_BATTERY_SMBUS_ENABLED
define AP_BATTERY_SMBUS_SUI_ENABLED AP_BATTERY_SMBUS_ENABLED
define AP_BATTERY_SMBUS_MAXELL_ENABLED AP_BATTERY_SMBUS_ENABLED
define AP_BATTERY_SMBUS_ROTOYE_ENABLED AP_BATTERY_SMBUS_ENABLED

# don't probe for external Barometers:
define AP_BARO_PROBE_EXTERNAL_I2C_BUSES 0

# no wind compensation code:
define HAL_BARO_WIND_COMP_ENABLED 0

# no mounts:
define HAL_MOUNT_ENABLED 0

# no generator:
define HAL_GENERATOR_ENABLED 0

# no NMEA output:
define HAL_NMEA_OUTPUT_ENABLED 0

# no Notify Display support:
define HAL_DISPLAY_ENABLED 0

# remove support for killing IMUs at runtime - a developer feature:
define AP_INERTIALSENSOR_KILL_IMU_ENABLED 0

# shortened names in @SYS/taskinfo.txt
define AP_SCHEDULER_EXTENDED_TASKINFO_ENABLED 0

define AP_LANDINGGEAR_ENABLED APM_BUILD_COPTER_OR_HELI

# Plane-specific defines; these defines are only used in the Plane
#  directory, but are seen across the entire codebase:
define AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED 0
define QAUTOTUNE_ENABLED 0

# Copter-specific defines; these defines are only used in the Copter
#  directory, but are seen across the entire codebase:
define MODE_FLOWHOLD_ENABLED 0
define MODE_ZIGZAG_ENABLED 0
define AC_NAV_GUIDED 0
define AP_OAPATHPLANNER_ENABLED 0
define MODE_FOLLOW_ENABLED 0
define MODE_GUIDED_NOGPS_ENABLED 0
define MODE_SYSTEMID_ENABLED 0
define WEATHERVANE_ENABLED 0
define MODE_AUTOROTATE_ENABLED 0

# don't send RELAY_STATUS messages:
define AP_MAVLINK_MSG_RELAY_STATUS_ENABLED 0

#fewer airspeed sensors
define AP_AIRSPEED_BACKEND_DEFAULT_ENABLED 0
define AP_AIRSPEED_MS4525_ENABLED 1
define AP_AIRSPEED_ASP5033_ENABLED 1
define AP_AIRSPEED_ANALOG_ENABLED 1
define AP_AIRSPEED_MS5525_ENABLED 1
define AP_AIRSPEED_SDP3X_ENABLED 1
define AP_AIRSPEED_NMEA_ENABLED 1  # additional checks for vehicle type in .cpp
define AP_AIRSPEED_DRONECAN_ENABLED HAL_ENABLE_DRONECAN_DRIVERS

# don't need payload place mission items on these boards, it's very niche:
define AP_MISSION_NAV_PAYLOAD_PLACE_ENABLED 0
# don't need the payload place flight behaviour either:
define AC_PAYLOAD_PLACE_ENABLED 0

# disable extra camera messages
define AP_CAMERA_INFO_FROM_SCRIPT_ENABLED 0
define AP_MAVLINK_MSG_VIDEO_STREAM_INFORMATION_ENABLED 0
