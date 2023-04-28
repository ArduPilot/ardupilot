# for now we make this define, but we should migrate from this to much
#  more specific defines - added below.
define HAL_MINIMIZE_FEATURES 1

# disable emitting nice strings when activating RC channel aux functions:
define AP_RC_CHANNEL_AUX_FUNCTION_STRINGS_ENABLED 0

# low-end boards aren't expected to be used in environments where
# things like satellite communications are required:
define HAL_HIGH_LATENCY2_ENABLED 0

# Crossfire telemetry must be explicitly enabled on minimized boards:
define HAL_CRSF_TELEM_ENABLED 0

# Gripper isn't a vital feature for smaller boards
define AP_GRIPPER_ENABLED 0

# Sprayer isn't a vital feature for smaller boards
define HAL_SPRAYER_ENABLED 0

# RunCam control isn't available on smaller boards
define HAL_RUNCAM_ENABLED 0

# disable use of onboard FFT library:
define HAL_GYROFFT_ENABLED 0

# several notify backends are rare and not supported by default on smaller boards:
define AP_NOTIFY_NCP5623_ENABLED 0

# Soaring is off by defauult on of smaller boards.
define HAL_SOARING_ENABLED 0

# HOTT telemetry is quite rare, so we don't include it on smaller boards
define HAL_HOTT_TELEM_ENABLED 0

# smaller boards lose SIRF GPS support
define AP_GPS_SIRF_ENABLED 0

# minimized boards don't get configuration via SmartAudio by default:
define AP_SMARTAUDIO_ENABLED 0

# minimized boards don't get configuration via Tramp by default:
define AP_TRAMP_ENABLED 0

# various structures increase the flash size when using >16 servos:
define NUM_SERVO_CHANNELS 16

# remove various OSD features by default:
define OSD_ENABLED 0
define HAL_PLUSCODE_ENABLE 0
define HAL_OSD_SIDEBAR_ENABLE 0
define OSD_PARAM_ENABLED 0

# no Winch if minimized:
define AP_WINCH_ENABLED 0

# prune out some odd camera backends:
define AP_CAMERA_BACKEND_DEFAULT_ENABLED 0
define AP_CAMERA_RELAY_ENABLED AP_CAMERA_ENABLED
define AP_CAMERA_SERVO_ENABLED AP_CAMERA_ENABLED

# no SLCAN on these boards (use can-over-mavlink if required)
define AP_CAN_SLCAN_ENABLED 0

# no beacon support on minimized boards:
define AP_BEACON_ENABLED 0

# restricted battery backends:
define AP_BATTERY_BACKEND_DEFAULT_ENABLED 0
define AP_BATTERY_ANALOG_ENABLED 1
define AP_BATTERY_ESC_ENABLED 1
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
