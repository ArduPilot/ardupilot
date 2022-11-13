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
