include minimize_common.inc

# remove various OSD features by default:
define OSD_ENABLED 0
define HAL_PLUSCODE_ENABLE 0
define HAL_OSD_SIDEBAR_ENABLE 0
define OSD_PARAM_ENABLED 0

# Crossfire telemetry must be explicitly enabled on minimized boards:
define HAL_CRSF_TELEM_ENABLED 0

# Soaring is off by defauult on of smaller boards.
define HAL_SOARING_ENABLED 0

# RunCam control isn't available on smaller boards
define HAL_RUNCAM_ENABLED 0
