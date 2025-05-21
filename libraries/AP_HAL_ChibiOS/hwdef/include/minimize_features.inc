# this is an include file to be used on low-flash boards which are not
#  generally used for FPV/OSD purposes.  Care must be taken to
#  preserve existing features on these boards.

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

# minimized boards don't get configuration via SmartAudio by default:
define AP_SMARTAUDIO_ENABLED 0

# no Spektrum telemetry:
define HAL_SPEKTRUM_TELEM_ENABLED 0

# no MSP:
define HAL_MSP_ENABLED 0

# Copter-specific defines; these defines are only used in the Copter
#  directory, but are seen across the entire codebase:
define MODE_TURTLE_ENABLED 0

define AP_MOTORS_FRAME_DEFAULT_ENABLED 0
define AP_MOTORS_FRAME_QUAD_ENABLED 1
define AP_MOTORS_FRAME_HEXA_ENABLED 1
define AP_MOTORS_FRAME_OCTA_ENABLED 1

define HAL_EFI_ENABLED 0
define AP_ICENGINE_ENABLED 0