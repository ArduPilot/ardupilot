include minimize_common.inc

# for now we make this define, but we should migrate from this to much
#  more specific defines - added below.
define HAL_MINIMIZE_FEATURES 1

# Crossfire telemetry must be explicitly enabled on minimized boards:
define HAL_CRSF_TELEM_ENABLED 0

# RunCam control isn't available on smaller boards
define HAL_RUNCAM_ENABLED 0

# minimized boards don't get configuration via SmartAudio by default:
define AP_SMARTAUDIO_ENABLED 0

# minimized boards don't get configuration via Tramp by default:
undef AP_TRAMP_ENABLED
define AP_TRAMP_ENABLED 0

# Soaring is off by defauult on of smaller boards.
define HAL_SOARING_ENABLED 0

# no CRSF helpful menus
undef HAL_CRSF_TELEM_TEXT_SELECTION_ENABLED
define HAL_CRSF_TELEM_TEXT_SELECTION_ENABLED 1

# no Spektrum telemetry:
define HAL_SPEKTRUM_TELEM_ENABLED 0

# remove various OSD features by default:
define OSD_ENABLED 0
define HAL_PLUSCODE_ENABLE 0
define HAL_OSD_SIDEBAR_ENABLE 0
define OSD_PARAM_ENABLED 0

# disable use of various RPM options
define AP_RPM_EFI_ENABLED 0
define AP_RPM_GENERATOR_ENABLED 0

# no MSP:
define HAL_MSP_ENABLED 0
