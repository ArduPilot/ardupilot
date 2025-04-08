undef AP_RC_CHANNEL_ENABLED
define AP_RC_CHANNEL_ENABLED 0

define AP_PERIPH_RTC_ENABLED 1

define AP_PERIPH_SERIAL_OPTIONS_ENABLED 1
define AP_NETWORKING_BACKEND_PPP 1
define AP_NETWORKING_CAN_MCAST_ENABLED 1

define AP_PERIPH_NET_PPP_PORT_DEFAULT 1
define AP_PERIPH_NET_PPP_BAUD_DEFAULT 12500000

// add scripting for web interface
define AP_SCRIPTING_ENABLED 1

// ROMFS filesystem only
define AP_FILESYSTEM_ROMFS_ENABLED 1

// allow scripts to add parameters
define AP_PARAM_DYNAMIC_ENABLED 1

ROMFS_DIRECTORY Tools/AP_Periph/Web

# we don't want to use external flash with network bootloader
EXT_FLASH_SIZE_MB 0
