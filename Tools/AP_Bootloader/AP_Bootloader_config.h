#pragma once

#include "hwdef.h"
#include <AP_Networking/AP_Networking_Config.h>

#ifndef AP_BOOTLOADER_FLASH_FROM_SD_ENABLED
#define AP_BOOTLOADER_FLASH_FROM_SD_ENABLED 0
#endif

#ifndef AP_BOOTLOADER_NETWORK_ENABLED
#define AP_BOOTLOADER_NETWORK_ENABLED AP_NETWORKING_ENABLED
#endif
