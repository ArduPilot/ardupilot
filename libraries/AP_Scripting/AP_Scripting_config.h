#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_SerialManager/AP_SerialManager_config.h>
#include <AP_Vehicle/AP_Vehicle_config.h>

#ifndef AP_SCRIPTING_ENABLED
#define AP_SCRIPTING_ENABLED (HAL_PROGRAM_SIZE_LIMIT_KB > 1024)
#endif

#if AP_SCRIPTING_ENABLED
    #include <AP_Filesystem/AP_Filesystem_config.h>
    // enumerate all of the possible places we can read a script from.
    #if !AP_FILESYSTEM_POSIX_ENABLED && !AP_FILESYSTEM_FATFS_ENABLED && !AP_FILESYSTEM_ESP32_ENABLED && !AP_FILESYSTEM_ROMFS_ENABLED && !AP_FILESYSTEM_LITTLEFS_ENABLED
        #error "Scripting requires a filesystem"
    #endif
#endif

#ifndef AP_SCRIPTING_SERIALDEVICE_ENABLED
#define AP_SCRIPTING_SERIALDEVICE_ENABLED AP_SERIALMANAGER_REGISTER_ENABLED && (HAL_PROGRAM_SIZE_LIMIT_KB>1024)
#endif

// bindings configuration
#ifndef AP_SCRIPTING_BINDING_MOTORS_ENABLED
#define AP_SCRIPTING_BINDING_MOTORS_ENABLED (AP_SCRIPTING_ENABLED && AP_VEHICLE_ENABLED)
#endif

#ifndef AP_SCRIPTING_BINDING_VEHICLE_ENABLED
#define AP_SCRIPTING_BINDING_VEHICLE_ENABLED 1
#endif  // AP_SCRIPTING_BINDING_VEHICLE_ENABLED
