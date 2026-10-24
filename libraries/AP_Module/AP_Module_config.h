#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#define AP_MODULE_DEFAULT_THREAD_PRIORITY 2
#else
#define AP_MODULE_DEFAULT_THREAD_PRIORITY 1
#endif

// Only relocation harnesses may load containers without runtime identity.
#ifndef AP_MODULE_LEGACY_PROBE_ENABLED
#define AP_MODULE_LEGACY_PROBE_ENABLED 0
#endif

#ifndef AP_MODULE_SHARED_STATE_ENABLED
#define AP_MODULE_SHARED_STATE_ENABLED AP_MODULE_ENABLED
#endif
