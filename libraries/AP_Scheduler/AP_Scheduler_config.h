#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_SCHEDULER_ENABLED
#define AP_SCHEDULER_ENABLED 1
#endif

#ifndef AP_SCHEDULER_EXTENDED_TASKINFO_ENABLED
#define AP_SCHEDULER_EXTENDED_TASKINFO_ENABLED 1
#endif

// the main loop rate Copter and Heli default to; a board whose main loop
// cannot carry it lowers it, leaving every other vehicle's default alone
#ifndef AP_SCHEDULER_COPTER_DEFAULT_LOOP_RATE_HZ
#define AP_SCHEDULER_COPTER_DEFAULT_LOOP_RATE_HZ 400
#endif
