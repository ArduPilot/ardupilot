#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_VEHICLE_ENABLED
#define AP_VEHICLE_ENABLED 1
#endif

// the main loop failsafe detects that the vehicle's main loop has
// stopped running and takes vehicle-specific action (Plane passes RC
// straight through to the outputs; Copter, Sub and Blimp wind the
// motors down; Rover disarms)
#ifndef AP_MAINLOOP_FAILSAFE_ENABLED
#define AP_MAINLOOP_FAILSAFE_ENABLED 1
#endif
