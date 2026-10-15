#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

// Maximum number of WheelEncoder instances.  Four allows one encoder per
// wheel on a four wheeled vehicle; a vehicle with one encoder per side
// simply configures the first two.  Boards short of flash keep two
#ifndef WHEELENCODER_MAX_INSTANCES
#if HAL_PROGRAM_SIZE_LIMIT_KB > 1024
#define WHEELENCODER_MAX_INSTANCES 4
#else
#define WHEELENCODER_MAX_INSTANCES 2
#endif
#endif

static_assert(WHEELENCODER_MAX_INSTANCES >= 1 && WHEELENCODER_MAX_INSTANCES <= 4,
              "WHEELENCODER_MAX_INSTANCES must be between 1 and 4");

