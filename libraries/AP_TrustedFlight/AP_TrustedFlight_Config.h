#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_TRUSTED_FLIGHT_ENABLED
// defaults to off.
#define AP_TRUSTED_FLIGHT_ENABLED 0
#endif

#ifndef AP_JWT_ENABLED
#define AP_JWT_ENABLED AP_TRUSTED_FLIGHT_ENABLED
#endif

#if AP_TRUSTED_FLIGHT_ENABLED
#define TRUSTED_FLIGHT_ARTIFACTS_TOTAL_LENGTH sizeof(trusted_flight_artifacts)
#else
#define TRUSTED_FLIGHT_ARTIFACTS_TOTAL_LENGTH 0
#endif
