#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_TRUSTED_FLIGHT_ENABLED
// defaults to off.
#define AP_TRUSTED_FLIGHT_ENABLED 0
#endif

#ifndef AP_JWT_ENABLED
// defaults to off.
#define AP_JWT_ENABLED 0
#endif

#ifndef AP_TRUSTED_FLIGHT_ISSUER_LENGTH 
// defaults to 0.
#define AP_TRUSTED_FLIGHT_ISSUER_LENGTH 0
#endif

#if AP_TRUSTED_FLIGHT_ENABLED
    struct trusted_flight_artifacts {
        uint32_t key_type;
        uint32_t key_len;
        uint8_t key[32];
        uint32_t issuer_len;
        uint8_t issuer[AP_TRUSTED_FLIGHT_ISSUER_LENGTH];
    };
#define TRUSTED_FLIGHT_ARTIFACTS_TOTAL_LENGTH sizeof(trusted_flight_artifacts)
#else
#define TRUSTED_FLIGHT_ARTIFACTS_TOTAL_LENGTH 0
#endif
