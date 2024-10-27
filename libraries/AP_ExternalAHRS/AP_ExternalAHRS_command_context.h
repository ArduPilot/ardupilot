#pragma once

#include "AP_ExternalAHRS_config.h"

#if HAL_EXTERNAL_AHRS_ENABLED

enum class ExternalAHRS_command {
    INVALID,
    START,
    STOP,
    ENABLE_GNSS,
    DISABLE_GNSS,
};

#endif
