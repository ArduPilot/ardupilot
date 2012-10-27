
#ifndef __AP_HAL_PX4_H__
#define __AP_HAL_PX4_H__

#include <AP_HAL.h>
#include "HAL_PX4.h"

/**
 * This module exports AP_HAL instances only.
 * All internal drivers must conform to AP_HAL interfaces
 * and not expose implementation details.
 */

extern const AP_HAL_PX4::HAL_PX4 AP_HAL_PX4_Instance;

#endif // __AP_HAL_PX4_H__

