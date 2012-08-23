
#ifndef __AP_HAL_AVR_H__
#define __AP_HAL_AVR_H__

#include <AP_HAL.h>
#include "HAL_AVR.h"

/**
 * This module exports AP_HAL instances only.
 * All internal drivers must conform to AP_HAL interfaces
 * and not expose implementation details.
 */

extern const AP_HAL_AVR::HAL_AVR AP_HAL_AVR_APM1;
extern const AP_HAL_AVR::HAL_AVR AP_HAL_AVR_APM2;

#endif // __AP_HAL_AVR_H__

