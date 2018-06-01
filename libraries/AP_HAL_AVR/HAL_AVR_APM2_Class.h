
#ifndef __AP_HAL_AVR_APM2_HAL_AVR_H__
#define __AP_HAL_AVR_APM2_HAL_AVR_H__

#include <AP_HAL/AP_HAL.h>

/* To save linker space, we need to make sure the HAL_AVR_APM2 class
 * is built iff we are building for HAL_BOARD_APM2. These defines must
 * wrap the whole HAL_AVR_APM2 class declaration and definition. */
#if CONFIG_HAL_BOARD == HAL_BOARD_APM2

#include "AP_HAL_AVR.h"
#include "AP_HAL_AVR_Namespace.h"

/**
 * HAL_AVR_APM2 class derives from HAL but provides an AVR-specific
 * init method.
 */

class HAL_AVR_APM2 : public AP_HAL::HAL {
public:
    HAL_AVR_APM2();
    void run(int argc, char* const argv[], Callbacks* callbacks) const override;
};

/**
 * Static instance exported here, defined in the Class.cpp file
 */
extern const HAL_AVR_APM2 AP_HAL_AVR_APM2;

#endif // CONFIG_HAL_BOARD == HAL_BOARD_APM2
#endif // __AP_HAL_AVR_APM2_HAL_AVR_H__

