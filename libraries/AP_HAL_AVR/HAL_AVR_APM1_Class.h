
#ifndef __AP_HAL_AVR_APM1_HAL_AVR_H__
#define __AP_HAL_AVR_APM1_HAL_AVR_H__

#include <AP_HAL/AP_HAL.h>

/* To save linker space, we need to make sure the HAL_AVR_APM1 class
 * is built iff we are building for HAL_BOARD_APM1. These defines must
 * wrap the whole HAL_AVR_APM1 class declaration and definition. */
#if CONFIG_HAL_BOARD == HAL_BOARD_APM1

#include "AP_HAL_AVR.h"
#include "AP_HAL_AVR_Namespace.h"

/**
 * HAL_AVR_APM1 class derives from HAL but provides a constructor to use the
 * correct drivers for the APM1, and an init to set them all up properly.
 */

class HAL_AVR_APM1 : public AP_HAL::HAL {
public:
    HAL_AVR_APM1();
    void run(int argc, char* const argv[], Callbacks* callbacks) const override;
};

/**
 * Static instance exported here, defined in the Class.cpp file
 */
extern const HAL_AVR_APM1 AP_HAL_AVR_APM1;

#endif // CONFIG_HAL_BOARD == HAL_BOARD_APM1
#endif // __AP_HAL_AVR_APM1_HAL_AVR_H__

