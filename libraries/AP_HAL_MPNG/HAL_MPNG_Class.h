
#ifndef __AP_HAL_MPNG_HAL_MPNG_H__
#define __AP_HAL_MPNG_HAL_MPNG_H__

#include <AP_HAL.h>

/* To save linker space, we need to make sure the HAL_MPNG class
 * is built iff we are building for HAL_BOARD_MPNG. These defines must
 * wrap the whole HAL_MPNG class declaration and definition. */
#if CONFIG_HAL_BOARD == HAL_BOARD_MPNG

#include <AP_HAL_MPNG.h>
#include "AP_HAL_MPNG_Namespace.h"

/**
 * HAL_MPNG class derives from HAL but provides a constructor to use the
 * correct drivers for the MPNG, and an init to set them all up properly.
 */

class HAL_MPNG : public AP_HAL::HAL {
public:
    HAL_MPNG();
    AP_HAL::UARTDriver* uartD;
    void init(int argc, char * const argv[]) const;
};

/**
 * Static instance exported here, defined in the Class.cpp file
 */
extern const HAL_MPNG AP_HAL_MPNG;

#endif // CONFIG_HAL_BOARD == HAL_BOARD_MPNG
#endif // __AP_HAL_MPNG_HAL_MPNG_H__

