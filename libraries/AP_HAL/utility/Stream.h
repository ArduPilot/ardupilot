#pragma once

#include <AP_HAL/AP_HAL_Namespace.h>
#include "Print.h"

/* A simple Stream library modeled after the bits we actually use
 * from Arduino Stream */

class AP_HAL::Stream : public AP_HAL::Print {
public:
    virtual int16_t available() = 0;
    /* NB txspace was traditionally a member of BetterStream in the
     * FastSerial library. As far as concerns go, it belongs with available() */
    virtual int16_t txspace() = 0;

    /* return value for read():
     * -1 if nothing available, uint8_t value otherwise. */
    virtual int16_t read() = 0;

};
