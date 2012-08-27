
#ifndef __AP_HAL_RC_INPUT_H__
#define __AP_HAL_RC_INPUT_H__

#include "AP_HAL_Namespace.h"

#define RC_INPUT_MIN_PULSEWIDTH 900
#define RC_INPUT_MAX_PULSEWIDTH 2100

class AP_HAL::RCInput {
public:
    /**
     * Call init from the platform hal instance init, so that both the type of
     * the RCInput implementation and init argument (e.g. ISRRegistry) are
     * known to the programmer. (Its too difficult to describe this dependency
     * in the C++ type system.)
     */
    virtual void init(void* implspecific) = 0;

    /**
     * Return the number of currently valid channels.
     * Typically 0 (no valid radio channels) or 8 (implementation-defined)
     * Could be less than or greater than 8 depending on your incoming radio
     * or PPM stream
     */
    virtual uint8_t  valid() = 0;

    /* Read a single channel at a time */
    virtual uint16_t read(uint8_t ch) = 0;

    /* Read an array of channels, return the valid count */
    virtual uint8_t read(uint16_t* periods, uint8_t len) = 0;
};

#endif // __AP_HAL_RC_INPUT_H__

