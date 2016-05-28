
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
     * Return true if there has been new input since the last read() call
     */
    virtual bool  new_input() = 0;

    /**
     * Return the number of valid channels in the last read
     */
    virtual uint8_t  num_channels() = 0;

    /* Read a single channel at a time */
    virtual uint16_t read(uint8_t ch) = 0;

    /* Read an array of channels, return the valid count */
    virtual uint8_t read(uint16_t* periods, uint8_t len) = 0;

    /**
     * Overrides: these are really grody and don't belong here but we need
     * them at the moment to make the port work.
     * case v of:
     *  v == -1 -> no change to this channel
     *  v == 0  -> do not override this channel
     *  v > 0   -> set v as override.
     */

    /* set_overrides: array starts at ch 0, for len channels */
    virtual bool set_overrides(int16_t *overrides, uint8_t len) = 0;
    /* set_override: set just a specific channel */
    virtual bool set_override(uint8_t channel, int16_t override) = 0;
    /* clear_overrides: equivelant to setting all overrides to 0 */
    virtual void clear_overrides() = 0;

};

#endif // __AP_HAL_RC_INPUT_H__

