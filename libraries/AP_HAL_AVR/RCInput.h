
#ifndef __AP_HAL_AVR_RC_INPUT_H__
#define __AP_HAL_AVR_RC_INPUT_H__

#include <AP_HAL.h>
#include "AP_HAL_AVR_Namespace.h"

#define AVR_RC_INPUT_NUM_CHANNELS 8
#define AVR_RC_INPUT_MIN_CHANNELS 5     // for ppm sum we allow less than 8 channels to make up a valid packet

class AP_HAL_AVR::APM1RCInput : public AP_HAL::RCInput {
public:
    /**
     * init:
     * HAL_AVR::init should pass in a AP_HAL_AVR::ISRRegistry* as a void*
     */
    void     init(void* isrregistry);

    /**
     * Return true if new input since the last read()
     */
    bool  new_input();

    /**
     * Return the number of input channels in last read()
     */
    uint8_t num_channels();

    /**
     * read(uint8_t):
     * Read a single channel at a time
     */
    uint16_t read(uint8_t ch);

    /**
     * read(uint16_t*,uint8_t):
     * Read an array of channels, return the valid count
     */
    uint8_t  read(uint16_t* periods, uint8_t len);

    /**
     * Overrides: these are really grody and don't belong here but we need
     * them at the moment to make the port work.
     * case v of:
     *  v == -1 -> no change to this channel
     *  v == 0  -> do not override this channel
     *  v > 0   -> set v as override.
     */

    /* set_overrides: array starts at ch 0, for len channels */
    bool set_overrides(int16_t *overrides, uint8_t len);
    /* set_override: set just a specific channel */
    bool set_override(uint8_t channel, int16_t override);
    /* clear_overrides: equivelant to setting all overrides to 0 */
    void clear_overrides();

private:
    /* private callback for input capture ISR */
    static void _timer4_capt_cb(void);
    /* private variables to communicate with input capture isr */
    static volatile uint16_t _pulse_capt[AVR_RC_INPUT_NUM_CHANNELS];
    static volatile uint8_t  _num_channels;
    static volatile bool     _new_input;

    /* override state */
    uint16_t _override[AVR_RC_INPUT_NUM_CHANNELS]; 
};

class AP_HAL_AVR::APM2RCInput : public AP_HAL::RCInput {
    /* Pass in a AP_HAL_AVR::ISRRegistry* as void*. */
    void     init(void* isrregistry);
    bool  new_input();
    uint8_t num_channels();
    uint16_t read(uint8_t ch);
    uint8_t  read(uint16_t* periods, uint8_t len);
    bool set_overrides(int16_t *overrides, uint8_t len);
    bool set_override(uint8_t channel, int16_t override);
    void clear_overrides();
private:
    /* private callback for input capture ISR */
    static void _timer5_capt_cb(void);
    /* private variables to communicate with input capture isr */
    static volatile uint16_t _pulse_capt[AVR_RC_INPUT_NUM_CHANNELS];
    static volatile uint8_t  _num_channels;
    static volatile bool  _new_input;

    /* override state */
    uint16_t _override[AVR_RC_INPUT_NUM_CHANNELS]; 
};

#endif // __AP_HAL_AVR_RC_INPUT_H__

