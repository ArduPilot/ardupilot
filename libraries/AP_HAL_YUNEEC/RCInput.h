
#ifndef __AP_HAL_YUNEEC_RCINPUT_H__
#define __AP_HAL_YUNEEC_RCINPUT_H__

#include <AP_HAL_YUNEEC.h>

#define YUNEEC_RC_INPUT_NUM_CHANNELS 8
#define YUNEEC_RC_INPUT_MIN_CHANNELS 5     // for ppm sum we allow less than 8 channels to make up a valid packet

typedef void (*voidFuncPtr)(void);

class YUNEEC::YUNEECRCInput : public AP_HAL::RCInput {
public:
    void init(void* machtnichts);
    bool  new_input();
    uint8_t num_channels();
    uint16_t read(uint8_t ch);
    uint8_t read(uint16_t* periods, uint8_t len);

    bool set_overrides(int16_t *overrides, uint8_t len);
    bool set_override(uint8_t channel, int16_t override);
    void clear_overrides();

private:
    /* private callback for input capture ISR */
    static void _timer_capt_cb(void);
    static void _attachInterrupt(voidFuncPtr callback);
    /* private variables to communicate with input capture isr */
    static volatile uint16_t _pulse_capt[YUNEEC_RC_INPUT_NUM_CHANNELS];
    static volatile uint8_t  _valid_channels;
    static volatile bool  _new_input;

    /* override state */
    uint16_t _override[YUNEEC_RC_INPUT_NUM_CHANNELS];
};

#endif // __AP_HAL_YUNEEC_RCINPUT_H__
