
#ifndef __AP_HAL_AVR_RC_OUTPUT_H__
#define __AP_HAL_AVR_RC_OUTPUT_H__

#include <AP_HAL.h>
#include "AP_HAL_AVR_Namespace.h"

class AP_HAL_AVR::APM1RCOutput : public AP_HAL::RCOutput {
public:
    /* No init argument required */
    void     init(void* machtnichts);

    /* Output freq (1/period) control */
    void     set_freq(uint32_t chmask, uint16_t freq_hz);
    uint16_t get_freq(uint8_t ch);

    /* Output active/highZ control, either by single channel at a time
     * or a mask of channels */
    void     enable_ch(uint8_t ch);
    void     enable_mask(uint32_t chmask);

    void     disable_ch(uint8_t ch);
    void     disable_mask(uint32_t chmask);

    /* Output, either single channel or bulk array of channels */
    void     write(uint8_t ch, uint16_t period_ms);
    void     write(uint8_t ch, uint16_t* period_ms, uint8_t len);

    /* Read back current output state, as either single channel or
     * array of channels. */
    uint16_t read(uint8_t ch);
    void     read(uint16_t* period_ms, uint8_t len);

private:
    uint16_t _timer_period(uint16_t speed_hz);
};

class AP_HAL_AVR::APM2RCOutput : public AP_HAL::RCOutput {
public:
    /* No init argument required */
    void     init(void* machtnichts);

    /* Output freq (1/period) control */
    void     set_freq(uint32_t chmask, uint16_t freq_hz);
    uint16_t get_freq(uint8_t ch);

    /* Output active/highZ control, either by single channel at a time
     * or a mask of channels */
    void     enable_ch(uint8_t ch);
    void     enable_mask(uint32_t chmask);

    void     disable_ch(uint8_t ch);
    void     disable_mask(uint32_t chmask);

    /* Output, either single channel or bulk array of channels */
    void     write(uint8_t ch, uint16_t period_us);
    void     write(uint8_t ch, uint16_t* period_us, uint8_t len);

    /* Read back current output state, as either single channel or
     * array of channels starting at 0. */
    uint16_t read(uint8_t ch);
    void     read(uint16_t* period_us, uint8_t len);

private:
    uint16_t _timer_period(uint16_t speed_hz);
};

#endif // __AP_HAL_AVR_RC_OUTPUT_H__

