
#ifndef __AP_HAL_EMPTY_RCOUTPUT_H__
#define __AP_HAL_EMPTY_RCOUTPUT_H__

#include <AP_HAL_Empty.h>

class Empty::EmptyRCOutput : public AP_HAL::RCOutput {
    void     init(void* machtnichts);
    void     set_freq(uint32_t chmask, uint16_t freq_hz);
    uint16_t get_freq(uint8_t ch);
    void     enable_ch(uint8_t ch);
    void     disable_ch(uint8_t ch);
    void     write(uint8_t ch, uint16_t period_us);
    void     write(uint8_t ch, uint16_t* period_us, uint8_t len);
    uint16_t read(uint8_t ch);
    void     read(uint16_t* period_us, uint8_t len);
};

#endif // __AP_HAL_EMPTY_RCOUTPUT_H__
