
#ifndef __AP_HAL_AVR_SITL_RCOUTPUT_H__
#define __AP_HAL_AVR_SITL_RCOUTPUT_H__

#include <AP_HAL_AVR_SITL.h>

class AVR_SITL::SITLRCOutput : public AP_HAL::RCOutput {
public:
    SITLRCOutput(SITL_State *sitlState) {
	    _sitlState = sitlState;
	    _freq_hz = 50;
    }
    void     init(void* machtnichts);
    void     set_freq(uint32_t chmask, uint16_t freq_hz);
    uint16_t get_freq(uint8_t ch);
    void     enable_ch(uint8_t ch);
    void     enable_mask(uint32_t chmask);
    void     disable_ch(uint8_t ch);
    void     disable_mask(uint32_t chmask);
    void     write(uint8_t ch, uint16_t period_us);
    void     write(uint8_t ch, uint16_t* period_us, uint8_t len);
    uint16_t read(uint8_t ch);
    void     read(uint16_t* period_us, uint8_t len);

private:    
    SITL_State *_sitlState;
    uint16_t _freq_hz;
};

#endif // __AP_HAL_AVR_SITL_RCOUTPUT_H__

