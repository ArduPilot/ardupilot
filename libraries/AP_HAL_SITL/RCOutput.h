
#ifndef __AP_HAL_SITL_RCOUTPUT_H__
#define __AP_HAL_SITL_RCOUTPUT_H__

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include "AP_HAL_SITL.h"

class HALSITL::SITLRCOutput : public AP_HAL::RCOutput {
public:
    SITLRCOutput(SITL_State *sitlState) {
        _sitlState = sitlState;
        _freq_hz = 50;
    }
    void     init(void* machtnichts);
    void     set_freq(uint32_t chmask, uint16_t freq_hz);
    uint16_t get_freq(uint8_t ch);
    void     enable_ch(uint8_t ch);
    void     disable_ch(uint8_t ch);
    void     write(uint8_t ch, uint16_t period_us);
    uint16_t read(uint8_t ch);
    void     read(uint16_t* period_us, uint8_t len);

private:
    SITL_State *_sitlState;
    uint16_t _freq_hz;
};

#endif
#endif // __AP_HAL_SITL_RCOUTPUT_H__

