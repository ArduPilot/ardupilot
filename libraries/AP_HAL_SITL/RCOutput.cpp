#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "RCOutput.h"

using namespace HALSITL;

void SITLRCOutput::init(void* machtnichts) {}

void SITLRCOutput::set_freq(uint32_t chmask, uint16_t freq_hz) {
    _freq_hz = freq_hz;
}

uint16_t SITLRCOutput::get_freq(uint8_t ch) {
    return _freq_hz;
}

void SITLRCOutput::enable_ch(uint8_t ch)
{}

void SITLRCOutput::disable_ch(uint8_t ch)
{}

void SITLRCOutput::write(uint8_t ch, uint16_t period_us)
{
    if (ch < SITL_NUM_CHANNELS) {
        _sitlState->pwm_output[ch] = period_us;
    }
}

uint16_t SITLRCOutput::read(uint8_t ch)
{
    if (ch < SITL_NUM_CHANNELS) {
        return _sitlState->pwm_output[ch];
    }
    return 0;
}

void SITLRCOutput::read(uint16_t* period_us, uint8_t len)
{
    memcpy(period_us, _sitlState->pwm_output, len*sizeof(uint16_t));
}

#endif
