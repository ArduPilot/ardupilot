#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "RCOutput.h"

#define ENABLE_DEBUG 0

#if ENABLE_DEBUG
# include <stdio.h>
# define Debug(fmt, args ...)  do {::printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while (0)
#else
# define Debug(fmt, args ...)
#endif

using namespace HALSITL;

void RCOutput::init() {}

void RCOutput::set_freq(uint32_t chmask, uint16_t freq_hz)
{
    Debug("set_freq(0x%04x, %u)\n", static_cast<uint32_t>(chmask), static_cast<uint32_t>(freq_hz));
    _freq_hz = freq_hz;
}

uint16_t RCOutput::get_freq(uint8_t ch)
{
    return _freq_hz;
}

void RCOutput::enable_ch(uint8_t ch)
{
    if (!(_enable_mask & (1U << ch))) {
        Debug("enable_ch(%u)\n", ch);
    }
    _enable_mask |= 1U << ch;
}

void RCOutput::disable_ch(uint8_t ch)
{
    if (_enable_mask & (1U << ch)) {
        Debug("disable_ch(%u)\n", ch);
    }
    _enable_mask &= ~1U << ch;
}

void RCOutput::write(uint8_t ch, uint16_t period_us)
{
    if (ch < SITL_NUM_CHANNELS && (_enable_mask & (1U<<ch))) {
        if (_corked) {
            _pending[ch] = period_us;
        } else {
            _sitlState->pwm_output[ch] = period_us;
        }
    }
}

uint16_t RCOutput::read(uint8_t ch)
{
    if (ch < SITL_NUM_CHANNELS) {
        return _sitlState->pwm_output[ch];
    }
    return 0;
}

void RCOutput::read(uint16_t* period_us, uint8_t len)
{
    memcpy(period_us, _sitlState->pwm_output, len * sizeof(uint16_t));
}

void RCOutput::cork(void)
{
    memcpy(_pending, _sitlState->pwm_output, SITL_NUM_CHANNELS * sizeof(uint16_t));
    _corked = true;
}

void RCOutput::push(void)
{
    memcpy(_sitlState->pwm_output, _pending, SITL_NUM_CHANNELS * sizeof(uint16_t));
    _corked = false;
}

#endif
