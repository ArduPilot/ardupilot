#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 && \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_PX4_AEROFC_V1

#include "RCOutput_Tap.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

extern const AP_HAL::HAL &hal;

using namespace PX4;

void RCOutput_Tap::init()
{
    _perf_rcout = perf_alloc(PC_ELAPSED, "APM_rcout");
}

/*
  set output frequency
 */
void RCOutput_Tap::set_freq(uint32_t chmask, uint16_t freq_hz)
{
}

uint16_t RCOutput_Tap::get_freq(uint8_t ch)
{
    return 400;
}

void RCOutput_Tap::enable_ch(uint8_t ch)
{
    if (ch >= MAX_MOTORS) {
        return;
    }
    _enabled_channels |= (1U << ch);
    if (_period[ch] == UINT16_MAX) {
        _period[ch] = 0;
    }
}

void RCOutput_Tap::disable_ch(uint8_t ch)
{
    if (ch >= MAX_MOTORS) {
        return;
    }

    _enabled_channels &= ~(1U << ch);
    _period[ch] = UINT16_MAX;
}

void RCOutput_Tap::write(uint8_t ch, uint16_t period_us)
{
    if (ch >= MAX_MOTORS) {
        return;
    }
    if (!(_enabled_channels & (1U << ch))) {
        // not enabled
        return;
    }

    _period[ch] = period_us;
}

uint16_t RCOutput_Tap::read(uint8_t ch)
{
    if (ch >= MAX_MOTORS) {
        return 0;
    }
    return _period[ch];
}

void RCOutput_Tap::read(uint16_t *period_us, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++) {
        period_us[i] = read(i);
    }
}

void RCOutput_Tap::_send_outputs(void)
{
    uint32_t __attribute__((unused)) now = AP_HAL::micros();

    perf_begin(_perf_rcout);

    perf_end(_perf_rcout);
}

void RCOutput_Tap::cork()
{
    _corking = true;
}

void RCOutput_Tap::push()
{
    _corking = false;
    _send_outputs();
}

#endif
