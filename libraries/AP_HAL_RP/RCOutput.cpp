
#include "RCOutput.h"
#include <AP_Math/AP_Math.h>

using namespace Empty;

void RCOutput::init() {}

void RCOutput::set_freq(uint32_t chmask, uint16_t freq_hz) {}

uint16_t RCOutput::get_freq(uint8_t chan) {
    return 50;
}

void RCOutput::enable_ch(uint8_t chan)
{}

void RCOutput::disable_ch(uint8_t chan)
{}

void RCOutput::write(uint8_t chan, uint16_t period_us)
{
    if (chan < ARRAY_SIZE(value)) {
        value[chan] = period_us;
    }
}

uint16_t RCOutput::read(uint8_t chan)
{
    if (chan < ARRAY_SIZE(value)) {
        return value[chan];
    }
    return 900;
}

void RCOutput::read(uint16_t* period_us, uint8_t len)
{
    len = MIN(len, ARRAY_SIZE(value));
    memcpy(period_us, value, len*sizeof(value[0]));
}

