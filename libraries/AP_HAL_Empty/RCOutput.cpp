
#include "RCOutput.h"

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
{}

uint16_t RCOutput::read(uint8_t chan) {
    return 900;
}

void RCOutput::read(uint16_t* period_us, uint8_t len)
{}

