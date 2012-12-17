
#include "RCOutput.h"

using namespace Empty;

void EmptyRCOutput::init(void* machtnichts) {}

void EmptyRCOutput::set_freq(uint32_t chmask, uint16_t freq_hz) {}

uint16_t EmptyRCOutput::get_freq(uint8_t ch) {
    return 50;
}

void EmptyRCOutput::enable_ch(uint8_t ch)
{}

void EmptyRCOutput::enable_mask(uint32_t chmask)
{}

void EmptyRCOutput::disable_ch(uint8_t ch)
{}

void EmptyRCOutput::disable_mask(uint32_t chmask)
{}

void EmptyRCOutput::write(uint8_t ch, uint16_t period_us)
{}

void EmptyRCOutput::write(uint8_t ch, uint16_t* period_us, uint8_t len)
{}

uint16_t EmptyRCOutput::read(uint8_t ch) {
    return 900;
}

void EmptyRCOutput::read(uint16_t* period_us, uint8_t len)
{}

