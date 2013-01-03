
#include "RCOutput.h"

using namespace SMACCM;

void SMACCMRCOutput::init(void* machtnichts) {}

void SMACCMRCOutput::set_freq(uint32_t chmask, uint16_t freq_hz) {}

uint16_t SMACCMRCOutput::get_freq(uint8_t ch) {
    return 50;
}

void SMACCMRCOutput::enable_ch(uint8_t ch)
{}

void SMACCMRCOutput::enable_mask(uint32_t chmask)
{}

void SMACCMRCOutput::disable_ch(uint8_t ch)
{}

void SMACCMRCOutput::disable_mask(uint32_t chmask)
{}

void SMACCMRCOutput::write(uint8_t ch, uint16_t period_us)
{}

void SMACCMRCOutput::write(uint8_t ch, uint16_t* period_us, uint8_t len)
{}

uint16_t SMACCMRCOutput::read(uint8_t ch) {
    return 900;
}

void SMACCMRCOutput::read(uint16_t* period_us, uint8_t len)
{}

