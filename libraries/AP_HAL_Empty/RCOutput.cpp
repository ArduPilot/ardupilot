
#include "RCOutput.h"

using namespace Empty;

bool EmptyRCOutput::init() {
	return true;
}

uint8_t EmptyRCOutput::get_num_channels() {
	return 8;
}

void EmptyRCOutput::set_freq(uint64_t chmask, uint16_t freq_hz) {}

uint16_t EmptyRCOutput::get_freq(uint8_t ch) {
    return 50;
}

void EmptyRCOutput::enable_ch(uint8_t ch)
{}

void EmptyRCOutput::disable_ch(uint8_t ch)
{}

void EmptyRCOutput::write(uint8_t ch, uint16_t period_us)
{}

uint16_t EmptyRCOutput::read(uint8_t ch) {
    return 900;
}
