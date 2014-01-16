#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "RCOutput.h"

using namespace Linux;

void LinuxRCOutput::init(void* machtnichts) {}

void LinuxRCOutput::set_freq(uint32_t chmask, uint16_t freq_hz) {}

uint16_t LinuxRCOutput::get_freq(uint8_t ch) {
    return 50;
}

void LinuxRCOutput::enable_ch(uint8_t ch)
{}

void LinuxRCOutput::disable_ch(uint8_t ch)
{}

void LinuxRCOutput::write(uint8_t ch, uint16_t period_us)
{}

void LinuxRCOutput::write(uint8_t ch, uint16_t* period_us, uint8_t len)
{}

uint16_t LinuxRCOutput::read(uint8_t ch) {
    return 900;
}

void LinuxRCOutput::read(uint16_t* period_us, uint8_t len)
{}

#endif // CONFIG_HAL_BOARD
