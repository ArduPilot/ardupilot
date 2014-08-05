#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_YUNEEC

#include "RCOutput.h"

using namespace YUNEEC;

void YUNEECRCOutput::init(void* machtnichts) {}

void YUNEECRCOutput::set_freq(uint32_t chmask, uint16_t freq_hz) {}

uint16_t YUNEECRCOutput::get_freq(uint8_t ch) {
    return 50;
}

void YUNEECRCOutput::enable_ch(uint8_t ch)
{}

void YUNEECRCOutput::disable_ch(uint8_t ch)
{}

void YUNEECRCOutput::write(uint8_t ch, uint16_t period_us)
{}

void YUNEECRCOutput::write(uint8_t ch, uint16_t* period_us, uint8_t len)
{}

uint16_t YUNEECRCOutput::read(uint8_t ch) {
    return 900;
}

void YUNEECRCOutput::read(uint16_t* period_us, uint8_t len)
{}

#endif
