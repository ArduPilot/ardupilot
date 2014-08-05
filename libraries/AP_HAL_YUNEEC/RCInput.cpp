#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_YUNEEC

#include "RCInput.h"

using namespace YUNEEC;
YUNEECRCInput::YUNEECRCInput()
{}

void YUNEECRCInput::init(void* machtnichts)
{}

bool YUNEECRCInput::new_input() {
    return false;
}

uint8_t YUNEECRCInput::num_channels() {
    return 0;
}

uint16_t YUNEECRCInput::read(uint8_t ch) {
    if (ch == 2) return 900; /* throttle should be low, for safety */
    else return 1500;
}

uint8_t YUNEECRCInput::read(uint16_t* periods, uint8_t len) {
    for (uint8_t i = 0; i < len; i++){
        if (i == 2) periods[i] = 900;
        else periods[i] = 1500;
    }
    return len;
}

bool YUNEECRCInput::set_overrides(int16_t *overrides, uint8_t len) {
    return true;
}

bool YUNEECRCInput::set_override(uint8_t channel, int16_t override) {
    return true;
}

void YUNEECRCInput::clear_overrides()
{}

#endif
