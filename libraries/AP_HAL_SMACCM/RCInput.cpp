
#include "RCInput.h"

using namespace SMACCM;
SMACCMRCInput::SMACCMRCInput()
{}

void SMACCMRCInput::init(void* machtnichts)
{}

uint8_t SMACCMRCInput::valid() {
    return 0;
}

uint16_t SMACCMRCInput::read(uint8_t ch) {
    if (ch == 3) return 900; /* throttle should be low, for safety */
    else return 1500;
}

uint8_t SMACCMRCInput::read(uint16_t* periods, uint8_t len) {
    for (uint8_t i = 0; i < len; i++){
        if (i == 3) periods[i] = 900;
        else periods[i] = 1500;
    }
    return len;
}

bool SMACCMRCInput::set_overrides(int16_t *overrides, uint8_t len) {
    return true;
}

bool SMACCMRCInput::set_override(uint8_t channel, int16_t override) {
    return true;
}

void SMACCMRCInput::clear_overrides()
{}

