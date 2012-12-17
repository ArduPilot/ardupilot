
#include "RCInput.h"

using namespace Empty;
EmptyRCInput::EmptyRCInput()
{}

void EmptyRCInput::init(void* machtnichts)
{}

uint8_t EmptyRCInput::valid() {
    return 0;
}

uint16_t EmptyRCInput::read(uint8_t ch) {
    if (ch == 3) return 900; /* throttle should be low, for safety */
    else return 1500;
}

uint8_t EmptyRCInput::read(uint16_t* periods, uint8_t len) {
    for (uint8_t i = 0; i < len; i++){
        if (i == 3) periods[i] = 900;
        else periods[i] = 1500;
    }
    return len;
}

bool EmptyRCInput::set_overrides(int16_t *overrides, uint8_t len) {
    return true;
}

bool EmptyRCInput::set_override(uint8_t channel, int16_t override) {
    return true;
}

void EmptyRCInput::clear_overrides()
{}

