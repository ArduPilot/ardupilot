
#include "RCInput.h"

using namespace Empty;
RCInput::RCInput()
{}

void RCInput::init()
{}

bool RCInput::new_input() {
    return false;
}

uint8_t RCInput::num_channels() {
    return 0;
}

uint16_t RCInput::read(uint8_t chan) {
    if (chan == 2) return 900; /* throttle should be low, for safety */
    else return 1500;
}

uint8_t RCInput::read(uint16_t* periods, uint8_t len) {
    for (uint8_t i = 0; i < len; i++){
        if (i == 2) periods[i] = 900;
        else periods[i] = 1500;
    }
    return len;
}

