
#include "RCInput.h"

using namespace Empty;
EmptyRCInput::EmptyRCInput()
{
  init_overrides(NULL, 0);
}

void EmptyRCInput::init(void* machtnichts)
{
}

uint8_t EmptyRCInput::valid_channels() {
    return 0;
}

uint16_t EmptyRCInput::read(uint8_t ch) {
    if (ch == 2) return 900; /* throttle should be low, for safety */
    else return 1500;
}

uint8_t EmptyRCInput::read(uint16_t* periods, uint8_t len) {
    for (uint8_t i = 0; i < len; i++){
        if (i == 2) periods[i] = 900;
        else periods[i] = 1500;
    }
    return len;
}


