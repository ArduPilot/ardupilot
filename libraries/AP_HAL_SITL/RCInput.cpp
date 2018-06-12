#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "RCInput.h"

using namespace HALSITL;

extern const AP_HAL::HAL& hal;

void RCInput::init()
{
}

bool RCInput::new_input()
{
    if (_sitlState->new_rc_input) {
        _sitlState->new_rc_input = false;
        return true;
    }
    return false;
}

uint16_t RCInput::read(uint8_t ch)
{
    if (ch >= SITL_RC_INPUT_CHANNELS) {
        return 0;
    }
    return _sitlState->pwm_input[ch];
}

uint8_t RCInput::read(uint16_t* periods, uint8_t len)
{
    if (len > SITL_RC_INPUT_CHANNELS) {
        len = SITL_RC_INPUT_CHANNELS;
    }
    for (uint8_t i=0; i < len; i++) {
        periods[i] = read(i);
    }
    return len;
}

#endif
