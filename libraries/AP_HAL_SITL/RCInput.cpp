#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "RCInput.h"

using namespace HALSITL;

extern const AP_HAL::HAL& hal;

void RCInput::init()
{
    clear_overrides();
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
    if (_override[ch]) {
        return _override[ch];
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

bool RCInput::set_overrides(int16_t *overrides, uint8_t len)
{
    bool res = false;
    if (len > SITL_RC_INPUT_CHANNELS) {
        len = SITL_RC_INPUT_CHANNELS;
    }
    for (uint8_t i = 0; i < len; i++) {
        res |= set_override(i, overrides[i]);
    }
    return res;
}

bool RCInput::set_override(uint8_t channel, int16_t override)
{
    if (override < 0) {
        return false;  /* -1: no change. */
    }
    if (channel < SITL_RC_INPUT_CHANNELS) {
        _override[channel] = static_cast<uint16_t>(override);
        if (override != 0) {
            return true;
        }
    }
    return false;
}

void RCInput::clear_overrides()
{
    memset(_override, 0, sizeof(_override));
}
#endif
