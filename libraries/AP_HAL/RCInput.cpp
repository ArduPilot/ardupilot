#include <AP_HAL.h>

#include <avr/io.h>
#include <avr/interrupt.h>

#include "RCInput.h"
using namespace AP_HAL;

void RCInput::init_overrides(uint16_t *buffer, uint8_t max_channels) {
    _overrides = buffer;
    this->max_channels = max_channels;
}

bool RCInput::set_overrides(int16_t *overrides, uint8_t len) {
    bool res = false;
    for (uint8_t i = 0; i < len; i++) {
        res |= set_override(i, overrides[i]);
    }
    return res;
}

bool RCInput::set_override(uint8_t channel, int16_t override) {
    if (override < 0) return _overrides[channel] != 0; /* -1: no change. */
    if (channel < max_channels) {
        _overrides[channel] = override;
        if (override != 0) {
            set_overrides_valid();
            return true;
        }
    }
    return false;
}

void RCInput::clear_overrides() {
    for (uint8_t i = 0; i < max_channels; i++) {
        _overrides[i] = 0;
    }
}

