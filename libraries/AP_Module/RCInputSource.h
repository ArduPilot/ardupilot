// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <stdint.h>

namespace AP_Module
{

// Host-owned publication state survives module exit. The producer supplies
// validity and failsafe policy; no radio protocol is interpreted here.
class RCInputSource
{
public:
    bool accept(uint32_t now, uint8_t count, uint32_t frame, uint32_t input,
                uint32_t valid_until, uint32_t timeout, uint32_t repeat)
    {
        if (count == 0 || count > 16 || (frame == _frame && input == _input) ||
            int32_t(now - valid_until) > 0) {
            return false;
        }
        _seen = true;
        _failed = false;
        _input = input;
        _frame = frame;
        _timeout = timeout;
        _repeat = repeat;
        return true;
    }

    bool failsafe_due(uint32_t now)
    {
        if (!_seen || _timeout == 0 || (!_failed && now - _input <= _timeout) ||
            (_failed && now - _reported < _repeat)) {
            return false;
        }
        _failed = true;
        _reported = now;
        return true;
    }

private:
    bool _seen = false, _failed = false;
    uint32_t _frame = 0, _input = 0, _timeout = 0, _repeat = 0, _reported = 0;
};

}
