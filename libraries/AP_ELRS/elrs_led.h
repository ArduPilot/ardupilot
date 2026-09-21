#pragma once
#include <stdint.h>

namespace ELRS
{
class StatusLED
{
public:
    enum class State : uint8_t { OFF, SEARCHING, CONNECTED, BINDING, FAILED };
    bool update(State state, uint32_t now_ms)
    {
        if (state != _state) {
            _state = state;
            _start_ms = now_ms;
        }
        const uint32_t elapsed = now_ms - _start_ms;
        switch (state) {
        case State::CONNECTED:
            return true;
        case State::SEARCHING:
            return elapsed % 1000 < 500;
        case State::BINDING: {
            const uint32_t phase = elapsed % 1300;
            return phase < 100 || (phase >= 200 && phase < 300);
        }
        case State::FAILED:
            return elapsed % 1200 < 200;
        case State::OFF:
            return false;
        }
        return false;
    }
private:
    State _state = State::OFF;
    uint32_t _start_ms = 0;
};
} // namespace ELRS
