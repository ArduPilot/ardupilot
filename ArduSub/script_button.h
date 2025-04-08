#pragma once

#if AP_SCRIPTING_ENABLED

#include <AP_Common/AP_Common.h>

// Joystick button object for use in Lua scripts.
//
// Provide 2 ways to use a joystick button:
//      is_pressed() returns true if the button is currently (as of the most recent MANUAL_CONTROL msg) pressed
//      get_and_clear_count() returns the number of times the button was pressed since the last call
//
class ScriptButton {
public:
    ScriptButton(): pressed(false), count(0) {}

    void press();

    void release();

    bool is_pressed() const WARN_IF_UNUSED;

    uint8_t get_count() const WARN_IF_UNUSED;

    void clear_count();

    uint8_t get_and_clear_count();

private:
    bool pressed;
    uint8_t count;
};

#endif // AP_SCRIPTING_ENABLED
