#include "AP_Scripting/AP_Scripting_config.h"

#if AP_SCRIPTING_ENABLED

#include <limits>
#include "script_button.h"

void ScriptButton::press()
{
    if (!pressed) {
        pressed = true;

        // The count will max out at 255, but it won't roll over to 0.
        if (count < std::numeric_limits<uint8_t>::max()) {
            count++;
        }
    }
}

void ScriptButton::release()
{
    pressed = false;
}

bool ScriptButton::is_pressed() const
{
    return pressed;
}

uint8_t ScriptButton::get_count() const
{
    return count;
}

void ScriptButton::clear_count()
{
    count = 0;
}

uint8_t ScriptButton::get_and_clear_count()
{
    auto result = get_count();
    clear_count();
    return result;
}

#endif // AP_SCRIPTING_ENABLED
