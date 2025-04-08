#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_JOYSTICK_SFML_ENABLED

#include "AP_RCProtocol_Joystick_SFML.h"

#include <SITL/SITL.h>

#ifdef HAVE_SFML_GRAPHICS_HPP
#include <SFML/Window/Joystick.hpp>
#elif HAVE_SFML_GRAPHIC_H
#include <SFML/Window/Joystick.h>
#endif

void AP_RCProtocol_Joystick_SFML::update()
{
    auto *_sitl = AP::sitl();
    if (_sitl == nullptr) {
        return;
    }

    sf::Joystick::update();

    const unsigned int stick_id = _sitl->sfml_joystick_id;
    if (!sf::Joystick::isConnected(stick_id)) {
        return;
    }

    uint16_t pwm_values[ARRAY_SIZE(_sitl->sfml_joystick_axis)]{};
    for (uint8_t ch=0; ch<ARRAY_SIZE(_sitl->sfml_joystick_axis); ch++) {
        const sf::Joystick::Axis axis = sf::Joystick::Axis(_sitl->sfml_joystick_axis[ch].get());
        if (!sf::Joystick::hasAxis(stick_id, axis)) {
            continue;
        }

        // pos is a value between -100 and 100:
        const auto pos = sf::Joystick::getAxisPosition(stick_id, axis);

        // convert to a "pwm" value between 1000 and 2000:
        const uint16_t pwm = (constrain_float(pos + 100, 0, 200) * 5) + 1000;
        pwm_values[ch] = pwm;
    }

    // never in failsafe:
    add_input(ARRAY_SIZE(pwm_values), pwm_values, false);
}

#endif  // AP_RCPROTOCOL_JOYSTICK_SFML_ENABLED
