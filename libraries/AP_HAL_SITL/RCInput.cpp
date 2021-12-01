#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && !defined(HAL_BUILD_AP_PERIPH)

#include "RCInput.h"
#include <SITL/SITL.h>
#include <AP_RCProtocol/AP_RCProtocol.h>

#ifdef SFML_JOYSTICK
  #ifdef HAVE_SFML_GRAPHICS_HPP
    #include <SFML/Window/Joystick.hpp>
  #elif HAVE_SFML_GRAPHIC_H
    #include <SFML/Window/Joystick.h>
  #endif
#endif // SFML_JOYSTICK

using namespace HALSITL;

extern const AP_HAL::HAL& hal;

void RCInput::init()
{
    AP::RC().init();
}

bool RCInput::new_input()
{
    if (!using_rc_protocol) {
        if (AP::RC().new_input()) {
            using_rc_protocol = true;
        }
    }
    if (using_rc_protocol) {
        return AP::RC().new_input();
    }
    if (_sitlState->new_rc_input) {
        _sitlState->new_rc_input = false;
        return true;
    }
    return false;
}

uint16_t RCInput::read(uint8_t ch)
{
    if (using_rc_protocol) {
        return AP::RC().read(ch);
    }
    if (ch >= num_channels()) {
        return 0;
    }
#ifdef SFML_JOYSTICK
    SITL::SIM *_sitl = AP::sitl();
    if (_sitl) {
        const sf::Joystick::Axis axis = sf::Joystick::Axis(_sitl->sfml_joystick_axis[ch].get());
        const unsigned int stickID = _sitl->sfml_joystick_id;
        if (sf::Joystick::hasAxis(stickID, axis)) {
            return (constrain_float(sf::Joystick::getAxisPosition(stickID, axis) + 100, 0, 200) * 5) + 1000;
        } else {
            return 0;
        }
    } else {
      return 0;
    }
#else
    return _sitlState->pwm_input[ch];
#endif
}

uint8_t RCInput::read(uint16_t* periods, uint8_t len)
{
    if (len > num_channels()) {
        len = num_channels();
    }
    for (uint8_t i=0; i < len; i++) {
        periods[i] = read(i);
    }
    return len;
}

uint8_t RCInput::num_channels()
{
    if (using_rc_protocol) {
        return AP::RC().num_channels();
    }
    SITL::SIM *_sitl = AP::sitl();
    if (_sitl) {
#ifdef SFML_JOYSTICK
        return (sf::Joystick::isConnected(_sitl->sfml_joystick_id.get())) ? ARRAY_SIZE(_sitl->sfml_joystick_axis) : 0;
#else
        return MIN(_sitl->rc_chancount.get(), SITL_RC_INPUT_CHANNELS);
#endif // SFML_JOYSTICK
    }
    return SITL_RC_INPUT_CHANNELS;
}

#endif
