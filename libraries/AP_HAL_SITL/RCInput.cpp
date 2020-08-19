#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "RCInput.h"
#include <SITL/SITL.h>
#include <AP_RCProtocol/AP_RCProtocol.h>

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
    return _sitlState->pwm_input[ch];
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
    SITL::SITL *_sitl = AP::sitl();
    if (_sitl) {
        return MIN(_sitl->rc_chancount.get(), SITL_RC_INPUT_CHANNELS);
    }
    return SITL_RC_INPUT_CHANNELS;
}

#endif
