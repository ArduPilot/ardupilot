#include <AP_HAL/AP_HAL.h>
#include <AP_RCProtocol/AP_RCProtocol_config.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && AP_RCPROTOCOL_ENABLED

#include "RCInput.h"
#include <AP_RCProtocol/AP_RCProtocol.h>

using namespace HALSITL;

extern const AP_HAL::HAL& hal;

void RCInput::init()
{
    AP::RC().init();
}

bool RCInput::new_input()
{
    return AP::RC().new_input();
}

uint16_t RCInput::read(uint8_t ch)
{
    return AP::RC().read(ch);
}

uint8_t RCInput::read(uint16_t* periods, uint8_t len)
{
    AP::RC().read(periods, len);
    return MIN(len, num_channels());
}

uint8_t RCInput::num_channels()
{
    return AP::RC().num_channels();
}

#endif
