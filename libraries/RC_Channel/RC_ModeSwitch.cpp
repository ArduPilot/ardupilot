#include <RC_Channel/RC_ModeSwitch.h>

extern const AP_HAL::HAL& hal;

bool RC_ModeSwitch::readSwitch(RC_ModeNum &ret) const
{
    uint16_t pulsewidth = hal.rcin->read(mode_channel - 1);
    if (pulsewidth <= 900 || pulsewidth >= 2200) {
        // This is an error condition
        return false;
    }
    if (pulsewidth <= 1230) {
        ret = 0;
    } else if (pulsewidth <= 1360) {
        ret = 1;
    } else if (pulsewidth <= 1490) {
        ret = 2;
    } else if (pulsewidth <= 1620) {
        ret = 3;
    } else if (pulsewidth <= 1749) {
        ret = 4;
    } else {
        ret= 5;
    }
    return true;
}
