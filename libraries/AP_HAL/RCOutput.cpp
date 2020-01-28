#include "AP_HAL.h"

extern const AP_HAL::HAL &hal;

// helper function for implementation of get_output_mode_banner
const char* AP_HAL::RCOutput::get_output_mode_string(enum output_mode out_mode) const
{
    // convert mode to string
    switch (out_mode) {
    case MODE_PWM_NONE:
        return "None";
    case MODE_PWM_NORMAL:
        return "PWM";
    case MODE_PWM_ONESHOT:
        return "OneS";
    case MODE_PWM_ONESHOT125:
        return "OS125";
    case MODE_PWM_BRUSHED:
        return "Brush";
    case MODE_PWM_DSHOT150:
        return "DS150";
    case MODE_PWM_DSHOT300:
        return "DS300";
    case MODE_PWM_DSHOT600:
        return "DS600";
    case MODE_PWM_DSHOT1200:
        return "DS1200";
    case MODE_NEOPIXEL:
        return "NeoP";
    }

    // we should never reach here but just in case
    return "Unknown";
}

// convert output mode to string.  helper function for implementation of get_output_mode_banner
void AP_HAL::RCOutput::append_to_banner(char banner_msg[], uint8_t banner_msg_len, output_mode out_mode, uint8_t low_ch, uint8_t high_ch) const
{
    const char* mode_str = get_output_mode_string(out_mode);

    // make copy of banner_msg
    char banner_msg_temp[banner_msg_len];
    memcpy(banner_msg_temp, banner_msg, banner_msg_len);

    if (low_ch == high_ch) {
        // handle single channel case
        hal.util->snprintf(banner_msg, banner_msg_len, "%s %s:%u", banner_msg_temp, mode_str, (unsigned)low_ch);
    } else {
        // the general case
        hal.util->snprintf(banner_msg, banner_msg_len, "%s %s:%u-%u", banner_msg_temp, mode_str, (unsigned)low_ch, (unsigned)high_ch);
    }
}
