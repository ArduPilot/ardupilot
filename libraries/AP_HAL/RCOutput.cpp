#include "AP_HAL.h"

extern const AP_HAL::HAL &hal;

uint32_t AP_HAL::RCOutput::DSHOT_BIT_WIDTH_TICKS = DSHOT_BIT_WIDTH_TICKS_DEFAULT;
uint32_t AP_HAL::RCOutput::DSHOT_BIT_0_TICKS = DSHOT_BIT_0_TICKS_DEFAULT;
uint32_t AP_HAL::RCOutput::DSHOT_BIT_1_TICKS = DSHOT_BIT_1_TICKS_DEFAULT;

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
    case MODE_PROFILED:
        return "ProfiLED";
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

/*
  true when the output mode is of type dshot
*/
bool AP_HAL::RCOutput::is_dshot_protocol(const enum output_mode mode)
{
    switch (mode) {
    case MODE_PWM_DSHOT150:
    case MODE_PWM_DSHOT300:
    case MODE_PWM_DSHOT600:
    case MODE_PWM_DSHOT1200:
        return true;
    default:
        return false;
    }
}

/*
 * calculate the prescaler required to achieve the desire bitrate
 */
uint32_t AP_HAL::RCOutput::calculate_bitrate_prescaler(uint32_t timer_clock, uint32_t target_frequency, bool is_dshot)
{
    if (target_frequency > timer_clock) {
        // we can't achieve the desired frequency
        return 0;
    }

    uint32_t prescaler;

    if (is_dshot) {
        // original prescaler calculation from betaflight. bi-dir dshot is incredibly sensitive to the bitrate
        prescaler = uint32_t(lrintf((float) timer_clock / target_frequency + 0.01f) - 1);
    } else {
        // adjust frequency to give an allowed value given the clock, erring on the high side
        prescaler = timer_clock / target_frequency;
        while ((timer_clock / prescaler) < target_frequency && prescaler > 1) {
            prescaler--;
        }
        // prescaler is divisor - 1
        prescaler--;
    }

    if (!is_dshot) {
        const uint32_t freq = timer_clock / (prescaler + 1);
        // find the closest value
        const float delta = fabsf(float(freq) - target_frequency);
        if (freq > target_frequency
            && delta > fabsf(float(timer_clock / (prescaler+2)) - target_frequency)) {
            prescaler++;
        } else if (prescaler > 0 && freq < target_frequency
            && delta > fabsf(float(timer_clock / (prescaler)) - target_frequency)) {
            prescaler--;
        }
    }

    return prescaler;
}

