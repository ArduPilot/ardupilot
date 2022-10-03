/*
   Generic RGBLed driver
*/

/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

*/


#include <AP_HAL/AP_HAL.h>
#include <AP_GPS/AP_GPS.h>
#include "RGBLed.h"
#include "AP_Notify.h"

extern const AP_HAL::HAL& hal;

RGBLed::RGBLed(uint8_t led_off, uint8_t led_bright, uint8_t led_medium, uint8_t led_dim):
    _led_off(led_off),
    _led_bright(led_bright),
    _led_medium(led_medium),
    _led_dim(led_dim)
{

}

// set_rgb - set color as a combination of red, green and blue values
void RGBLed::_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    if (red != _red_curr ||
        green != _green_curr ||
        blue != _blue_curr) {
        // call the hardware update routine
        if (hw_set_rgb(red, green, blue)) {
            _red_curr = red;
            _green_curr = green;
            _blue_curr = blue;
        }
    }
}

RGBLed::rgb_source_t RGBLed::rgb_source() const
{
    return rgb_source_t(pNotify->_rgb_led_override.get());
}

// set_rgb - set color as a combination of red, green and blue values
void RGBLed::set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    if (rgb_source() == mavlink) {
        // don't set if in override mode
        return;
    }
    _set_rgb(red, green, blue);
}

uint8_t RGBLed::get_brightness(void) const
{
    uint8_t brightness = _led_bright;

    switch (pNotify->_rgb_led_brightness) {
    case RGB_LED_OFF:
        brightness = _led_off;
        break;
    case RGB_LED_LOW:
        brightness = _led_dim;
        break;
    case RGB_LED_MEDIUM:
        brightness = _led_medium;
        break;
    case RGB_LED_HIGH:
        brightness = _led_bright;
        break;
    }

    // use dim light when connected through USB
    if (hal.gpio->usb_connected() && brightness > _led_dim) {
        brightness = _led_dim;
    }
    return brightness;
}

uint32_t RGBLed::get_colour_sequence_obc(void) const
{
    if (AP_Notify::flags.armed) {
        return DEFINE_COLOUR_SEQUENCE_SOLID(RED);
    }
    return DEFINE_COLOUR_SEQUENCE_SOLID(GREEN);
}

// _scheduled_update - updates _red, _green, _blue according to notify flags
uint32_t RGBLed::get_colour_sequence(void) const
{
    // initialising pattern
    if (AP_Notify::flags.initialising) {
        return sequence_initialising;
    }

    // save trim or any calibration pattern
    if (AP_Notify::flags.esc_calibration ||
        AP_Notify::flags.compass_cal_running ||
        AP_Notify::flags.temp_cal_running) {
        return sequence_trim_or_esc;
    }

    // radio and battery failsafe patter: flash yellow
    // gps failsafe pattern : flashing yellow and blue
    // ekf_bad pattern : flashing yellow and red
    if (AP_Notify::flags.failsafe_radio ||
        AP_Notify::flags.failsafe_gcs ||
        AP_Notify::flags.failsafe_battery ||
        AP_Notify::flags.ekf_bad ||
        AP_Notify::flags.gps_glitching ||
        AP_Notify::flags.leak_detected) {

        if (AP_Notify::flags.leak_detected) {
            // purple if leak detected
            return sequence_failsafe_leak;
        } else if (AP_Notify::flags.ekf_bad) {
            // red on if ekf bad
            return sequence_failsafe_ekf;
        } else if (AP_Notify::flags.gps_glitching) {
            // blue on gps glitch
            return sequence_failsafe_gps_glitching;
        }
        // all off for radio or battery failsafe
        return sequence_failsafe_radio_or_battery;
    }

    // solid green or blue if armed
    if (AP_Notify::flags.armed) {
        // solid green if armed with GPS 3d lock
        if (AP_Notify::flags.gps_status >= AP_GPS::GPS_OK_FIX_3D) {
            return sequence_armed;
        }
        // solid blue if armed with no GPS lock
        return sequence_armed_nogps;
    }

    // double flash yellow if failing pre-arm checks
    if (!AP_Notify::flags.pre_arm_check) {
        return sequence_prearm_failing;
    }
    if (AP_Notify::flags.gps_status >= AP_GPS::GPS_OK_FIX_3D_DGPS && AP_Notify::flags.pre_arm_gps_check) {
        return sequence_disarmed_good_dgps;
    }

    if (AP_Notify::flags.gps_status >= AP_GPS::GPS_OK_FIX_3D && AP_Notify::flags.pre_arm_gps_check) {
        return sequence_disarmed_good_gps;
    }

    return sequence_disarmed_bad_gps;
}

uint32_t RGBLed::get_colour_sequence_traffic_light(void) const
{
    if (AP_Notify::flags.initialising) {
        return DEFINE_COLOUR_SEQUENCE(RED,GREEN,BLUE,RED,GREEN,BLUE,RED,GREEN,BLUE,BLACK);
    }

    if (AP_Notify::flags.armed) {
        return DEFINE_COLOUR_SEQUENCE_SLOW(RED);
    }

    if (hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED) {
        if (!AP_Notify::flags.pre_arm_check) {
            return DEFINE_COLOUR_SEQUENCE_ALTERNATE(YELLOW, BLACK);
        } else {
            return DEFINE_COLOUR_SEQUENCE_SLOW(YELLOW);
        }
    }

    if (!AP_Notify::flags.pre_arm_check) {
        return DEFINE_COLOUR_SEQUENCE_ALTERNATE(GREEN, BLACK);
    }
    return DEFINE_COLOUR_SEQUENCE_SLOW(GREEN);
}

// update - updates led according to timed_updated.  Should be called
// at 50Hz
void RGBLed::update()
{
    uint32_t current_colour_sequence = 0;

    switch (rgb_source()) {
    case mavlink:
        update_override();
        return; // note this is a return not a break!
    case standard:
        current_colour_sequence = get_colour_sequence();
        break;
    case obc:
        current_colour_sequence = get_colour_sequence_obc();
        break;
    case traffic_light:
        current_colour_sequence = get_colour_sequence_traffic_light();
        break;
    }

    const uint8_t brightness = get_brightness();

    uint8_t step = (AP_HAL::millis()/100) % 10;

    // ensure we can't skip a step even with awful timing
    if (step != last_step) {
        step = (last_step+1) % 10;
        last_step = step;
    }

    const uint8_t colour = (current_colour_sequence >> (step*3)) & 7;

    uint8_t red_des = (colour & RED) ? brightness : _led_off;
    uint8_t green_des = (colour & GREEN) ? brightness : _led_off;
    uint8_t blue_des = (colour & BLUE) ? brightness : _led_off;

    set_rgb(red_des, green_des, blue_des);
}

/*
  handle LED control, only used when LED_OVERRIDE=1
*/
void RGBLed::handle_led_control(const mavlink_message_t &msg)
{
    if (rgb_source() != mavlink) {
        // ignore LED_CONTROL commands if not in LED_OVERRIDE mode
        return;
    }

    // decode mavlink message
    mavlink_led_control_t packet;
    mavlink_msg_led_control_decode(&msg, &packet);

    _led_override.start_ms = AP_HAL::millis();

    switch (packet.custom_len) {
    case 3:
        _led_override.rate_hz = 0;
        _led_override.r = packet.custom_bytes[0];
        _led_override.g = packet.custom_bytes[1];
        _led_override.b = packet.custom_bytes[2];
        break;
    case 4:
        _led_override.rate_hz = packet.custom_bytes[3];
        _led_override.r = packet.custom_bytes[0];
        _led_override.g = packet.custom_bytes[1];
        _led_override.b = packet.custom_bytes[2];
        break;
    default:
        // not understood
        break;
    }
}

/*
  update LED when in override mode
 */
void RGBLed::update_override(void)
{
    if (_led_override.rate_hz == 0) {
        // solid colour
        _set_rgb(_led_override.r, _led_override.g, _led_override.b);
        return;
    }
    // blinking
    uint32_t ms_per_cycle = 1000 / _led_override.rate_hz;
    uint32_t cycle = (AP_HAL::millis() - _led_override.start_ms) % ms_per_cycle;
    if (cycle > ms_per_cycle / 2) {
        // on
        _set_rgb(_led_override.r, _led_override.g, _led_override.b);
    } else {
        _set_rgb(0, 0, 0);
    }
}

/*
  RGB control
  give RGB and flash rate, used with scripting
*/
void RGBLed::rgb_control(uint8_t r, uint8_t g, uint8_t b, uint8_t rate_hz)
{
    _led_override.rate_hz = rate_hz;
    _led_override.r = r;
    _led_override.g = g;
    _led_override.b = b;
}
