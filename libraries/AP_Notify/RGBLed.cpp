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

bool RGBLed::init()
{
    return hw_init();
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

// set_rgb - set color as a combination of red, green and blue values
void RGBLed::set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    if (pNotify->_rgb_led_override) {
        // don't set if in override mode
        return;
    }
    _set_rgb(red, green, blue);
}


// _scheduled_update - updates _red, _green, _blue according to notify flags
void RGBLed::update_colours(void)
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

    const uint8_t step = (AP_HAL::millis()/100) % 10;

    // use dim light when connected through USB
    if (hal.gpio->usb_connected() && brightness > _led_dim) {
        brightness = _led_dim;
    }

    // initialising pattern
    if (AP_Notify::flags.initialising) {
        if (step & 1) {
            // odd steps display red light
            _red_des = brightness;
            _blue_des = _led_off;
            _green_des = _led_off;
        } else {
            // even display blue light
            _red_des = _led_off;
            _blue_des = brightness;
            _green_des = _led_off;
        }

        // exit so no other status modify this pattern
        return;
    }
    
    // save trim and esc calibration pattern
    if (AP_Notify::flags.save_trim || AP_Notify::flags.esc_calibration) {
        switch(step) {
            case 0:
            case 3:
            case 6:
                // red on
                _red_des = brightness;
                _blue_des = _led_off;
                _green_des = _led_off;
                break;

            case 1:
            case 4:
            case 7:
                // blue on
                _red_des = _led_off;
                _blue_des = brightness;
                _green_des = _led_off;
                break;

            case 2:
            case 5:
            case 8:
                // green on
                _red_des = _led_off;
                _blue_des = _led_off;
                _green_des = brightness;
                break;

            case 9:
                // all off
                _red_des = _led_off;
                _blue_des = _led_off;
                _green_des = _led_off;
                break;
        }
        // exit so no other status modify this pattern
        return;
    }

    // radio and battery failsafe patter: flash yellow
    // gps failsafe pattern : flashing yellow and blue
    // ekf_bad pattern : flashing yellow and red
    if (AP_Notify::flags.failsafe_radio || AP_Notify::flags.failsafe_battery ||
            AP_Notify::flags.ekf_bad || AP_Notify::flags.gps_glitching || AP_Notify::flags.leak_detected) {
        switch(step) {
            case 0:
            case 1:
            case 2:
            case 3:
            case 4:
                // yellow on
                _red_des = brightness;
                _blue_des = _led_off;
                _green_des = brightness;
                break;
            case 5:
            case 6:
            case 7:
            case 8:
            case 9:
                if (AP_Notify::flags.leak_detected) {
                    // purple if leak detected
                    _red_des = brightness;
                    _blue_des = brightness;
                    _green_des = brightness;
                } else if (AP_Notify::flags.ekf_bad) {
                    // red on if ekf bad
                    _red_des = brightness;
                    _blue_des = _led_off;
                    _green_des = _led_off;
                } else if (AP_Notify::flags.gps_glitching) {
                    // blue on gps glitch
                    _red_des = _led_off;
                    _blue_des = brightness;
                    _green_des = _led_off;
                }else{
                    // all off for radio or battery failsafe
                    _red_des = _led_off;
                    _blue_des = _led_off;
                    _green_des = _led_off;
                }
                break;
        }
        // exit so no other status modify this pattern
        return;
    }

    // solid green or blue if armed
    if (AP_Notify::flags.armed) {
        // solid green if armed with GPS 3d lock
        if (AP_Notify::flags.gps_status >= AP_GPS::GPS_OK_FIX_3D) {
            _red_des = _led_off;
            _blue_des = _led_off;
            _green_des = brightness;
        }else{
            // solid blue if armed with no GPS lock
            _red_des = _led_off;
            _blue_des = brightness;
            _green_des = _led_off;
        }
        return;
    }else{
        // double flash yellow if failing pre-arm checks
        if (!AP_Notify::flags.pre_arm_check) {
            switch(step) {
                case 0:
                case 1:
                case 4:
                case 5:
                    // yellow on
                    _red_des = brightness;
                    _blue_des = _led_off;
                    _green_des = brightness;
                    break;
                case 2:
                case 3:
                case 6:
                case 7:
                case 8:
                case 9:
                    // all off
                    _red_des = _led_off;
                    _blue_des = _led_off;
                    _green_des = _led_off;
                    break;
            }
        }else{
            // fast flashing green if disarmed with GPS 3D lock and DGPS
            // slow flashing green if disarmed with GPS 3d lock (and no DGPS)
            // flashing blue if disarmed with no gps lock or gps pre_arm checks have failed
            bool fast_green = AP_Notify::flags.gps_status >= AP_GPS::GPS_OK_FIX_3D_DGPS && AP_Notify::flags.pre_arm_gps_check;
            switch(step) {
                case 0:
                    if (fast_green) {
                        _green_des = brightness;
                    }
                    break;
                case 1:
                    if (fast_green) {
                        _green_des = _led_off;
                    }
                    break;
                case 2:
                    if (fast_green) {
                        _green_des = brightness;
                    }
                    break;
                case 3:
                    if (fast_green) {
                        _green_des = _led_off;
                    }
                    break;
                case 4:
                    _red_des = _led_off;
                    if (AP_Notify::flags.gps_status >= AP_GPS::GPS_OK_FIX_3D && AP_Notify::flags.pre_arm_gps_check) {
                        // flashing green if disarmed with GPS 3d lock
                        _blue_des = _led_off;
                        _green_des = brightness;
                    }else{
                        // flashing blue if disarmed with no gps lock
                        _blue_des = brightness;
                        _green_des = _led_off;
                    }
                    break;
                case 5:
                    if (fast_green) {
                        _green_des = _led_off;
                    }
                    break;

                case 6:
                    if (fast_green) {
                        _green_des = brightness;
                    }
                    break;

                case 7:
                    if (fast_green) {
                        _green_des = _led_off;
                    }
                    break;
                case 8:
                    if (fast_green) {
                        _green_des = brightness;
                    }
                    break;
                case 9:
                    // all off
                    _red_des = _led_off;
                    _blue_des = _led_off;
                    _green_des = _led_off;
                    break;
            }
        }
    }
}

// update - updates led according to timed_updated.  Should be called
// at 50Hz
void RGBLed::update()
{
    if (!pNotify->_rgb_led_override) {
        update_colours();
        set_rgb(_red_des, _green_des, _blue_des);
    } else {
        update_override();
    }
}

/*
  handle LED control, only used when LED_OVERRIDE=1
*/
void RGBLed::handle_led_control(mavlink_message_t *msg)
{
    if (!pNotify->_rgb_led_override) {
        // ignore LED_CONTROL commands if not in LED_OVERRIDE mode
        return;
    }

    // decode mavlink message
    mavlink_led_control_t packet;
    mavlink_msg_led_control_decode(msg, &packet);

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
