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


#include <AP_HAL.h>
#include <AP_GPS.h>
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
    _healthy = hw_init();
    return _healthy;
}

// set_rgb - set color as a combination of red, green and blue values
void RGBLed::set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    // return immediately if not enabled
    if (!_healthy) {
        return;
    }

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


// _scheduled_update - updates _red, _green, _blue according to notify flags
void RGBLed::update_colours(void)
{
    uint8_t brightness = _led_bright;
    // slow rate from 50Hz to 10hz
    counter++;
    if (counter < 5) {
        return;
    }

    // reset counter
    counter = 0;

    // move forward one step
    step++;
    if (step >= 10) {
        step = 0;
    }

    // use dim light when connected through USB
    if (hal.gpio->usb_connected()) {
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
    // baro glitching pattern : flashing yellow and purple
    // ekf_bad pattern : flashing yellow and red
    if (AP_Notify::flags.failsafe_radio || AP_Notify::flags.failsafe_battery ||
            AP_Notify::flags.failsafe_gps || AP_Notify::flags.gps_glitching ||
            AP_Notify::flags.baro_glitching ||
            AP_Notify::flags.ekf_bad) {
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
                if (AP_Notify::flags.failsafe_gps || AP_Notify::flags.gps_glitching) {
                    // blue on for gps failsafe or glitching
                    _red_des = _led_off;
                    _blue_des = brightness;
                    _green_des = _led_off;
                } else if (AP_Notify::flags.baro_glitching) {
                    // purple on if baro glitching
                    _red_des = brightness;
                    _blue_des = brightness;
                    _green_des = _led_off;
                } else if (AP_Notify::flags.ekf_bad) {
                    // red on if ekf bad
                    _red_des = brightness;
                    _blue_des = _led_off;
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
    // return immediately if not enabled
    if (!_healthy) {
        return;
    }
    update_colours();
    set_rgb(_red_des, _green_des, _blue_des);
}
