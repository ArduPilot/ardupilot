/*
   ToshibaLED driver
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

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL.h>
#include "ToshibaLED.h"
#include "AP_Notify.h"

extern const AP_HAL::HAL& hal;

void ToshibaLED::init()
{
    _healthy = hw_init();
}

// set_rgb - set color as a combination of red, green and blue values
void ToshibaLED::set_rgb(uint8_t red, uint8_t green, uint8_t blue)
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
void ToshibaLED::update_colours(void)
{
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

    // initialising pattern
    if (AP_Notify::flags.initialising) {
        if (step & 1) {
            // odd steps display red light
            _red_des = TOSHIBA_LED_DIM;
            _blue_des = TOSHIBA_LED_OFF;
            _green_des = TOSHIBA_LED_OFF;
        } else {
            // even display blue light
            _red_des = TOSHIBA_LED_OFF;
            _blue_des = TOSHIBA_LED_DIM;
            _green_des = TOSHIBA_LED_OFF;
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
                _red_des = TOSHIBA_LED_DIM;
                _blue_des = TOSHIBA_LED_OFF;
                _green_des = TOSHIBA_LED_OFF;
                break;

            case 1:
            case 4:
            case 7:
                // blue on
                _red_des = TOSHIBA_LED_OFF;
                _blue_des = TOSHIBA_LED_DIM;
                _green_des = TOSHIBA_LED_OFF;
                break;

            case 2:
            case 5:
            case 8:
                // green on
                _red_des = TOSHIBA_LED_OFF;
                _blue_des = TOSHIBA_LED_OFF;
                _green_des = TOSHIBA_LED_DIM;
                break;

            case 9:
                // all off
                _red_des = TOSHIBA_LED_OFF;
                _blue_des = TOSHIBA_LED_OFF;
                _green_des = TOSHIBA_LED_OFF;
                break;
        }
        // exit so no other status modify this pattern
        return;
    }

    // failsafe patterns for radio and battery - single flash yellow
    // failsafe pattern for gps - flashing blue and yellow
    if (AP_Notify::flags.failsafe_radio || AP_Notify::flags.failsafe_battery || AP_Notify::flags.failsafe_gps || AP_Notify::flags.gps_glitching) {
        switch(step) {
            case 0:
            case 1:
            case 2:
            case 3:
            case 4:
                // yellow on
                _red_des = TOSHIBA_LED_DIM;
                _blue_des = TOSHIBA_LED_OFF;
                _green_des = TOSHIBA_LED_DIM;
                break;
            case 5:
            case 6:
            case 7:
            case 8:
            case 9:
                // all off of radio or battery, blue on for gps
                _red_des = TOSHIBA_LED_OFF;
                if (AP_Notify::flags.failsafe_gps || AP_Notify::flags.gps_glitching) {
                    _blue_des = TOSHIBA_LED_DIM;
                }else{
                    _blue_des = TOSHIBA_LED_OFF;
                }
                _green_des = TOSHIBA_LED_OFF;
                break;
        }
        // exit so no other status modify this pattern
        return;
    }

    // solid green or flashing green if armed
    if (AP_Notify::flags.armed) {
        // solid green if armed with GPS 3d lock
        if (AP_Notify::flags.gps_status == 3) {
            _red_des = TOSHIBA_LED_OFF;
            _blue_des = TOSHIBA_LED_OFF;
            _green_des = TOSHIBA_LED_DIM;
        }else{
            // solid blue if armed with no GPS lock
            _red_des = TOSHIBA_LED_OFF;
            _blue_des = TOSHIBA_LED_DIM;
            _green_des = TOSHIBA_LED_OFF;
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
                    _red_des = TOSHIBA_LED_DIM;
                    _blue_des = TOSHIBA_LED_OFF;
                    _green_des = TOSHIBA_LED_DIM;
                    break;
                case 2:
                case 3:
                case 6:
                case 7:
                case 8:
                case 9:
                    // all off
                    _red_des = TOSHIBA_LED_OFF;
                    _blue_des = TOSHIBA_LED_OFF;
                    _green_des = TOSHIBA_LED_OFF;
                    break;
            }
        }else{
            // flashing green if disarmed with GPS 3d lock
            // flashing blue if disarmed with no gps lock
            switch(step) {
                case 0:
                case 1:
                case 2:
                case 3:
                case 4:
                    _red_des = TOSHIBA_LED_OFF;
                    if (AP_Notify::flags.gps_status == 3) {
                        // flashing green if disarmed with GPS 3d lock
                        _blue_des = TOSHIBA_LED_OFF;
                        _green_des = TOSHIBA_LED_DIM;
                    }else{
                        // flashing blue if disarmed with no gps lock
                        _blue_des = TOSHIBA_LED_DIM;
                        _green_des = TOSHIBA_LED_OFF;
                    }
                    break;
                case 5:
                case 6:
                case 7:
                case 8:
                case 9:
                    // all off
                    _red_des = TOSHIBA_LED_OFF;
                    _blue_des = TOSHIBA_LED_OFF;
                    _green_des = TOSHIBA_LED_OFF;
                    break;
            }
        }
    }
}

// update - updates led according to timed_updated.  Should be called
// at 50Hz
void ToshibaLED::update()
{
    // return immediately if not enabled
    if (!_healthy) {
        return;
    }
    update_colours();
    set_rgb(_red_des, _green_des, _blue_des);
}
