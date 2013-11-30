/*
  Buzzer driver
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

#include "Buzzer.h"
#include "AP_Notify.h"

extern const AP_HAL::HAL& hal;

void Buzzer::init()
{
    // return immediately if disabled
    if (!AP_Notify::flags.external_leds) {
        return;
    }

    // setup the pin and ensure it's off
    hal.gpio->pinMode(BUZZER_PIN, GPIO_OUTPUT);
    on(false);

    // set initial boot states. This prevents us issueing a arming
    // warning in plane and rover on every boot
    _flags.armed = AP_Notify::flags.armed;
    _flags.failsafe_battery = AP_Notify::flags.failsafe_battery;
}

// update - updates led according to timed_updated.  Should be called at 50Hz
void Buzzer::update()
{
    // return immediately if disabled
    if (!AP_Notify::flags.external_leds) {
        return;
    }

    // reduce 50hz call down to 10hz
    _counter++;
    if (_counter < 5) {
        return;
    }
    _counter = 0;

    // complete currently played pattern
    if (_pattern != NONE) {
        _pattern_counter++;
        switch (_pattern) {
            case SINGLE_BUZZ:
                // buzz for 10th of a second
                if (_pattern_counter == 1) {
                    on(true);
                }else{
                    on(false);
                    _pattern = NONE;
                }
                return;
            case DOUBLE_BUZZ:
                // buzz for 10th of a second
                switch (_pattern_counter) {
                    case 1:
                        on(true);
                        break;
                    case 2:
                        on(false);
                        break;
                    case 3:
                        on(true);
                        break;
                    case 4:
                    default:
                        on(false);
                        _pattern = NONE;
                        break;
                }
                return;
            case GPS_GLITCH:
                // play bethoven's 5th type buzz (three fast, one long)
                switch (_pattern_counter) {
                    case 1:
                    case 3:
                    case 5:
                    case 7:
                        on(true);
                        break;
                    case 2:
                    case 4:
                    case 6:
                        on(false);
                        break;
                    case 17:
                        on(false);
                        _pattern = NONE;
                        break;
                    default:
                        // do nothing
                        break;
                }
                return;
            default:
                // do nothing
                break;
        }
    }

    // check if armed status has changed
    if (_flags.armed != AP_Notify::flags.armed) {
        _flags.armed = AP_Notify::flags.armed;
        if (_flags.armed) {
            // double buzz when armed
            play_pattern(DOUBLE_BUZZ);
        }else{
            // single buzz when disarmed
            play_pattern(SINGLE_BUZZ);
        }
        return;
    }

    // check gps glitch
    if (_flags.gps_glitching != AP_Notify::flags.gps_glitching) {
        _flags.gps_glitching = AP_Notify::flags.gps_glitching;
        if (_flags.gps_glitching) {
            // gps glitch warning buzz
            play_pattern(GPS_GLITCH);
        }
        return;
    }

    // check gps failsafe
    if (_flags.failsafe_gps != AP_Notify::flags.failsafe_gps) {
        _flags.failsafe_gps = AP_Notify::flags.failsafe_gps;
        if (_flags.failsafe_gps) {
            // gps glitch warning buzz
            play_pattern(GPS_GLITCH);
        }
        return;
    }

    // if battery failsafe constantly single buzz
    if (AP_Notify::flags.failsafe_battery) {
        play_pattern(SINGLE_BUZZ);
    }
}

// on - turns the buzzer on or off
void Buzzer::on(bool turn_on)
{
    // return immediately if nothing to do
    if (_flags.on == turn_on) {
        return;
    }

    // update state
    _flags.on = turn_on;

    // pull pin high or low
    hal.gpio->write(BUZZER_PIN, _flags.on);
}

/// play_pattern - plays the defined buzzer pattern
void Buzzer::play_pattern(BuzzerPattern pattern_id)
{
    _pattern = pattern_id;
    _pattern_counter = 0;
}