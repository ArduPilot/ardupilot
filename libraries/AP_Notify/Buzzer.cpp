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
#include "Buzzer.h"

#include <AP_HAL/AP_HAL.h>

#include "AP_Notify.h"

extern const AP_HAL::HAL& hal;

bool Buzzer::init()
{
    if (pNotify->buzzer_enabled() == false) {
        return false;
    }
    _pin = pNotify->get_buzz_pin();
    if (_pin <= 0) {
        // no buzzer
        return false;
    }

    // setup the pin and ensure it's off
    hal.gpio->pinMode(_pin, HAL_GPIO_OUTPUT);
    on(false);

    // set initial boot states. This prevents us issuing a arming
    // warning in plane and rover on every boot
    _flags.armed = AP_Notify::flags.armed;
    _flags.failsafe_battery = AP_Notify::flags.failsafe_battery;
    return true;
}

// update - updates led according to timed_updated.  Should be called at 50Hz
void Buzzer::update()
{
    update_pattern_to_play();
    update_playing_pattern();
}

void Buzzer::update_pattern_to_play()
{
    // check for arming failed event
    if (AP_Notify::events.arming_failed) {
        // arming failed buzz
        play_pattern(SINGLE_BUZZ);
        return;
    }

    if (AP_HAL::millis() - _pattern_start_time < _pattern_start_interval_time_ms) {
        // do not interrupt playing patterns / enforce minumum separation
        return;
    }

    // initializing?
    if (_flags.gyro_calibrated != AP_Notify::flags.gyro_calibrated) {
        _flags.gyro_calibrated = AP_Notify::flags.gyro_calibrated;
        play_pattern(INIT_GYRO);
    }

    // check if prearm check are good
    if (AP_Notify::flags.pre_arm_check  && !_flags.pre_arm_check) {
        _flags.pre_arm_check = true;
        play_pattern(PRE_ARM_GOOD);
    }

    // check if armed status has changed
    if (_flags.armed != AP_Notify::flags.armed) {
        _flags.armed = AP_Notify::flags.armed;
        if (_flags.armed) {
            // double buzz when armed
            play_pattern(ARMING_BUZZ);
        } else {
            // single buzz when disarmed
            play_pattern(SINGLE_BUZZ);
        }
        return;
    }

    // check ekf bad
    if (_flags.ekf_bad != AP_Notify::flags.ekf_bad) {
        _flags.ekf_bad = AP_Notify::flags.ekf_bad;
        if (_flags.ekf_bad) {
            // ekf bad warning buzz
            play_pattern(EKF_BAD);
        }
        return;
    }

    // if vehicle lost was enabled, starting beep
    if (AP_Notify::flags.vehicle_lost) {
        play_pattern(DOUBLE_BUZZ);
        return;
    }

    // if battery failsafe constantly single buzz
    if (AP_Notify::flags.failsafe_battery) {
        play_pattern(SINGLE_BUZZ);
        return;
    }
}


void Buzzer::update_playing_pattern()
{
    if (_pattern == 0UL) {
        return;
    }

    const uint32_t delta = AP_HAL::millis() - _pattern_start_time;
    if (delta >= 3200) {
        // finished playing pattern
        on(false);
        _pattern = 0UL;
        return;
    }
    const uint32_t bit = delta / 100UL; // each bit is 100ms
    on(_pattern & (1U<<(31-bit)));
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
    const uint8_t buzz_on = pNotify->get_buzz_level();
    hal.gpio->write(_pin, _flags.on? buzz_on : !buzz_on);
}

/// play_pattern - plays the defined buzzer pattern
void Buzzer::play_pattern(const uint32_t pattern)
{
    _pattern = pattern;
    _pattern_start_time = AP_HAL::millis();
}

