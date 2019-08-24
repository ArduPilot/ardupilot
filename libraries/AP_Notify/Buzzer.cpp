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

#ifndef HAL_BUZZER_ON
  #if !defined(HAL_BUZZER_PIN)
    #define HAL_BUZZER_ON (pNotify->get_buzz_level())
    #define HAL_BUZZER_OFF (!pNotify->get_buzz_level())
  #else
    #define HAL_BUZZER_ON 1
    #define HAL_BUZZER_OFF 0 
  #endif
#endif



extern const AP_HAL::HAL& hal;


bool Buzzer::init()
{
    if (pNotify->buzzer_enabled() == false) {
        return false;
    }
#if defined(HAL_BUZZER_PIN)
    _pin = HAL_BUZZER_PIN;
#else
    _pin = pNotify->get_buzz_pin();
#endif
    if(!_pin) return false;

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
    if (!_flags.initialize_done) {
        // play a quick beep at startup
        if (!_flags.startup_beep) {
            _flags.startup_beep = true;
            play_pattern(SINGLE_BUZZ);
            return;
        }
        // play beeps when system initialization complete
        if (!_flags.initialize_started) {
            if (AP_Notify::flags.initialising) {
                _flags.initialize_started = true;
            }
        } else if (!AP_Notify::flags.initialising) {
            _flags.initialize_done = true;
            play_pattern(INITIALIZE_BUZZ);
            return;
        }
    }

    // check if radio failsafe status has changed
    if (_flags.failsafe_radio != AP_Notify::flags.failsafe_radio) {
        _flags.failsafe_radio = AP_Notify::flags.failsafe_radio;
        // play beeps to indicate radio status (if initialization completed)
        if (_flags.initialize_done) {
            play_pattern(_flags.failsafe_radio ? RADIOLOST_BUZZ : RADIOBACK_BUZZ);
            return;
        }
    }

    // check for arming failed event
    if (AP_Notify::events.arming_failed) {
        // arming failed buzz
        play_pattern(SINGLE_BUZZ);
        return;
    }

    // do not interrupt playing patterns
    if (is_playing_pattern()) {
        return;
    }

    // if battery failsafe constantly buzz
    if (AP_Notify::flags.failsafe_battery) {
        play_pattern(CONSTANT_BUZZ);
        return;
    }

    // if radio failsafe active and system was (ever) armed then
    // repeat SOS beeps until radio recovered (for lost model locator)
    if (_flags.failsafe_radio && _flags.was_armed) {
        play_pattern(RADIOSOS_BUZZ);
        return;
    }

    // check if armed status has changed
    if (_flags.armed != AP_Notify::flags.armed) {
        _flags.armed = AP_Notify::flags.armed;
        if (_flags.armed) {
            // double buzz when armed
            play_pattern(ARMING_BUZZ);
            _flags.was_armed = true;
        }else{
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
            return;
        }
    }

    // if vehicle lost was enabled, starting beep
    if (AP_Notify::flags.vehicle_lost) {
        play_pattern(DOUBLE_BUZZ);
        return;
    }
}


void Buzzer::update_playing_pattern()
{
    // if no '1's to play in pattern and buzzer not on then just return
    if (_pattern == 0UL && !_flags.on) {
        return;
    }

    // reduce 50hz call down to 10hz
    if (--_update_counter > 0) {
        return;
    }
    _update_counter = 5;

    // put buzzer on if leftmost bit set, and then shift one bit to the left
    const bool on_flag = _pattern & (0b10000000000000000000000000000000UL);
    _pattern <<= 1;
    ++_pattern_pos;
    
    // can use trailing '...001' in pattern to extend timing, so
    // don't turn buzzer on if it's off when that last '1' is reached
    if (_pattern_pos != (uint8_t)32 || _pattern != 0UL || _flags.on) {
        on(on_flag);
    }
}

bool Buzzer::is_playing_pattern()
{
    return (_pattern != 0UL || _flags.on);
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
    hal.gpio->write(_pin, _flags.on? HAL_BUZZER_ON : HAL_BUZZER_OFF);
}

/// play_pattern - plays the defined buzzer pattern
void Buzzer::play_pattern(const uint32_t pattern)
{
    _pattern = pattern;
    _update_counter = 0;  // init counter for no initial delay
    _pattern_pos = 0;
}

