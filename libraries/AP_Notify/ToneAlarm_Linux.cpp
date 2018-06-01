/*
  ToneAlarm Linux driver
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

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include "ToneAlarm_Linux.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <AP_HAL_Linux/Util.h>

#include "AP_Notify.h"

extern const AP_HAL::HAL& hal;

bool ToneAlarm_Linux::init()
{
    // open the tone alarm device
    _initialized = hal.util->toneAlarm_init();
    if (!_initialized) {
        hal.console->printf("AP_Notify: Failed to initialise ToneAlarm");
        return false;
    }

    // set initial boot states. This prevents us issuing a arming
    // warning in plane and rover on every boot
    flags.armed = AP_Notify::flags.armed;
    flags.failsafe_battery = AP_Notify::flags.failsafe_battery;
    return true;
}

// play_tune - play one of the pre-defined tunes
bool ToneAlarm_Linux::play_tune(uint8_t tune_number)
{
    hal.util->toneAlarm_set_tune(tune_number);
    return true;
}


// update - updates led according to timed_updated.  Should be called at 50Hz
void ToneAlarm_Linux::update()
{
    // exit immediately if we haven't initialised successfully
    if (!_initialized) {
        return;
    }

    // exit if buzzer is not enabled
    if (pNotify->buzzer_enabled() == false) {
        return;
    }

    // check for arming failure
    if (AP_Notify::events.arming_failed) {
        play_tune(TONE_ARMING_FAILURE_TUNE);
    }

    // check if arming status has changed
    if (flags.armed != AP_Notify::flags.armed) {
        flags.armed = AP_Notify::flags.armed;
        if (flags.armed) {
            // arming tune
            play_tune(TONE_ARMING_WARNING_TUNE);
        }else{
            // disarming tune
            play_tune(TONE_NOTIFY_NEUTRAL_TUNE);
        }
    }

    // check if battery status has changed
    if (flags.failsafe_battery != AP_Notify::flags.failsafe_battery) {
        flags.failsafe_battery = AP_Notify::flags.failsafe_battery;
        if (flags.failsafe_battery) {
            // low battery warning tune
            play_tune(TONE_BATTERY_WARNING_FAST_TUNE);
        }
    }

    // check parachute release
    if (flags.parachute_release != AP_Notify::flags.parachute_release) {
        flags.parachute_release = AP_Notify::flags.parachute_release;
        if (flags.parachute_release) {
            // parachute release warning tune
            play_tune(TONE_PARACHUTE_RELEASE_TUNE);
        }
    }
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_PX4
