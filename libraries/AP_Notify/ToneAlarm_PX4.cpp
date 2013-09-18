/*
  ToneAlarm PX4 driver
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

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include "ToneAlarm_PX4.h"
#include "AP_Notify.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <drivers/drv_tone_alarm.h>
#include <stdio.h>
#include <errno.h>

extern const AP_HAL::HAL& hal;

bool ToneAlarm_PX4::init()
{
    // open the tone alarm device
    _tonealarm_fd = open(TONEALARM_DEVICE_PATH, 0);
    if (_tonealarm_fd == -1) {
        hal.console->printf("Unable to open " TONEALARM_DEVICE_PATH);
        return false;
    }
    
    // set initial boot states. This prevents us issueing a arming
    // warning in plane and rover on every boot
    flags.armed = AP_Notify::flags.armed;
    flags.failsafe_battery = AP_Notify::flags.failsafe_battery;
    return true;
}

// play_tune - play one of the pre-defined tunes
bool ToneAlarm_PX4::play_tune(const uint8_t tune_number)
{
    int ret = ioctl(_tonealarm_fd, TONE_SET_ALARM, tune_number);
    return (ret == 0);
}


// update - updates led according to timed_updated.  Should be called at 50Hz
void ToneAlarm_PX4::update()
{
    // exit immediately if we haven't initialised successfully
    if (_tonealarm_fd == -1) {
        return;
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
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_PX4
