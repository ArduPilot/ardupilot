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
#ifndef __TONE_ALARM_Linux_H__
#define __TONE_ALARM_Linux_H__

#include "NotifyDevice.h"

class ToneAlarm_Linux: public NotifyDevice
{
public:
    /// init - initialised the tone alarm
    bool init(void);

    /// update - updates led according to timed_updated.  Should be called at 50Hz
    void update();

private:
    /// play_tune - play one of the pre-defined tunes
    bool play_tune(uint8_t tune_number);

    bool _initialized = false;

    /// tonealarm_type - bitmask of states we track
    struct tonealarm_type {
        bool armed              : 1;    // false = disarmed, true = armed
        bool failsafe_battery   : 1;    // true if battery failsafe
        bool parachute_release  : 1;    // true if parachute is being released
    } flags;
};

#endif // __TONE_ALARM_Linux_H__
