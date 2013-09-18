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
#ifndef __TONE_ALARM_PX4_H__
#define __TONE_ALARM_PX4_H__

#include "ToneAlarm_PX4.h"

class ToneAlarm_PX4
{
public:
    /// init - initialised the tone alarm
    bool init(void);

    /// update - updates led according to timed_updated.  Should be called at 50Hz
    void update();

private:
    /// play_tune - play one of the pre-defined tunes
    bool play_tune(const uint8_t tune_number);

    int _tonealarm_fd;      // file descriptor for the tone alarm

    /// tonealarm_type - bitmask of states we track
    struct tonealarm_type {
        uint8_t armed              : 1;    // 0 = disarmed, 1 = armed
        uint8_t failsafe_battery   : 1;    // 1 if battery failsafe
    } flags;
};

#endif // __TONE_ALARM_PX4_H__
