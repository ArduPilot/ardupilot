/*
 *  AP_Notify Library. 
 * based upon a prototype library by David "Buzz" Bussenschutt.
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

#ifndef __DYCO_LED_H__
#define __DYCO_LED_H__

#include <AP_HAL.h>
#include "NotifyDevice.h"
#include "../AP_HAL_LINUX/DycoLEDDriver.h"

#define MAX_NUM_LEDS    35
#define MAX_PATTERN_STEPS 16
#define STATUS_LED 0
#define STROBE_LED 1

#define NOTIFY_INITIALISING             0
#define NOTIFY_SAV_TRIM_ESC_CAL         1
#define NOTIFY_FS_RAD_BATT              2
#define NOTIFY_FS_GPS                   3
#define NOTIFY_BARO_GLITCH              4
#define NOTIFY_EKF_BAD                  5
#define NOTIFY_ARMED_GPS                6
#define NOTIFY_ARMED_NOGPS              7
#define NOTIFY_PREARM_CHECK             8
#define NOTIFY_DISARMED_GPS             9
#define NOTIFY_DISARMED_NOGPS           10
#define NOTIFY_SAFE_STROBE              11
#define NOTIFY_FAIL_STROBE              12
#define NOTIFY_NEUTRAL_STROBE           13

class DycoLED: public NotifyDevice
{
public:
    // init - initialised the LED
    bool init();

    // update - updates led according to timed_updated.  Should be
    // called at 50Hz
    void update();
protected:
    void set_preset_pattern(uint16_t led,uint8_t patt);
};

struct led_pattern
{
    uint16_t color[MAX_PATTERN_STEPS];
    uint16_t time[MAX_PATTERN_STEPS];
    float brightness[MAX_PATTERN_STEPS];
    uint8_t res;
    uint8_t len;
};
#endif // __TOSHIBA_LED_H__
