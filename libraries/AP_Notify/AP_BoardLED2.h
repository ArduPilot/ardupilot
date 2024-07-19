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
#pragma once

#include "AP_Notify_config.h"

#if AP_NOTIFY_GPIO_LED_2_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL_Boards.h>

#include "NotifyDevice.h"

class AP_BoardLED2: public NotifyDevice
{
public:
    // initialise the LED driver
    bool init(void) override;

    // should be called at 50Hz
    void update(void) override;

private:
    // counter incremented at 50Hz
    uint8_t _counter;
    uint16_t _sat_cnt;
    uint8_t save_trim_counter;
    uint8_t arm_counter = 0;
};

#endif  // AP_NOTIFY_GPIO_LED_2_ENABLED
