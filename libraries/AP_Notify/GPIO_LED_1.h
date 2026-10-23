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

#if AP_NOTIFY_GPIO_LED_1_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL_Boards.h>

#include "NotifyDevice.h"

class GPIO_LED_1 : public NotifyDevice
{
public:
    // initialise the LED driver
    bool init(void) override;

    // should be called at 50Hz
    void update(void) override;

private:

    // left-to-right, each bit represents 100ms
    static const uint32_t INITIALIZING = 0b10101010101010101010101010101010UL;
    static const uint32_t NOT_READY_TO_ARM = 0b11110000000000001111111110000000UL;
    static const uint32_t READY_TO_ARM = 0b11111111111111100000000000000000UL;
    static const uint32_t ARMED = 0b11111111111111111111111111111111UL;
    static const uint32_t FAILSAFE = 0b11110000111100001111000011110000UL;

    uint32_t current_pattern = INITIALIZING;
    uint32_t last_timestep_ms;
    uint8_t next_bit;
};

#endif  // AP_NOTIFY_GPIO_LED_1_ENABLED
