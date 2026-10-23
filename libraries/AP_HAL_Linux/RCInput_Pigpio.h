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
/*
   RCInput driver using pigpio library for edge-detection based RC input.
   This uses pigpio's alert callback to get precise edge timestamps,
   which are fed to AP_RCProtocol's soft-serial decoder.
 */
#pragma once

#include "AP_HAL_Linux.h"

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RPI

#include "RCInput.h"

namespace Linux {

class RCInput_Pigpio : public RCInput
{
public:
    RCInput_Pigpio(uint32_t gpio_pin, uint32_t baudrate, bool inverted);
    ~RCInput_Pigpio();

    void early_init() override;
    void init() override;
    void _timer_tick() override;
    void teardown() override;

private:
    uint32_t _gpio_pin;
    uint32_t _baudrate;
    bool _inverted;
    bool _initialized;
    bool _pigpio_initialized;
    uint32_t _last_high_us;
};

}

#endif // CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RPI
