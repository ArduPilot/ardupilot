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

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_QFLIGHT

#include "RCInput.h"
#include "RCInput_DSM.h"

class Linux::RCInput_DSM : public Linux::RCInput
{
public:
    void init() override;
    void _timer_tick(void) override;
    void set_device_path(const char *path);

private:
    const char *device_path;
    int32_t fd = -1;
};
#endif // CONFIG_HAL_BOARD_SUBTYPE

