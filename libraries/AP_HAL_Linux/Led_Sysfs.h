/*
  Copyright (C) 2017 Mathieu Othacehe. All rights reserved.

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

#include "AP_HAL_Linux.h"
#include "Util.h"

namespace Linux {

class Led_Sysfs {
public:
    bool init();
    bool set_brightness(uint8_t brightness);

    Led_Sysfs(const char* led_name);
    ~Led_Sysfs();

private:
    int _brightness_fd = -1;
    int _max_brightness = 0;
    const char *_led_name = nullptr;
};

}
