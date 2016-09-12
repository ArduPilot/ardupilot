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

#include "AP_HAL_Linux.h"

namespace Linux {

class CameraSensor {
public:
    CameraSensor(const char *device_path) { _device_path = device_path; }

    bool set_format(uint32_t width, uint32_t height, uint32_t format);
    bool set_crop(uint32_t left, uint32_t top);

private:
    const char *_device_path;
};

}
