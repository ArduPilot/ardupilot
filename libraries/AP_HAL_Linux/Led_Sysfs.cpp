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

#include "Led_Sysfs.h"

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <AP_HAL/AP_HAL.h>

static const AP_HAL::HAL &hal = AP_HAL::get_HAL();

namespace Linux {

Led_Sysfs::Led_Sysfs(const char *led_name)
    : _led_name(led_name)
{
}

Led_Sysfs::~Led_Sysfs()
{
    if (_brightness_fd >= 0) {
        close(_brightness_fd);
    }
}

bool Led_Sysfs::init()
{
    char *br_path;
    char *max_br_path;

    if (asprintf(&br_path, "/sys/class/leds/%s/brightness", _led_name) == -1) {
        AP_HAL::panic("LinuxLed_Sysfs : Couldn't allocate brightness path");
    }

    _brightness_fd = open(br_path, O_WRONLY | O_CLOEXEC);
    if (_brightness_fd < 0) {
        printf("LinuxLed_Sysfs: Unable to open file %s\n", br_path);
        free(br_path);
        return false;
    }

    if (asprintf(&max_br_path, "/sys/class/leds/%s/max_brightness", _led_name) == -1) {
        AP_HAL::panic("LinuxLed_Sysfs : Couldn't allocate max_brightness path");
    }

    if (Util::from(hal.util)->read_file(max_br_path, "%u", &_max_brightness) < 0) {
        AP_HAL::panic("LinuxLed_Sysfs : Unable to read max_brightness in %s",
                      max_br_path);
    }

    free(max_br_path);
    free(br_path);

    return true;
}

bool Led_Sysfs::set_brightness(uint8_t brightness)
{
    if (_brightness_fd < 0) {
        return false;
    }

    unsigned int br = brightness * _max_brightness / UINT8_MAX;

    /* Don't log fails since this could spam the console */
    if (dprintf(_brightness_fd, "%u", br) < 0) {
        return false;
    }

    return true;
}

}
