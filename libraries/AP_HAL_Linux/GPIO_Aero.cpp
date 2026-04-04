/*
 * Copyright (C) 2016  Intel Corporation. All rights reserved.
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <AP_HAL/AP_HAL.h>

#if HAL_LINUX_GPIO_AERO_ENABLED

#include "GPIO_Aero.h"

const unsigned Linux::GPIO_Sysfs::pin_table[] = {
    [AERO_GPIO_BMI160_INT1] = 411,
};

const uint8_t Linux::GPIO_Sysfs::n_pins = _AERO_GPIO_MAX;

static_assert(ARRAY_SIZE(Linux::GPIO_Sysfs::pin_table) == _AERO_GPIO_MAX,
              "GPIO pin_table must have the same size of entries in enum gpio_aero");

#endif  // HAL_LINUX_GPIO_AERO_ENABLED
