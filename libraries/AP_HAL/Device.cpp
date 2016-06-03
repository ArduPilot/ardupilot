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
#include "AP_HAL.h"
#include "Device.h"

using namespace AP_HAL;

extern const HAL &hal;

bool Device::write_register(uint8_t reg, uint8_t val, uint16_t delay_ms)
{
    bool ret = write_register(reg, val);
    if (ret) {
        hal.scheduler->delay(delay_ms);
    }
    return ret;
}
