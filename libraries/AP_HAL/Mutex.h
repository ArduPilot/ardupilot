/*
 * Copyright (C) 2017  Intel Corporation. All rights reserved.
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
#pragma once

#include <inttypes.h>

#include <AP_HAL/AP_HAL_Macros.h>

namespace AP_HAL {

template <typename T>
class Mutex {
public:
    virtual void lock() = 0;
    virtual bool try_lock() WARN_IF_UNUSED = 0;
    virtual bool try_lock(uint32_t timeout_msec) WARN_IF_UNUSED = 0;
    virtual void unlock();
    virtual ~Mutex() { }

    static Mutex *create();
};

}
