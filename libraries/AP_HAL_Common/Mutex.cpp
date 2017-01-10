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
#include "Mutex.h"

#include <AP_HAL/AP_HAL.h>

using namespace AP;

const extern AP_HAL::HAL& hal;

void Mutex::lock()
{
    pthread_mutex_lock(&_lock);
}

bool Mutex::try_lock()
{
    return pthread_mutex_trylock(&_lock) == 0;
}

bool Mutex::try_lock(uint32_t timeout_msec)
{
    if (timeout_msec == 0) {
        lock();
        return true;
    }

    if (try_lock()) {
        return true;
    }

    uint64_t start = AP_HAL::micros64();
    do {
        hal.scheduler->delay_microseconds(200);
        if (try_lock()) {
            return true;
        }
    } while ((AP_HAL::micros64() - start) < timeout_msec * 1000);

    return false;
}

void Mutex::unlock()
{
    pthread_mutex_unlock(&_lock);
}


static AP_HAL::Mutex *AP_HAL::create()
{
    return new Mutex{};
}
