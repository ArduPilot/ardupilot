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

#include <pthread.h>

#include <AP_HAL/AP_HAL_Macros.h>
#include <AP_HAL/Mutex.h>

namespace AP {

class Mutex : public AP_HAL::Mutex {
public:
    void lock() override;
    bool try_lock() override WARN_IF_UNUSED;
    bool try_lock(uint32_t timeout_msec) override WARN_IF_UNUSED;
    void unlock() override;

private:
    pthread_mutex_t _lock = PTHREAD_MUTEX_INITIALIZER;
};

}
