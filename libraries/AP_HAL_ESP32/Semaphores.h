/*
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

#include <stdint.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/AP_HAL_Macros.h>
#include <AP_HAL/Semaphores.h>
#include "HAL_ESP32_Namespace.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

class ESP32::Semaphore : public AP_HAL::Semaphore
{
public:
    Semaphore();
    bool give() override;
    bool take(uint32_t timeout_ms) override;
    bool take_nonblocking() override;
    void take_blocking() override;

    bool check_owner();
protected:
    void*  handle;
};

class ESP32::BinarySemaphore : public AP_HAL::BinarySemaphore {
public:
    BinarySemaphore(bool initial_state=false);
    ~BinarySemaphore(void);

    CLASS_NO_COPY(BinarySemaphore);

    bool wait(uint32_t timeout_us) override;
    bool wait_blocking(void) override;
    void signal(void) override;
    void signal_ISR(void) override;

protected:
    SemaphoreHandle_t _sem;
};

