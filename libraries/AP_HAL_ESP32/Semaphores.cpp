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

#include <AP_HAL/AP_HAL.h>

#include "Semaphores.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

extern const AP_HAL::HAL& hal;

using namespace ESP32;

Semaphore::Semaphore()
{
    handle = xSemaphoreCreateRecursiveMutex();
}

bool Semaphore::give()
{
    return xSemaphoreGiveRecursive((QueueHandle_t)handle);
}

bool Semaphore::take(uint32_t timeout_ms)
{
    if (timeout_ms == HAL_SEMAPHORE_BLOCK_FOREVER) {
        take_blocking();
        return true;
    }
    if (take_nonblocking()) {
        return true;
    }
    uint64_t start = AP_HAL::micros64();
    do {
        hal.scheduler->delay_microseconds(200);
        if (take_nonblocking()) {
            return true;
        }
    } while ((AP_HAL::micros64() - start) < timeout_ms*1000);
    return false;
}

void Semaphore::take_blocking()
{
    xSemaphoreTakeRecursive((QueueHandle_t)handle, portMAX_DELAY);
}

bool Semaphore::take_nonblocking()
{
    bool ok = xSemaphoreTakeRecursive((QueueHandle_t)handle, 0) == pdTRUE;
    if (ok) {
        give();
    }

    return ok;
}

bool Semaphore::check_owner()
{
    return xSemaphoreGetMutexHolder((QueueHandle_t)handle) == xTaskGetCurrentTaskHandle();
}
