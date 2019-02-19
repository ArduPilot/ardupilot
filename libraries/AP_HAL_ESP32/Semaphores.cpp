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

#include "Semaphores.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

using namespace ESP32;

Semaphore::Semaphore()
{
    handle = xSemaphoreCreateMutex();
}

bool Semaphore::give()
{
    return xSemaphoreGive(handle);
}

bool Semaphore::take(uint32_t timeout_ms)
{
    return xSemaphoreTake(handle, timeout_ms/portTICK_PERIOD_MS);
}

void Semaphore::take_blocking()
{
    xSemaphoreTake(handle, portMAX_DELAY);
}

bool Semaphore::take_nonblocking()
{
    return xSemaphoreTake(handle, 0);
}

Semaphore_Recursive::Semaphore_Recursive()
{
    handle = xSemaphoreCreateRecursiveMutex();
}

bool Semaphore_Recursive::give()
{
    return xSemaphoreGiveRecursive(handle);
}

bool Semaphore_Recursive::take(uint32_t timeout_ms)
{
    return xSemaphoreTakeRecursive(handle, timeout_ms/portTICK_PERIOD_MS);
}

void Semaphore_Recursive::take_blocking()
{
    xSemaphoreTakeRecursive(handle, portMAX_DELAY);
}

bool Semaphore_Recursive::take_nonblocking()
{
    return xSemaphoreTakeRecursive(handle, 0);
}
