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
