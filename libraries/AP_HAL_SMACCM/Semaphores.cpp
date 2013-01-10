
#include "Semaphores.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_SMACCM

#include <task.h>

using namespace SMACCM;

extern const AP_HAL::HAL& hal;

/** Return true if this thread already holds "sem". */
static bool already_held(xSemaphoreHandle sem)
{
  xTaskHandle self = xTaskGetCurrentTaskHandle();
  return xSemaphoreGetMutexHolder(sem) == self;
}

SMACCMSemaphore::SMACCMSemaphore()
  : m_semaphore(NULL)
{
}

void SMACCMSemaphore::init()
{
  m_semaphore = xSemaphoreCreateMutex();
}

bool SMACCMSemaphore::take(uint32_t timeout_ms)
{
  portTickType delay;

  if (already_held(m_semaphore))
    hal.scheduler->panic("PANIC: Recursive semaphore take.");

  if (timeout_ms == HAL_SEMAPHORE_BLOCK_FOREVER)
    delay = portMAX_DELAY;
  else
    delay = timeout_ms / portTICK_RATE_MS;

  return xSemaphoreTake(m_semaphore, delay);
}

bool SMACCMSemaphore::take_nonblocking()
{
  if (already_held(m_semaphore))
    hal.scheduler->panic("PANIC: Recursive semaphore take.");

  return xSemaphoreTake(m_semaphore, 0);
}

bool SMACCMSemaphore::give()
{
  return xSemaphoreGive(m_semaphore);
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_SMACCM
