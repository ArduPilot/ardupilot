
#include "Semaphores.h"

using namespace SMACCM;

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

  if (timeout_ms == HAL_SEMAPHORE_BLOCK_FOREVER)
    delay = portMAX_DELAY;
  else
    delay = timeout_ms / portTICK_RATE_MS;

  return xSemaphoreTake(m_semaphore, delay);
}

bool SMACCMSemaphore::take_nonblocking()
{
  return xSemaphoreTake(m_semaphore, 0);
}

bool SMACCMSemaphore::give()
{
  return xSemaphoreGive(m_semaphore);
}
