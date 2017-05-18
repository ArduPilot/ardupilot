#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_QURT
#include "Semaphores.h"
#include <stdio.h>

extern const AP_HAL::HAL& hal;

#include "Util.h"

using namespace QURT;

/*
  always report 256k of free memory. I don't know how to query
  available memory on QURT
 */
uint32_t Util::available_memory(void)
{
    return 256*1024;
}

// create a new semaphore
Semaphore *Util::new_semaphore(void)
{
    return new Semaphore;
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_QURT
