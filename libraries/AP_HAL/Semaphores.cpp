#include "AP_HAL.h"

#ifndef HAL_BOOTLOADER_BUILD
extern const AP_HAL::HAL &hal;
#endif
/*
  implement WithSemaphore class for WITH_SEMAPHORE() support
 */
WithSemaphore::WithSemaphore(AP_HAL::Semaphore *mtx, uint32_t line) :
    WithSemaphore(*mtx, line)
{}

WithSemaphore::WithSemaphore(AP_HAL::Semaphore &mtx, uint32_t line) :
    _mtx(mtx)
{
#ifndef HAL_BOOTLOADER_BUILD
    bool in_main = hal.scheduler->in_main_thread();
    if (in_main) {
        hal.util->persistent_data.semaphore_line = line;
    }
#endif
    _mtx.take_blocking();
#ifndef HAL_BOOTLOADER_BUILD
    if (in_main) {
        hal.util->persistent_data.semaphore_line = 0;
    }
#endif
}

WithSemaphore::~WithSemaphore()
{
    _mtx.give();
}
