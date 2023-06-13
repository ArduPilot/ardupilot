#include "AP_HAL.h"
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL &hal;

#if AP_DEADLOCK_DETECTOR_ENABLED
uint32_t AP_HAL::Semaphore::total_sem;
#include <stdio.h>
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
    bool in_main = hal.scheduler->in_main_thread();
    if (in_main) {
        hal.util->persistent_data.semaphore_line = line;
    }
    _mtx.take_blocking();
    if (in_main) {
        hal.util->persistent_data.semaphore_line = 0;
    }
}

WithSemaphore::~WithSemaphore()
{
    _mtx.give();
}

#if AP_DEADLOCK_DETECTOR_ENABLED
uint32_t sem_error_count;

void AP_HAL::Semaphore::push_list(void)
{
    depth++;
    if (depth != 1) {
        return;
    }
    next = get_sem_list();
    set_sem_list(this);
}

void AP_HAL::Semaphore::pop_list(void)
{
    depth--;
    if (depth != 0) {
        return;
    }
    if (next) {
        ofs_min = MAX(ofs_min, next->ofs_min+1);
        next->ofs_max = MAX(next->ofs_max, ofs_max+1);
    }
    if (-int32_t(total_sem)+ofs_min > int32_t(total_sem)-ofs_max) {
        sem_error_count++;
        //::printf("sem_error_count=%u\n", unsigned(sem_error_count));
    }
    set_sem_list(next);
    next = nullptr;
}
#endif // AP_DEADLOCK_DETECTOR_ENABLED
