#include "AP_HAL.h"

extern const AP_HAL::HAL &hal;

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

uint32_t sem_error_count;

void AP_HAL::Semaphore::push_list(void)
{
    depth++;
    if (depth != 1) {
        return;
    }
    next = get_sem_list();
    if (next) {
        for (uint8_t i=0; i<HAL_SEMAPHORE_MAX_CALLERS; i++) {
            if (parent[i] == next) {
                break;
            }
            if (parent[i] == nullptr) {
                parent[i] = next;
                for (uint8_t j=0; j<HAL_SEMAPHORE_MAX_CALLERS; j++) {
                    if (child[j] == nullptr) {
                        break;
                    }
                    if (next == child[j]) {
                        sem_error_count++;
                        break;
                    }
                }
                break;
            }
        }
        auto *child2 = next->child;
        auto *parent2 = next->parent;
        for (uint8_t i=0; i<HAL_SEMAPHORE_MAX_CALLERS; i++) {
            if (child2[i] == this) {
                break;
            }
            if (child2[i] == nullptr) {
                child2[i] = this;
                for (uint8_t j=0; j<HAL_SEMAPHORE_MAX_CALLERS; j++) {
                    if (parent2[j] == nullptr) {
                        break;
                    }
                    if (this == parent2[j]) {
                        sem_error_count++;
                        break;
                    }
                }
                break;
            }
        }
    }
    set_sem_list(this);
}

void AP_HAL::Semaphore::pop_list(void)
{
    depth--;
    if (depth != 0) {
        return;
    }
    set_sem_list(next);
    next = nullptr;
}
