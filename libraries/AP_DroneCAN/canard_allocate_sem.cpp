#include <AP_HAL/AP_HAL.h>
#include <canard.h>

void canard_allocate_sem_take(CanardPoolAllocator *allocator) {
    if (allocator->semaphore == nullptr) {
        allocator->semaphore = NEW_NOTHROW HAL_Semaphore;
        if (allocator->semaphore == nullptr) {
            // out of memory
            CANARD_ASSERT(0);
            return;
        }
    }
    ((HAL_Semaphore*)allocator->semaphore)->take_blocking();
}

void canard_allocate_sem_give(CanardPoolAllocator *allocator) {
    if (allocator->semaphore == nullptr) {
        // it should have been allocated by canard_allocate_sem_take
        CANARD_ASSERT(0);
        return;
    }
    ((HAL_Semaphore*)allocator->semaphore)->give();
}

