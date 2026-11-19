// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "AP_Module_config.h"

#if AP_MODULE_ENABLED
#include <AP_HAL/Semaphores.h>
#include <AP_MultiHeap/AP_MultiHeap.h>

namespace AP_Module
{

// One module owns each heap. The owner must join its thread before destruction.
class Heap
{
public:
    Heap() = default;
    CLASS_NO_COPY(Heap);
    ~Heap();

    bool create(uint32_t size, bool allow_expansion = true, uint32_t max_capacity = 0);
    void *allocate(uint32_t size, uint32_t alignment);
    bool deallocate(void *pointer);
    void clear();

    struct Stats {
        uint32_t used;
        uint32_t peak;
        uint32_t allocations;
        uint32_t expanded_to;
        uint8_t expansion_failures;
        uint32_t capacity; // Current backing arena budget, including expansion.
        uint32_t free;     // Allocator free bytes, populated only by stats(true).
    };
    // Free-space inspection may walk fragmented arenas; only diagnostic
    // callers request it, not the manager's regular allocation-counter polls.
    Stats stats(bool include_free = false);

private:
    struct Allocation {
        Allocation *next;
        void *pointer;
        uint32_t charge;
    };

    MultiHeap _heap{};
    HAL_Semaphore _sem;
    Allocation *_allocations{};
    Stats _stats{};
};

}
#endif // AP_MODULE_ENABLED
