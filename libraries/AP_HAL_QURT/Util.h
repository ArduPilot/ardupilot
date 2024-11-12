#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_QURT_Namespace.h"

class QURT::Util : public AP_HAL::Util
{
public:
    /*
      set HW RTC in UTC microseconds
     */
    void set_hw_rtc(uint64_t time_utc_usec) override {}

    /*
      get system clock in UTC microseconds
     */
    uint64_t get_hw_rtc() const override
    {
        return 0;
    }

    uint32_t available_memory(void) override;

    /*
      return state of safety switch, if applicable
     */
    enum safety_state safety_switch_state(void) override;
    
#if ENABLE_HEAP
    // heap functions, note that a heap once alloc'd cannot be dealloc'd
    virtual void *allocate_heap_memory(size_t size) override;
    virtual void *heap_realloc(void *h, void *ptr, size_t old_size, size_t new_size) override;

    struct heap_allocation_header {
        uint64_t allocation_size; // size of allocated block, not including this header
    };

    struct heap {
        size_t max_heap_size;
        size_t current_heap_usage;
    };
#endif // ENABLE_HEAP
};
