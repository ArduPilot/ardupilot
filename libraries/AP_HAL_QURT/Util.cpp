#include <AP_HAL/AP_HAL.h>
#include "Semaphores.h"
#include <stdio.h>

extern const AP_HAL::HAL& hal;

#include "Util.h"
#include "RCOutput.h"

using namespace QURT;

extern "C" {
    void *fc_heap_alloc(size_t size);
    void fc_heap_free(void* ptr);
    size_t fc_heap_size(void);
    size_t fc_heap_usage(void);
}

uint32_t Util::available_memory(void)
{
    return fc_heap_size() - fc_heap_usage();
}

/*
  return state of safety switch, if applicable
*/
Util::safety_state Util::safety_switch_state(void)
{
    const auto *rcout = (QURT::RCOutput *)hal.rcout;
    if (rcout != nullptr && rcout->safety_on) {
        return SAFETY_DISARMED;
    }
    return SAFETY_ARMED;
}
