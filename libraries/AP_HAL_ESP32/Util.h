#pragma once

#include <AP_HAL/AP_HAL.h>
#include "HAL_ESP32_Namespace.h"

//see components/heap/include/esp_heas_cap.h

class ESP32::Util : public AP_HAL::Util {
public:
    bool run_debug_shell(AP_HAL::BetterStream *stream) override { return false; }

    /**
       how much free memory do we have in bytes.  on esp32 this returns the
       largest free block of memory able to be allocated with the given capabilities.
       , which in this case is "Memory must be able to run executable code"
     */
     virtual uint32_t available_memory(void) {
    	 return heap_caps_get_largest_free_block(MALLOC_CAP_EXEC);
     }
};
