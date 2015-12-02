
#ifndef __AP_HAL_SITL_UTIL_H__
#define __AP_HAL_SITL_UTIL_H__

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_SITL_Namespace.h"

class HALSITL::SITLUtil : public AP_HAL::Util {
public:
    bool run_debug_shell(AP_HAL::BetterStream *stream) {
        return false;
    }

    /**
       how much free memory do we have in bytes. 
     */
    uint32_t available_memory(void) override {
        // SITL is assumed to always have plenty of memory. Return 128k for now
        return 0x20000;
    }
};

#endif // __AP_HAL_SITL_UTIL_H__
