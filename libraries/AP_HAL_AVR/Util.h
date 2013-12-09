
#ifndef __AP_HAL_AVR_UTIL_H__
#define __AP_HAL_AVR_UTIL_H__

#include <AP_HAL.h>
#include "AP_HAL_AVR_Namespace.h"

class AP_HAL_AVR::AVRUtil : public AP_HAL::Util {
public:
    bool run_debug_shell(AP_HAL::BetterStream *stream) { return false; }
};

#endif // __AP_HAL_AVR_UTIL_H__
