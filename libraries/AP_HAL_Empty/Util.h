
#ifndef __AP_HAL_EMPTY_UTIL_H__
#define __AP_HAL_EMPTY_UTIL_H__

#include <AP_HAL.h>
#include "AP_HAL_Empty_Namespace.h"

class Empty::EmptyUtil : public AP_HAL::Util {
public:
    bool run_debug_shell(AP_HAL::BetterStream *stream) { return false; }
};

#endif // __AP_HAL_EMPTY_UTIL_H__
