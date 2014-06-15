
#ifndef __AP_HAL_MPNG_UTIL_H__
#define __AP_HAL_MPNG_UTIL_H__

#include <AP_HAL.h>
#include "AP_HAL_MPNG_Namespace.h"
#include "memcheck.h"

class MPNG::AVRUtil : public AP_HAL::Util {
public:
    bool run_debug_shell(AP_HAL::BetterStream *stream) { return false; }
	uint16_t available_memory(void) { return memcheck_available_memory(); }
};

#endif // __AP_HAL_MPNG_UTIL_H__
