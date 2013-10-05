
#ifndef __AP_HAL_PX4_UTIL_H__
#define __AP_HAL_PX4_UTIL_H__

#include <AP_HAL.h>
#include "AP_HAL_PX4_Namespace.h"

class PX4::PX4Util : public AP_HAL::Util {
public:
    PX4Util(void);
    bool run_debug_shell(AP_HAL::BetterStream *stream);

    enum safety_state safety_switch_state(void);

private:
    int _safety_handle;
};

#endif // __AP_HAL_PX4_UTIL_H__
