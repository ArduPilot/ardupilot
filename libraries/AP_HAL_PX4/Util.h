
#ifndef __AP_HAL_PX4_UTIL_H__
#define __AP_HAL_PX4_UTIL_H__

#include <AP_HAL.h>
#include "AP_HAL_PX4_Namespace.h"

class PX4::PX4Util : public AP_HAL::Util {
public:
    PX4Util(void);
    bool run_debug_shell(AP_HAL::BetterStream *stream);

    enum safety_state safety_switch_state(void);

    /*
      set system clock in UTC microseconds
     */
    void set_system_clock(uint64_t time_utc_usec);

    /*
      get system identifier (STM32 serial number)
     */
    bool get_system_id(char buf[40]);

    uint16_t available_memory(void);

private:
    int _safety_handle;
};

#endif // __AP_HAL_PX4_UTIL_H__
