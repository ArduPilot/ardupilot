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
};
