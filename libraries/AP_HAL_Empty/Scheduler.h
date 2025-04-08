#pragma once

#include "AP_HAL_Empty.h"

class Empty::Scheduler : public AP_HAL::Scheduler {
public:
    Scheduler() {}
    void     init() override {}
    void     delay(uint16_t ms) override {}
    void     delay_microseconds(uint16_t us) override {}
    void     register_timer_process(AP_HAL::MemberProc) override {}
    void     register_io_process(AP_HAL::MemberProc) override {}

    void     register_timer_failsafe(AP_HAL::Proc, uint32_t period_us) override {}

    void     set_system_initialized() override {}
    bool     is_system_initialized() override { return true; }

    void     reboot(bool hold_in_bootloader) override { for (;;); }

};
