#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_RP_Namespace.h"

class RP::Util : public AP_HAL::Util {
public:
    Util();

    // Returns free memory (Heap)
    uint32_t available_memory(void) override;

    // Create a semaphore (RP2350 uses the standard one for HAL)
    //HAL_Semaphore *new_semaphore(void) override;

    // Get unique chip ID (from flash memory via Pico SDK)
    bool get_system_id(char buf[50]) override;
    bool get_system_id_unformatted(uint8_t buf[], uint8_t &len) override;

    // The state of the safety switch (if it is not implemented in hardware).
    // Most modern RP2350 boards do not have a physical safety switch,
    // so we return SAFETY_NONE (always ready) or software logic.
    enum safety_state safety_switch_state(void) override { return SAFETY_NONE; }

    // Power control
    void set_soft_armed(bool armed) override { _soft_armed = armed; }
    bool get_soft_armed() const { return _soft_armed; }

    // Real Time Counter
    uint64_t get_hw_rtc() const override; //  get system clock in UTC microseconds
    void set_hw_rtc(uint64_t time_utc_usec) override; //  set system clock in UTC microseconds

    void reboot(bool hold_in_bootloader);

#if HAL_GCS_ENABLED
    bool get_reset_info(uint32_t &reason);
#endif

private:
    bool _soft_armed;
};
