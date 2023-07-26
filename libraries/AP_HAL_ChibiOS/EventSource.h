#pragma once
#include <AP_HAL/AP_HAL.h>

#include <stdint.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/AP_HAL_Macros.h>
#include <AP_HAL/EventHandle.h>
#include "AP_HAL_ChibiOS_Namespace.h"
#include <ch.hpp>

#if CH_CFG_USE_EVENTS == TRUE
class ChibiOS::EventSource : public AP_HAL::EventSource {
    // Single event source to be shared across multiple users
    chibios_rt::EventSource ch_evt_src_;

public:
    // generate event from thread context
    void signal(uint32_t evt_mask) override;

    // generate event from interrupt context
    void signalI(uint32_t evt_mask) override;

    // Wait on an Event handle, method for internal use by EventHandle
    bool wait(uint16_t duration_us, AP_HAL::EventHandle* evt_handle) override;
};
#endif //#if CH_CFG_USE_EVENTS == TRUE
