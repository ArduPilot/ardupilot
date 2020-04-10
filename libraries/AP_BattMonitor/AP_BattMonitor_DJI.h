#pragma once

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"

class AP_BattMonitor_DJI : public AP_BattMonitor_Backend
{
public:

    /// Constructor
    AP_BattMonitor_DJI(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params);

    /// Read the battery voltage and percentage.
    void read() override;

    /// returns true if battery monitor provides consumed energy info
    bool has_consumed_energy() const override { return true; }

    /// returns true if battery monitor provides current info
    bool has_current() const override { return true; }

    void init(void) override {}

private:
    AP_HAL::UARTDriver *port;
    
    uint32_t last_send_us;
    uint32_t count;
    uint8_t pktbuf[64];
};