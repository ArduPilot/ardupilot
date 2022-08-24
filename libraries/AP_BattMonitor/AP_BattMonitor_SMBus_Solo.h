#pragma once

#include "AP_BattMonitor_SMBus.h"

class AP_BattMonitor_SMBus_Solo : public AP_BattMonitor_SMBus
{
public:

    // Constructor
    AP_BattMonitor_SMBus_Solo(AP_BattMonitor &mon,
                             AP_BattMonitor::BattMonitor_State &mon_state,
                             AP_BattMonitor_Params &params);

private:

    void timer(void) override;

    uint8_t _button_press_count;
};
