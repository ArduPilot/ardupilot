#pragma once

#include "AP_BattMonitor_SMBus.h"

class AP_BattMonitor_SMBus_NeoDesign : public AP_BattMonitor_SMBus
{
public:
    AP_BattMonitor_SMBus_NeoDesign(AP_BattMonitor &mon,
                             AP_BattMonitor::BattMonitor_State &mon_state,
                             AP_BattMonitor_Params &params,
                             AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

private:

    void timer(void) override;

    uint8_t _cell_count;
};
