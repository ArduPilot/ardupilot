#pragma once

#include "AP_BattMonitor_SMBus.h"

#if AP_BATTERY_SMBUS_NEODESIGN_ENABLED

class AP_BattMonitor_SMBus_NeoDesign : public AP_BattMonitor_SMBus
{
public:
    AP_BattMonitor_SMBus_NeoDesign(AP_BattMonitor &mon,
                             AP_BattMonitor::BattMonitor_State &mon_state,
                             AP_BattMonitor_Params &params);

private:

    void timer(void) override;

    uint8_t _cell_count;

    static const constexpr uint8_t max_cell_count = 10;
};

#endif  // AP_BATTERY_SMBUS_NEODESIGN_ENABLED
