#pragma once

#include "AP_BattMonitor_SMBus_Generic.h"

#if AP_BATTERY_SMBUS_ROTOYE_ENABLED

class AP_BattMonitor_SMBus_Rotoye : public AP_BattMonitor_SMBus_Generic
{
    using AP_BattMonitor_SMBus_Generic::AP_BattMonitor_SMBus_Generic;

private:

    // Rotoye Batmon has two temperature readings
    void read_temp(void) override;

};

#endif  // AP_BATTERY_SMBUS_ROTOYE_ENABLED
