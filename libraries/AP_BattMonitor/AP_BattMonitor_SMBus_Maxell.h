#pragma once

#include "AP_BattMonitor_SMBus_Generic.h"

#if AP_BATTERY_SMBUS_MAXELL_ENABLED

class AP_BattMonitor_SMBus_Maxell : public AP_BattMonitor_SMBus_Generic
{
    using AP_BattMonitor_SMBus_Generic::AP_BattMonitor_SMBus_Generic;

private:

    // return a scaler that should be multiplied by the battery's reported capacity numbers to arrive at the actual capacity in mAh
    uint16_t get_capacity_scaler() const override { return 2; }

};

#endif  // AP_BATTERY_SMBUS_MAXELL_ENABLED
