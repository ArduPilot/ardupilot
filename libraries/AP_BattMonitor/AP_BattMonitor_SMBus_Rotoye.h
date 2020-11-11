#pragma once

#include "AP_BattMonitor_SMBus_Generic.h"

class AP_BattMonitor_SMBus_Rotoye : public AP_BattMonitor_SMBus_Generic
{
    using AP_BattMonitor_SMBus_Generic::AP_BattMonitor_SMBus_Generic;

private:

    // Rotoye Batmon has two temperature readings
    bool read_temp(void) override;

};