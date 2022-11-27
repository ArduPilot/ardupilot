#pragma once

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Analog.h"

#if AP_BATTMON_SYNTHETIC_CURRENT_ENABLED
class AP_BattMonitor_Synthetic_Current : public AP_BattMonitor_Analog
{
public:

    /// Constructor
    AP_BattMonitor_Synthetic_Current(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params);

    /// Read the battery voltage and current.  Should be called at 10hz
    void read() override;

    /// returns true if battery monitor provides consumed energy info
    bool has_consumed_energy() const override { return true; }

    /// returns true if battery monitor provides current info
    bool has_current() const override { return true; }

    void init(void) override {}

    static const struct AP_Param::GroupInfo var_info[];

protected:

    AP_Float    _max_voltage;           /// maximum battery voltage used in current caluculation   
};
#endif
