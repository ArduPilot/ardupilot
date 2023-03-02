#pragma once

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"

class AP_BattMonitor_Sum : public AP_BattMonitor_Backend
{
public:

    /// Constructor
    AP_BattMonitor_Sum(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params, uint8_t instance);

    /// Read the battery voltage and current.  Should be called at 10hz
    void read() override;

    /// returns true if battery monitor provides consumed energy info
    bool has_consumed_energy() const override { return has_current(); }

    /// returns true if battery monitor provides current info
    bool has_current() const override { return _has_current; }

    void init(void) override {}

    static const struct AP_Param::GroupInfo var_info[];

private:

    // Return true if this instance should be included in calculation
    bool include_instance(uint8_t i) const;

    // Read sum of monitors
    void read_sum();

    // Read max of monitors
    void read_max();

    AP_Int16  _sum_mask;
    uint8_t _instance;
    bool _has_current;
};
