#pragma once

#include "AP_BattMonitor_Backend.h"

#define AP_BATTMONITOR_JBDCAN_TIMEOUT_MICROS 5000000

class AP_BattMonitor_JBDCAN : public AP_BattMonitor_Backend {
public:
    AP_BattMonitor_JBDCAN(AP_BattMonitor &mon,
                          AP_BattMonitor::BattMonitor_State &mon_state,
                          AP_BattMonitor_Params &params);

    static const AP_Param::GroupInfo var_info[];

    void init() override;
    void read() override;
    bool capacity_remaining_pct(uint8_t &percentage) const override;

    bool has_current() const override { return true; }
    bool has_temperature() const override { return false; }
    bool has_consumed_energy() const override { return true; }
    bool has_time_remaining() const override { return false; }
    bool has_cell_voltages() const override { return false; }
};
