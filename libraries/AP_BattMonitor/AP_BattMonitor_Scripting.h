#pragma once

#include "AP_BattMonitor_Backend.h"

#if AP_BATTERY_SCRIPTING_ENABLED

class AP_BattMonitor_Scripting : public AP_BattMonitor_Backend
{
public:
    // Inherit constructor
    using AP_BattMonitor_Backend::AP_BattMonitor_Backend;

    bool has_current() const override { return last_update_us != 0 && !isnan(internal_state.current_amps); }
    bool has_consumed_energy() const override { return has_current(); }
    bool has_cell_voltages() const override { return internal_state.cell_count > 0; }
    bool has_temperature() const override { return last_update_us != 0 && !isnan(internal_state.temperature); }
    bool capacity_remaining_pct(uint8_t &percentage) const override;
    bool get_cycle_count(uint16_t &cycles) const override;

    void read() override;

    bool handle_scripting(const BattMonitorScript_State &battmon_state) override;

protected:
    BattMonitorScript_State internal_state;
    uint32_t last_update_us;

    HAL_Semaphore sem;
};

#endif // AP_BATTMONITOR_SCRIPTING_ENABLED

