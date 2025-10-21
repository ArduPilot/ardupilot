#include "AP_BattMonitor_config.h"

#if AP_BATTERY_SCRIPTING_ENABLED

#include "AP_BattMonitor_Scripting.h"

#define AP_BATT_MONITOR_SCRIPTING_TIMEOUT_US 5000000

bool AP_BattMonitor_Scripting::capacity_remaining_pct(uint8_t &percentage) const
{
    if (internal_state.capacity_remaining_pct != UINT8_MAX) {
        percentage = internal_state.capacity_remaining_pct;
        return true;
    }
    // Fall back to default implementation
    return AP_BattMonitor_Backend::capacity_remaining_pct(percentage);
}

bool AP_BattMonitor_Scripting::get_cycle_count(uint16_t &cycles) const
{
    if (internal_state.cycle_count == UINT16_MAX) {
        return false;
    }
    cycles = internal_state.cycle_count;
    return true;
}

// Called by frontend to update the state. Called at 10Hz
void AP_BattMonitor_Scripting::read()
{
    WITH_SEMAPHORE(sem);

    // Check for timeout, to prevent a faulty script from appearing healthy
    if (last_update_us == 0 || AP_HAL::micros() - last_update_us > AP_BATT_MONITOR_SCRIPTING_TIMEOUT_US) {
        _state.healthy = false;
        return;
    }

    if (_state.last_time_micros == last_update_us) {
        // No new data
        return;
    }

    /*
      the script can fill in voltages up to 32 cells, for mavlink reporting
      the extra cell voltages get distributed over the max of 14 for mavlink
     */
    for (uint8_t i = 0; i < MIN(AP_BATT_MONITOR_CELLS_MAX,internal_state.cell_count); i++) {
        _state.cell_voltages.cells[i] = internal_state.cell_voltages[i];
    }
    _state.voltage = internal_state.voltage;
    if (!isnan(internal_state.current_amps)) {
        _state.current_amps = internal_state.current_amps;
    }
    if (!isnan(internal_state.consumed_mah)) {
        _state.consumed_mah = internal_state.consumed_mah;
    }
    // Overide integrated consumed energy with script value if it has been set
    if (!isnan(internal_state.consumed_wh)) {
        _state.consumed_wh = internal_state.consumed_wh;
    }
    if (!isnan(internal_state.temperature)) {
        _state.temperature = internal_state.temperature;
    }

    _state.healthy = internal_state.healthy;

    // Update the timestamp (has to be done after the consumed_mah calculation)
    _state.last_time_micros = last_update_us;
}

bool AP_BattMonitor_Scripting::handle_scripting(const BattMonitorScript_State &battmon_state)
{
    WITH_SEMAPHORE(sem);
    internal_state = battmon_state;
    const uint32_t now_us = AP_HAL::micros();
    uint32_t dt_us = now_us - last_update_us;
    if (last_update_us != 0 && !isnan(internal_state.current_amps) && isnan(internal_state.consumed_mah)) {
        AP_BattMonitor_Backend::update_consumed(_state, dt_us);
    }
    last_update_us = now_us;
    return true;
}

#endif // AP_BATTERY_SCRIPTING_ENABLED
