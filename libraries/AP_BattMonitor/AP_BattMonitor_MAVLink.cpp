#include "AP_BattMonitor_config.h"

#if AP_BATTERY_MAVLINK_ENABLED

#include "AP_BattMonitor_MAVLink.h"


// Constructor
AP_BattMonitor_MAVLink::AP_BattMonitor_MAVLink(AP_BattMonitor &mon,
                                               AP_BattMonitor::BattMonitor_State &mon_state,
                                               AP_BattMonitor_Params &params)
    : AP_BattMonitor_Backend(mon, mon_state, params)
{
    _state.healthy = true;
}

void AP_BattMonitor_MAVLink::handle_msg(const mavlink_message_t &msg)
{
    // decode MAVlink message
    mavlink_battery_status_t packet;
    mavlink_msg_battery_status_decode(&msg, &packet);

    _state.voltage = 0.0;
    // copy independent cells voltage from MAVLink msg
    for (uint8_t i = 0; i < MAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_LEN; i++)
    {
        _state.cell_voltages.cells[i] = packet.voltages[i];
        _state.voltage += packet.voltages[i] * 0.001;
    }

    // copy battery readings from the MAVLink message
    _state.current_amps = packet.current_battery;
    _state.consumed_mah = packet.current_consumed;
    _state.temperature = packet.temperature * 0.1;
    _state.time_remaining = packet.time_remaining;
    _state.last_time_micros = AP_HAL::micros();
    _remaining_pct = packet.battery_remaining;
}

void AP_BattMonitor_MAVLink::read()
{
    update_health();
}

// update battery health flag
void AP_BattMonitor_MAVLink::update_health()
{
    uint32_t current_time_micros = AP_HAL::micros();
    if (current_time_micros - _state.last_time_micros > AP_BATTMONITOR_MAVLINK_TIMEOUT_MICROS)
    {
        _state.healthy = false;
        _have_info = false;
        return;
    }
    _state.healthy = true;
    _have_info = true;
}

// capacity_remaining_pct - returns true if the battery % is available and writes to the percentage argument
bool AP_BattMonitor_MAVLink::capacity_remaining_pct(uint8_t &percentage) const
{
    if (_have_info)
    {
        percentage = _remaining_pct;
    }
    return _have_info;
}

#endif // AP_BATTERY_MAVLINK_ENABLED
