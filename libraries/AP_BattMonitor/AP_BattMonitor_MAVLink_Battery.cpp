#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include "AP_BattMonitor_MAVLink_Battery.h"

AP_BattMonitor_MAVLink_Battery::AP_BattMonitor_MAVLink_Battery(AP_BattMonitor &mon,
                                             AP_BattMonitor::BattMonitor_State &mon_state,
                                             AP_BattMonitor_Params &params) :
    AP_BattMonitor_Backend(mon, mon_state, params)
{
	_state.healthy = false;
}

void AP_BattMonitor_MAVLink_Battery::read()
{
    if ((AP_HAL::micros() - _state.last_time_micros) > MAVLINK_SMART_BATTERY_TIMEOUT_MICROS) {
		_state.healthy = false;
		return;
	} 
	_state.healthy = true;	
   return;
}

bool AP_BattMonitor_MAVLink_Battery::capacity_remaining_pct(uint8_t &cap_remain_pct) const
{
    if (capacity_rem_pct != INT8_MIN) {
		/* positive capacity remaining is expected */
		if (capacity_rem_pct > 0) {
			cap_remain_pct = capacity_rem_pct;
		} else {
			cap_remain_pct = 0;
		}
    	return true;
	}
	return false;
}

bool AP_BattMonitor_MAVLink_Battery::get_cycle_count(uint16_t &cycles) const
{
    if (cycle_count != UINT16_MAX) {
		cycles = cycle_count;
    	return true;
	}
	return false;
}

void AP_BattMonitor_MAVLink_Battery::process_info_msg(const mavlink_message_t &msg)
{
	int32_t capacity = mavlink_msg_smart_battery_info_get_capacity_full(&msg);
	if (capacity != -1
		&& capacity != _params._pack_capacity) {
		_params._pack_capacity.set_and_notify(capacity);
	}
	cycle_count = mavlink_msg_smart_battery_info_get_cycle_count(&msg);
}

void AP_BattMonitor_MAVLink_Battery::process_status_msg(const mavlink_message_t &msg)
{
	__mavlink_battery_status_t bat_stat;
	mavlink_msg_battery_status_decode(&msg, &bat_stat);
	fault_bitmask = bat_stat.fault_bitmask;
	uint32_t volt_sum_mV = 0;
	for (uint8_t cell = 0; cell < CELLS_PER_BATTERY_MAVLINK_MSG_MAX; cell++) {
        if (bat_stat.voltages[cell] != UINT16_MAX) {
			volt_sum_mV += bat_stat.voltages[cell];
		}
		_state.cell_voltages.cells[cell] = bat_stat.voltages[cell];
	}
	_state.voltage = (float) volt_sum_mV * 0.001f;
	_state.current_amps = (float) bat_stat.current_battery * 0.01f;
	_state.consumed_mah = (float) bat_stat.current_consumed;
    _state.consumed_wh = (float) _state.consumed_mah;
	if (bat_stat.temperature == INT16_MAX) {
		have_temp = false;
	} else {
    	_state.temperature = (float) bat_stat.temperature * 0.01f;
		have_temp = true;
	}
	if (bat_stat.time_remaining < 0) {
		bat_stat.time_remaining *= -1;									/* if negative: time until charged */
	}
	_state.time_remaining = (uint32_t) bat_stat.time_remaining;
	_state.has_time_remaining = (bool) bat_stat.time_remaining;
	_state.last_time_micros = AP_HAL::micros();
	capacity_rem_pct = bat_stat.battery_remaining;
}

void AP_BattMonitor_MAVLink_Battery::handle_mavlink_battery_message(const mavlink_message_t &msg) {
	if (!accept_mavlink_battery_message(msg)) {
		return;
	}
	switch (msg.msgid) {
		case MAVLINK_MSG_ID_BATTERY_STATUS:
			process_status_msg(msg);
			break;
		case MAVLINK_MSG_ID_SMART_BATTERY_INFO:
			process_info_msg(msg);
			break;
		default:
			return;
	}
}

bool AP_BattMonitor_MAVLink_Battery::accept_mavlink_battery_message(const mavlink_message_t &msg) {
	if (!accept_sys_comp_id(msg)) {
		return false;
	}
	if (!check_driver_type(msg)) {
		return false;
	}
	return true;
}

/*
  other sysids may report their battery data (compid must be MAV_COMP_ID_BATTERY/MAV_COMP_ID_BATTERY2)
 */
bool
AP_BattMonitor_MAVLink_Battery::accept_sys_comp_id(const mavlink_message_t &msg)
{
	if (msg.sysid != mavlink_system.sysid
		|| (msg.compid != MAV_COMP_ID_BATTERY && msg.compid != MAV_COMP_ID_BATTERY2)) {
		return false;
	}
	return true;
}

bool AP_BattMonitor_MAVLink_Battery::check_driver_type(const mavlink_message_t &msg)
{
	return ((AP_BattMonitor::Type)_params._type.get() == AP_BattMonitor::Type::MAVLink_Battery);
}
