#include <AP_HAL/AP_HAL.h>

#if HAL_ENABLE_LIBUAVCAN_DRIVERS

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_UAVCAN.h"

#include <AP_CANManager/AP_CANManager.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_UAVCAN/AP_UAVCAN.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

#include <uavcan/equipment/power/BatteryInfo.hpp>
#include <ardupilot/equipment/power/BatteryInfoAux.hpp>

#define LOG_TAG "BattMon"

extern const AP_HAL::HAL& hal;

UC_REGISTRY_BINDER(BattInfoCb, uavcan::equipment::power::BatteryInfo);
UC_REGISTRY_BINDER(BattInfoAuxCb, ardupilot::equipment::power::BatteryInfoAux);

/// Constructor
AP_BattMonitor_UAVCAN::AP_BattMonitor_UAVCAN(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, BattMonitor_UAVCAN_Type type, AP_BattMonitor_Params &params) :
    AP_BattMonitor_Backend(mon, mon_state, params),
    _type(type)
{
    // starts with not healthy
    _state.healthy = false;
}

void AP_BattMonitor_UAVCAN::subscribe_msgs(AP_UAVCAN* ap_uavcan)
{
    if (ap_uavcan == nullptr) {
        return;
    }

    auto* node = ap_uavcan->get_node();

    uavcan::Subscriber<uavcan::equipment::power::BatteryInfo, BattInfoCb> *battinfo_listener;
    battinfo_listener = new uavcan::Subscriber<uavcan::equipment::power::BatteryInfo, BattInfoCb>(*node);
    // Backend Msg Handler
    const int battinfo_listener_res = battinfo_listener->start(BattInfoCb(ap_uavcan, &handle_battery_info_trampoline));
    if (battinfo_listener_res < 0) {
        AP_BoardConfig::allocation_error("UAVCAN BatteryInfo subscriber start problem\n\r");
        return;
    }

    uavcan::Subscriber<ardupilot::equipment::power::BatteryInfoAux, BattInfoAuxCb> *battinfo_aux_listener;
    battinfo_aux_listener = new uavcan::Subscriber<ardupilot::equipment::power::BatteryInfoAux, BattInfoAuxCb>(*node);
    // Backend Msg Handler
    const int battinfo_aux_listener_res = battinfo_aux_listener->start(BattInfoAuxCb(ap_uavcan, &handle_battery_info_aux_trampoline));
    if (battinfo_aux_listener_res < 0) {
        AP_BoardConfig::allocation_error("UAVCAN BatteryInfoAux subscriber start problem");
        return;
    }
}

AP_BattMonitor_UAVCAN* AP_BattMonitor_UAVCAN::get_uavcan_backend(AP_UAVCAN* ap_uavcan, uint8_t node_id, uint8_t battery_id)
{
    if (ap_uavcan == nullptr) {
        return nullptr;
    }
    for (uint8_t i = 0; i < AP::battery()._num_instances; i++) {
        if (AP::battery().drivers[i] == nullptr ||
            AP::battery().get_type(i) != AP_BattMonitor::Type::UAVCAN_BatteryInfo) {
            continue;
        }
        AP_BattMonitor_UAVCAN* driver = (AP_BattMonitor_UAVCAN*)AP::battery().drivers[i];
        if (driver->_ap_uavcan == ap_uavcan && driver->_node_id == node_id && match_battery_id(i, battery_id)) {
            return driver;
        }
    }
    // find empty uavcan driver
    for (uint8_t i = 0; i < AP::battery()._num_instances; i++) {
        if (AP::battery().drivers[i] != nullptr &&
            AP::battery().get_type(i) == AP_BattMonitor::Type::UAVCAN_BatteryInfo &&
            match_battery_id(i, battery_id)) {

            AP_BattMonitor_UAVCAN* batmon = (AP_BattMonitor_UAVCAN*)AP::battery().drivers[i];
            if(batmon->_ap_uavcan != nullptr || batmon->_node_id != 0) {
                continue;
            }
            batmon->_ap_uavcan = ap_uavcan;
            batmon->_node_id = node_id;
            batmon->init();
            AP::can().log_text(AP_CANManager::LOG_INFO,
                            LOG_TAG,
                            "Registered BattMonitor Node %d on Bus %d\n",
                            node_id,
                            ap_uavcan->get_driver_index());
            return batmon;
        }
    }
    return nullptr;
}

void AP_BattMonitor_UAVCAN::handle_battery_info(const BattInfoCb &cb)
{
    WITH_SEMAPHORE(_sem_battmon);
    _interim_state.voltage = cb.msg->voltage;
    _interim_state.current_amps = cb.msg->current;
    _soc = cb.msg->state_of_charge_pct;
    _remaining_capacity_wh = cb.msg->remaining_capacity_wh;
    _full_charge_capacity_wh = cb.msg->full_charge_capacity_wh;

    if (!isnanf(cb.msg->temperature) && cb.msg->temperature > 0) {
        // Temperature reported from battery in kelvin and stored internally in Celsius.
        _interim_state.temperature = cb.msg->temperature - C_TO_KELVIN;
        _interim_state.temperature_time = AP_HAL::millis();
    }

    uint32_t tnow = AP_HAL::micros();

    if (!_has_battery_info_aux) {
        uint32_t dt = tnow - _interim_state.last_time_micros;

        // update total current drawn since startup
        if (_interim_state.last_time_micros != 0 && dt < 2000000) {
            // .0002778 is 1/3600 (conversion to hours)
            float mah = (float) ((double) _interim_state.current_amps * (double) dt * (double) 0.0000002778f);
            _interim_state.consumed_mah += mah;
            _interim_state.consumed_wh  += 0.001f * mah * _interim_state.voltage;
        }
    }

    // record time
    _interim_state.last_time_micros = tnow;
    _interim_state.healthy = true;
}

void AP_BattMonitor_UAVCAN::handle_battery_info_aux(const BattInfoAuxCb &cb)
{
    WITH_SEMAPHORE(_sem_battmon);
    uint8_t cell_count = MIN(ARRAY_SIZE(_interim_state.cell_voltages.cells), cb.msg->voltage_cell.size());
    float remaining_capacity_ah = _remaining_capacity_wh / cb.msg->nominal_voltage;
    float full_charge_capacity_ah = _full_charge_capacity_wh / cb.msg->nominal_voltage;

    _cycle_count = cb.msg->cycle_count;
    for (uint8_t i = 0; i < cell_count; i++) {
        _interim_state.cell_voltages.cells[i] = cb.msg->voltage_cell[i] * 1000;
    }
    _interim_state.is_powering_off = cb.msg->is_powering_off;
    _interim_state.consumed_mah = (full_charge_capacity_ah - remaining_capacity_ah) * 1000;
    _interim_state.consumed_wh = _full_charge_capacity_wh - _remaining_capacity_wh;
    _interim_state.time_remaining =  is_zero(_interim_state.current_amps) ? 0 : (remaining_capacity_ah / _interim_state.current_amps * 3600);
    _interim_state.has_time_remaining = true;

    _has_cell_voltages = true;
    _has_time_remaining = true;
    _has_consumed_energy = true;
    _has_battery_info_aux = true;
}

void AP_BattMonitor_UAVCAN::handle_battery_info_trampoline(AP_UAVCAN* ap_uavcan, uint8_t node_id, const BattInfoCb &cb)
{
    AP_BattMonitor_UAVCAN* driver = get_uavcan_backend(ap_uavcan, node_id, cb.msg->battery_id);
    if (driver == nullptr) {
        return;
    }
    driver->handle_battery_info(cb);
}

void AP_BattMonitor_UAVCAN::handle_battery_info_aux_trampoline(AP_UAVCAN* ap_uavcan, uint8_t node_id, const BattInfoAuxCb &cb)
{
    AP_BattMonitor_UAVCAN* driver = get_uavcan_backend(ap_uavcan, node_id, cb.msg->battery_id);
    if (driver == nullptr) {
        return;
    }
    driver->handle_battery_info_aux(cb);
}

// read - read the voltage and current
void AP_BattMonitor_UAVCAN::read()
{
    uint32_t tnow = AP_HAL::micros();

    // timeout after 5 seconds
    if ((tnow - _interim_state.last_time_micros) > AP_BATTMONITOR_UAVCAN_TIMEOUT_MICROS) {
        _interim_state.healthy = false;
    }
    // Copy over relevant states over to main state
    WITH_SEMAPHORE(_sem_battmon);
    _state.temperature = _interim_state.temperature;
    _state.temperature_time = _interim_state.temperature_time;
    _state.voltage = _interim_state.voltage;
    _state.current_amps = _interim_state.current_amps;
    _state.consumed_mah = _interim_state.consumed_mah;
    _state.consumed_wh = _interim_state.consumed_wh;
    _state.last_time_micros = _interim_state.last_time_micros;
    _state.healthy = _interim_state.healthy;
    _state.time_remaining = _interim_state.time_remaining;
    _state.has_time_remaining = _interim_state.has_time_remaining;
    _state.is_powering_off = _interim_state.is_powering_off;
    memcpy(_state.cell_voltages.cells, _interim_state.cell_voltages.cells, sizeof(_state.cell_voltages));

    _has_temperature = (AP_HAL::millis() - _state.temperature_time) <= AP_BATT_MONITOR_TIMEOUT;
}

/// capacity_remaining_pct - returns true if the percentage is valid and writes to percentage argument
bool AP_BattMonitor_UAVCAN::capacity_remaining_pct(uint8_t &percentage) const
{
    if ((uint32_t(_params._options.get()) & uint32_t(AP_BattMonitor_Params::Options::Ignore_UAVCAN_SoC)) ||
        _soc > 100) {
        // a UAVCAN battery monitor may not be able to supply a state of charge. If it can't then
        // the user can set the option to use current integration in the backend instead.
        return AP_BattMonitor_Backend::capacity_remaining_pct(percentage);
    }

    // the monitor must have current readings in order to estimate consumed_mah and be healthy
    if (!has_current() || !_state.healthy) {
        return false;
    }

    percentage = _soc;
    return true;
}

/// get_cycle_count - return true if cycle count can be provided and fills in cycles argument
bool AP_BattMonitor_UAVCAN::get_cycle_count(uint16_t &cycles) const
{
    if (_has_battery_info_aux) {
        cycles = _cycle_count;
        return true;
    }
    return false;
}

#endif
