#include <AP_HAL/AP_HAL.h>

#if HAL_ENABLE_LIBUAVCAN_DRIVERS

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_UAVCAN.h"

#include <AP_CANManager/AP_CANManager.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_UAVCAN/AP_UAVCAN.h>

#include <uavcan/equipment/power/BatteryInfo.hpp>

#define LOG_TAG "BattMon"

extern const AP_HAL::HAL& hal;

UC_REGISTRY_BINDER(BattInfoCb, uavcan::equipment::power::BatteryInfo);

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
        AP_HAL::panic("UAVCAN BatteryInfo subscriber start problem\n\r");
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

    if (!isnanf(cb.msg->temperature) && cb.msg->temperature > 0) {
        // Temperature reported from battery in kelvin and stored internally in Celsius.
        _interim_state.temperature = cb.msg->temperature - C_TO_KELVIN;
        _interim_state.temperature_time = AP_HAL::millis();
    }

    uint32_t tnow = AP_HAL::micros();
    uint32_t dt = tnow - _interim_state.last_time_micros;

    // update total current drawn since startup
    if (_interim_state.last_time_micros != 0 && dt < 2000000) {
        // .0002778 is 1/3600 (conversion to hours)
        float mah = (float) ((double) _interim_state.current_amps * (double) dt * (double) 0.0000002778f);
        _interim_state.consumed_mah += mah;
        _interim_state.consumed_wh  += 0.001f * mah * _interim_state.voltage;
    }

    // record time
    _interim_state.last_time_micros = tnow;

    _interim_state.healthy = true;
}

void AP_BattMonitor_UAVCAN::handle_battery_info_trampoline(AP_UAVCAN* ap_uavcan, uint8_t node_id, const BattInfoCb &cb)
{
    AP_BattMonitor_UAVCAN* driver = get_uavcan_backend(ap_uavcan, node_id, cb.msg->battery_id);
    if (driver == nullptr) {
        return;
    }
    driver->handle_battery_info(cb);
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

    _has_temperature = (AP_HAL::millis() - _state.temperature_time) <= AP_BATT_MONITOR_TIMEOUT;
}

/// capacity_remaining_pct - returns the % battery capacity remaining (0 ~ 100)
uint8_t AP_BattMonitor_UAVCAN::capacity_remaining_pct() const
{
    if ((uint32_t(_params._options.get()) & uint32_t(AP_BattMonitor_Params::Options::Ignore_UAVCAN_SoC)) ||
        _soc > 100) {
        // a UAVCAN battery monitor may not be able to supply a state of charge. If it can't then
        // the user can set the option to use current integration in the backend instead.
        return AP_BattMonitor_Backend::capacity_remaining_pct();
    }
    return _soc;
}

#endif
