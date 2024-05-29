#include "AP_BattMonitor_config.h"

#if AP_BATTERY_UAVCAN_BATTERYINFO_ENABLED

#include <AP_HAL/AP_HAL.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_DroneCAN.h"

#include <AP_CANManager/AP_CANManager.h>
#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>
#include <AP_DroneCAN/AP_DroneCAN.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

#define LOG_TAG "BattMon"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_BattMonitor_DroneCAN::var_info[] = {

    // @Param: CURR_MULT
    // @DisplayName: Scales reported power monitor current
    // @Description: Multiplier applied to all current related reports to allow for adjustment if no UAVCAN param access or current splitting applications
    // @Range: .1 10
    // @User: Advanced
    AP_GROUPINFO("CURR_MULT", 30, AP_BattMonitor_DroneCAN, _curr_mult, 1.0),

    // Param indexes must be between 30 and 35 to avoid conflict with other battery monitor param tables loaded by pointer

    AP_GROUPEND
};

/// Constructor
AP_BattMonitor_DroneCAN::AP_BattMonitor_DroneCAN(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, BattMonitor_DroneCAN_Type type, AP_BattMonitor_Params &params) :
    AP_BattMonitor_Backend(mon, mon_state, params)
{
    AP_Param::setup_object_defaults(this,var_info);
    _state.var_info = var_info;

    // starts with not healthy
    _state.healthy = false;
}

void AP_BattMonitor_DroneCAN::subscribe_msgs(AP_DroneCAN* ap_dronecan)
{
    if (ap_dronecan == nullptr) {
        return;
    }

    if (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_battery_info_trampoline, ap_dronecan->get_driver_index()) == nullptr) {
        AP_BoardConfig::allocation_error("battinfo_sub");
    }

    if (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_battery_info_aux_trampoline, ap_dronecan->get_driver_index()) == nullptr) {
        AP_BoardConfig::allocation_error("battinfo_aux_sub");
    }

    if (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_mppt_stream_trampoline, ap_dronecan->get_driver_index()) == nullptr) {
        AP_BoardConfig::allocation_error("mppt_stream_sub");
    }
}

/*
  match a battery ID to driver serial number
  when serial number is negative, all batteries are accepted, otherwise it must match
*/
bool AP_BattMonitor_DroneCAN::match_battery_id(uint8_t instance, uint8_t battery_id)
{
    const auto serial_num = AP::battery().get_serial_number(instance);
    return serial_num < 0 || serial_num == (int32_t)battery_id;
}

AP_BattMonitor_DroneCAN* AP_BattMonitor_DroneCAN::get_dronecan_backend(AP_DroneCAN* ap_dronecan, uint8_t node_id, uint8_t battery_id)
{
    if (ap_dronecan == nullptr) {
        return nullptr;
    }
    const auto &batt = AP::battery();
    for (uint8_t i = 0; i < batt._num_instances; i++) {
        if (batt.drivers[i] == nullptr ||
            batt.get_type(i) != AP_BattMonitor::Type::UAVCAN_BatteryInfo) {
            continue;
        }
        AP_BattMonitor_DroneCAN* driver = (AP_BattMonitor_DroneCAN*)batt.drivers[i];
        if (driver->_ap_dronecan == ap_dronecan && driver->_node_id == node_id && match_battery_id(i, battery_id)) {
            return driver;
        }
    }
    // find empty uavcan driver
    for (uint8_t i = 0; i < batt._num_instances; i++) {
        if (batt.drivers[i] != nullptr &&
            batt.get_type(i) == AP_BattMonitor::Type::UAVCAN_BatteryInfo &&
            match_battery_id(i, battery_id)) {

            AP_BattMonitor_DroneCAN* batmon = (AP_BattMonitor_DroneCAN*)batt.drivers[i];
            if(batmon->_ap_dronecan != nullptr || batmon->_node_id != 0) {
                continue;
            }
            batmon->_ap_dronecan = ap_dronecan;
            batmon->_node_id = node_id;
            batmon->_instance = i;
            batmon->init();
            AP::can().log_text(AP_CANManager::LOG_INFO,
                            LOG_TAG,
                            "Registered BattMonitor Node %d on Bus %d\n",
                            node_id,
                            ap_dronecan->get_driver_index());
            return batmon;
        }
    }
    return nullptr;
}

void AP_BattMonitor_DroneCAN::handle_battery_info(const uavcan_equipment_power_BatteryInfo &msg)
{
    update_interim_state(msg.voltage, msg.current, msg.temperature, msg.state_of_charge_pct, msg.state_of_health_pct); 

    WITH_SEMAPHORE(_sem_battmon);
    _remaining_capacity_wh = msg.remaining_capacity_wh;
    _full_charge_capacity_wh = msg.full_charge_capacity_wh;

    // consume state of health
    if (msg.state_of_health_pct != UAVCAN_EQUIPMENT_POWER_BATTERYINFO_STATE_OF_HEALTH_UNKNOWN) {
        _interim_state.state_of_health_pct = msg.state_of_health_pct;
        _interim_state.has_state_of_health_pct = true;
    }
}

void AP_BattMonitor_DroneCAN::update_interim_state(const float voltage, const float current, const float temperature_K, const uint8_t soc, uint8_t soh_pct)
{
    WITH_SEMAPHORE(_sem_battmon);

    _interim_state.voltage = voltage;
    _interim_state.current_amps = _curr_mult * current;
    _soc = soc;

    if (!isnan(temperature_K) && temperature_K > 0) {
        // Temperature reported from battery in kelvin and stored internally in Celsius.
        _interim_state.temperature = KELVIN_TO_C(temperature_K);
        _interim_state.temperature_time = AP_HAL::millis();
    }

    const uint32_t tnow = AP_HAL::micros();

    if (!_has_battery_info_aux ||
        !use_CAN_SoC()) {
        const uint32_t dt_us = tnow - _interim_state.last_time_micros;

        // update total current drawn since startup
        update_consumed(_interim_state, dt_us);
    }

    // state of health
    if (soh_pct != UAVCAN_EQUIPMENT_POWER_BATTERYINFO_STATE_OF_HEALTH_UNKNOWN) {
        _interim_state.state_of_health_pct = soh_pct;
        _interim_state.has_state_of_health_pct = true;
    }

    // record time
    _interim_state.last_time_micros = tnow;
    _interim_state.healthy = true;
}

void AP_BattMonitor_DroneCAN::handle_battery_info_aux(const ardupilot_equipment_power_BatteryInfoAux &msg)
{
    WITH_SEMAPHORE(_sem_battmon);
    uint8_t cell_count = MIN(ARRAY_SIZE(_interim_state.cell_voltages.cells), msg.voltage_cell.len);

    _cycle_count = msg.cycle_count;
    for (uint8_t i = 0; i < cell_count; i++) {
        _interim_state.cell_voltages.cells[i] = msg.voltage_cell.data[i] * 1000;
    }
    _interim_state.is_powering_off = msg.is_powering_off;
    if (!isnan(msg.nominal_voltage) && msg.nominal_voltage > 0) {
        float remaining_capacity_ah = _remaining_capacity_wh / msg.nominal_voltage;
        float full_charge_capacity_ah = _full_charge_capacity_wh / msg.nominal_voltage;
        _interim_state.consumed_mah = (full_charge_capacity_ah - remaining_capacity_ah) * 1000;
        _interim_state.consumed_wh = _full_charge_capacity_wh - _remaining_capacity_wh;
        _interim_state.time_remaining =  is_zero(_interim_state.current_amps) ? 0 : (remaining_capacity_ah / _interim_state.current_amps * 3600);
        _interim_state.has_time_remaining = true;
    }

    _has_cell_voltages = true;
    _has_battery_info_aux = true;
}

void AP_BattMonitor_DroneCAN::handle_mppt_stream(const mppt_Stream &msg)
{
    const bool use_input_value = option_is_set(AP_BattMonitor_Params::Options::MPPT_Use_Input_Value);
    const float voltage = use_input_value ? msg.input_voltage : msg.output_voltage;
    const float current = use_input_value ? msg.input_current : msg.output_current;

    // use an invalid soc so we use the library calculated one
    const uint8_t soc = 127;

    // convert C to Kelvin
    const float temperature_K = isnan(msg.temperature) ? 0 : C_TO_KELVIN(msg.temperature);

    update_interim_state(voltage, current, temperature_K, soc, UAVCAN_EQUIPMENT_POWER_BATTERYINFO_STATE_OF_HEALTH_UNKNOWN); 

    if (!_mppt.is_detected) {
        // this is the first time the mppt message has been received
        // so set powered up state
        _mppt.is_detected = true;

        // Boot/Power-up event
        if (option_is_set(AP_BattMonitor_Params::Options::MPPT_Power_On_At_Boot)) {
            mppt_set_powered_state(true);
        } else if (option_is_set(AP_BattMonitor_Params::Options::MPPT_Power_Off_At_Boot)) {
            mppt_set_powered_state(false);
        }
    }

#if AP_BATTMONITOR_UAVCAN_MPPT_DEBUG
    if (_mppt.fault_flags != msg.fault_flags) {
        mppt_report_faults(_instance, msg.fault_flags);
    }
#endif
    _mppt.fault_flags = msg.fault_flags;
}

void AP_BattMonitor_DroneCAN::handle_battery_info_trampoline(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_power_BatteryInfo &msg)
{
    AP_BattMonitor_DroneCAN* driver = get_dronecan_backend(ap_dronecan, transfer.source_node_id, msg.battery_id);
    if (driver == nullptr) {
        return;
    }
    driver->handle_battery_info(msg);
}

void AP_BattMonitor_DroneCAN::handle_battery_info_aux_trampoline(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const ardupilot_equipment_power_BatteryInfoAux &msg)
{
    const auto &batt = AP::battery();
    AP_BattMonitor_DroneCAN *driver = nullptr;

    /*
      check for a backend with AllowSplitAuxInfo set, allowing InfoAux
      from a different CAN node than the base battery information
     */
    for (uint8_t i = 0; i < batt._num_instances; i++) {
        const auto *drv = batt.drivers[i];
        if (drv != nullptr &&
            batt.get_type(i) == AP_BattMonitor::Type::UAVCAN_BatteryInfo &&
            drv->option_is_set(AP_BattMonitor_Params::Options::AllowSplitAuxInfo) &&
            batt.get_serial_number(i) == int32_t(msg.battery_id)) {
            driver = (AP_BattMonitor_DroneCAN *)batt.drivers[i];
            if (driver->_ap_dronecan == nullptr) {
                /* we have not received the main battery information
                   yet. Discard InfoAux until we do so we can init the
                   backend with the right node ID
                 */
                return;
            }
            break;
        }
    }
    if (driver == nullptr) {
        driver = get_dronecan_backend(ap_dronecan, transfer.source_node_id, msg.battery_id);
    }
    if (driver == nullptr) {
        return;
    }
    driver->handle_battery_info_aux(msg);
}

void AP_BattMonitor_DroneCAN::handle_mppt_stream_trampoline(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const mppt_Stream &msg)
{
    AP_BattMonitor_DroneCAN* driver = get_dronecan_backend(ap_dronecan, transfer.source_node_id, transfer.source_node_id);
    if (driver == nullptr) {
        return;
    }
    driver->handle_mppt_stream(msg);
}

// read - read the voltage and current
void AP_BattMonitor_DroneCAN::read()
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
    _state.state_of_health_pct = _interim_state.state_of_health_pct;
    _state.has_state_of_health_pct = _interim_state.has_state_of_health_pct;
    memcpy(_state.cell_voltages.cells, _interim_state.cell_voltages.cells, sizeof(_state.cell_voltages));

    _has_temperature = (AP_HAL::millis() - _state.temperature_time) <= AP_BATT_MONITOR_TIMEOUT;

    // check if MPPT should be powered on/off depending upon arming state
    if (_mppt.is_detected) {
        mppt_check_powered_state();
    }
}

// Return true if the DroneCAN state of charge should be used.
// Return false if state of charge should be calculated locally by counting mah.
bool AP_BattMonitor_DroneCAN::use_CAN_SoC() const
{
    // a UAVCAN battery monitor may not be able to supply a state of charge. If it can't then
    // the user can set the option to use current integration in the backend instead.
    // SOC of 127 is used as an invalid SOC flag ie system configuration errors or SOC estimation unavailable
    return !(option_is_set(AP_BattMonitor_Params::Options::Ignore_UAVCAN_SoC) ||
            _mppt.is_detected ||
            (_soc == 127));
}

/// capacity_remaining_pct - returns true if the percentage is valid and writes to percentage argument
bool AP_BattMonitor_DroneCAN::capacity_remaining_pct(uint8_t &percentage) const
{
    if (!use_CAN_SoC()) {
        return AP_BattMonitor_Backend::capacity_remaining_pct(percentage);
    }

    // the monitor must have current readings in order to estimate consumed_mah and be healthy
    if (!has_current() || !_state.healthy) {
        return false;
    }

    percentage = _soc;
    return true;
}

// reset remaining percentage to given value
bool AP_BattMonitor_DroneCAN::reset_remaining(float percentage)
{
    if (use_CAN_SoC()) {
        // Cannot reset external state of charge
        return false;
    }

    WITH_SEMAPHORE(_sem_battmon);

    if (!AP_BattMonitor_Backend::reset_remaining(percentage)) {
        // Base class reset failed
        return false;
    }

    // Reset interim state that is used internally, this is then copied back to the main state in the read() call
    _interim_state.consumed_mah = _state.consumed_mah;
    _interim_state.consumed_wh = _state.consumed_wh;
    return true;
}

/// get_cycle_count - return true if cycle count can be provided and fills in cycles argument
bool AP_BattMonitor_DroneCAN::get_cycle_count(uint16_t &cycles) const
{
    if (_has_battery_info_aux) {
        cycles = _cycle_count;
        return true;
    }
    return false;
}

// request MPPT board to power on/off depending upon vehicle arming state as specified by BATT_OPTIONS
void AP_BattMonitor_DroneCAN::mppt_check_powered_state()
{
    if ((_mppt.powered_state_remote_ms != 0) && (AP_HAL::millis() - _mppt.powered_state_remote_ms >= 1000)) {
        // there's already a set attempt that didnt' respond. Retry at 1Hz
        mppt_set_powered_state(_mppt.powered_state);
    }

    // check if vehicle armed state has changed
    const bool vehicle_armed = hal.util->get_soft_armed();
    if ((!_mppt.vehicle_armed_last && vehicle_armed) && option_is_set(AP_BattMonitor_Params::Options::MPPT_Power_On_At_Arm)) {
        // arm event
        mppt_set_powered_state(true);
    } else if ((_mppt.vehicle_armed_last && !vehicle_armed) && option_is_set(AP_BattMonitor_Params::Options::MPPT_Power_Off_At_Disarm)) {
        // disarm event
        mppt_set_powered_state(false);
    }
    _mppt.vehicle_armed_last = vehicle_armed;
}

// request MPPT board to power on or off
// power_on should be true to power on the MPPT, false to power off
// force should be true to force sending the state change request to the MPPT
void AP_BattMonitor_DroneCAN::mppt_set_powered_state(bool power_on)
{
    if (_ap_dronecan == nullptr || !_mppt.is_detected) {
        return;
    }

    _mppt.powered_state = power_on;

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Battery %u: powering %s%s", (unsigned)_instance+1, _mppt.powered_state ? "ON" : "OFF",
        (_mppt.powered_state_remote_ms == 0) ? "" : " Retry");

    mppt_OutputEnableRequest request;
    request.enable = _mppt.powered_state;
    request.disable = !request.enable;

    if (mppt_outputenable_client == nullptr) {
        mppt_outputenable_client = new Canard::Client<mppt_OutputEnableResponse>{_ap_dronecan->get_canard_iface(), mppt_outputenable_res_cb};
        if (mppt_outputenable_client == nullptr) {
            return;
        }
    }
    mppt_outputenable_client->request(_node_id, request);
}

// callback from outputEnable to verify it is enabled or disabled
void AP_BattMonitor_DroneCAN::handle_outputEnable_response(const CanardRxTransfer& transfer, const mppt_OutputEnableResponse& response)
{
    if (transfer.source_node_id != _node_id) {
        // this response is not from the node we are looking for
        return;
    }

    if (response.enabled == _mppt.powered_state) {
        // we got back what we expected it to be. We set it on, it now says it on (or vice versa).
        // Clear the timer so we don't re-request
        _mppt.powered_state_remote_ms = 0;
    }
}

#if AP_BATTMONITOR_UAVCAN_MPPT_DEBUG
// report changes in MPPT faults
void AP_BattMonitor_DroneCAN::mppt_report_faults(const uint8_t instance, const uint8_t fault_flags)
{
    // handle recovery
    if (fault_flags == 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Battery %u: OK", (unsigned)instance+1);
        return;
    }

    // send battery faults via text messages
    for (uint8_t fault_bit=0x01; fault_bit <= 0x08; fault_bit <<= 1) {
        // this loop is to generate multiple messages if there are multiple concurrent faults, but also run once if there are no faults
        if ((fault_bit & fault_flags) != 0) {
            const MPPT_FaultFlags err = (MPPT_FaultFlags)fault_bit;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Battery %u: %s", (unsigned)instance+1, mppt_fault_string(err));
        }
    }
}

// returns string description of MPPT fault bit. Only handles single bit faults
const char* AP_BattMonitor_DroneCAN::mppt_fault_string(const MPPT_FaultFlags fault)
{
    switch (fault) {
        case MPPT_FaultFlags::OVER_VOLTAGE:
            return "over voltage";
        case MPPT_FaultFlags::UNDER_VOLTAGE:
            return "under voltage";
        case MPPT_FaultFlags::OVER_CURRENT:
            return "over current";
        case MPPT_FaultFlags::OVER_TEMPERATURE:
            return "over temp";
    }
    return "unknown";
}
#endif

// return mavlink fault bitmask (see MAV_BATTERY_FAULT enum)
uint32_t AP_BattMonitor_DroneCAN::get_mavlink_fault_bitmask() const
{
    // return immediately if not mppt or no faults
    if (!_mppt.is_detected || (_mppt.fault_flags == 0)) {
        return 0;
    }

    // convert mppt fault bitmask to mavlink fault bitmask
    uint32_t mav_fault_bitmask = 0;
    if ((_mppt.fault_flags & (uint8_t)MPPT_FaultFlags::OVER_VOLTAGE) || (_mppt.fault_flags & (uint8_t)MPPT_FaultFlags::UNDER_VOLTAGE)) {
        mav_fault_bitmask |= MAV_BATTERY_FAULT_INCOMPATIBLE_VOLTAGE;
    }
    if (_mppt.fault_flags & (uint8_t)MPPT_FaultFlags::OVER_CURRENT) {
        mav_fault_bitmask |= MAV_BATTERY_FAULT_OVER_CURRENT;
    }
    if (_mppt.fault_flags & (uint8_t)MPPT_FaultFlags::OVER_TEMPERATURE) {
        mav_fault_bitmask |= MAV_BATTERY_FAULT_OVER_TEMPERATURE;
    }
    return mav_fault_bitmask;
}

#endif  // AP_BATTERY_UAVCAN_BATTERYINFO_ENABLED
