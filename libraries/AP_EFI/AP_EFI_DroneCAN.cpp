#include <AP_HAL/AP_HAL.h>

#include "AP_EFI_DroneCAN.h"

#if AP_EFI_DRONECAN_ENABLED

#include <AP_CANManager/AP_CANManager.h>
#include <AP_DroneCAN/AP_DroneCAN.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

extern const AP_HAL::HAL& hal;

AP_EFI_DroneCAN *AP_EFI_DroneCAN::driver;

// constructor
AP_EFI_DroneCAN::AP_EFI_DroneCAN(AP_EFI &_frontend) :
    AP_EFI_Backend(_frontend)
{
    driver = this;
}

// links the DroneCAN message to this backend
void AP_EFI_DroneCAN::subscribe_msgs(AP_DroneCAN *ap_dronecan)
{
    if (ap_dronecan == nullptr) {
        return;
    }

    if (Canard::allocate_sub_arg_callback(ap_dronecan, &trampoline_status, ap_dronecan->get_driver_index()) == nullptr) {
        AP_BoardConfig::allocation_error("status_sub");
    }
}

// Called from frontend to update with the readings received by handler
void AP_EFI_DroneCAN::update()
{
}

// EFI message handler
void AP_EFI_DroneCAN::trampoline_status(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_ice_reciprocating_Status &msg)
{
    if (driver == nullptr) {
        return;
    }
    driver->handle_status(msg);
}

/*
  handle reciprocating ICE status message from DroneCAN
 */
void AP_EFI_DroneCAN::handle_status(const uavcan_equipment_ice_reciprocating_Status &pkt)
{
    auto &istate = internal_state;

    // state maps 1:1 from Engine_State
    istate.engine_state = Engine_State(pkt.state);

    if (!(pkt.flags & UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_CRANKSHAFT_SENSOR_ERROR_SUPPORTED)) {
        istate.crankshaft_sensor_status = Crankshaft_Sensor_Status::NOT_SUPPORTED;
    } else if (pkt.flags & UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_CRANKSHAFT_SENSOR_ERROR) {
        istate.crankshaft_sensor_status = Crankshaft_Sensor_Status::ERROR;
    } else {
        istate.crankshaft_sensor_status = Crankshaft_Sensor_Status::OK;
    }

    if (!(pkt.flags & UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_TEMPERATURE_SUPPORTED)) {
        istate.temperature_status = Temperature_Status::NOT_SUPPORTED;
    } else if (pkt.flags & UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_TEMPERATURE_BELOW_NOMINAL) {
        istate.temperature_status = Temperature_Status::BELOW_NOMINAL;
    } else if (pkt.flags & UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_TEMPERATURE_ABOVE_NOMINAL) {
        istate.temperature_status = Temperature_Status::ABOVE_NOMINAL;
    } else if (pkt.flags & UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_TEMPERATURE_OVERHEATING) {
        istate.temperature_status = Temperature_Status::OVERHEATING;
    } else if (pkt.flags & UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_TEMPERATURE_EGT_ABOVE_NOMINAL) {
        istate.temperature_status = Temperature_Status::EGT_ABOVE_NOMINAL;
    } else {
        istate.temperature_status = Temperature_Status::OK;
    }

    if (!(pkt.flags & UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_FUEL_PRESSURE_SUPPORTED)) {
        istate.fuel_pressure_status = Fuel_Pressure_Status::NOT_SUPPORTED;
    } else if (pkt.flags & UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_FUEL_PRESSURE_BELOW_NOMINAL) {
        istate.fuel_pressure_status = Fuel_Pressure_Status::BELOW_NOMINAL;
    } else if (pkt.flags & UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_FUEL_PRESSURE_ABOVE_NOMINAL) {
        istate.fuel_pressure_status = Fuel_Pressure_Status::ABOVE_NOMINAL;
    } else {
        istate.fuel_pressure_status = Fuel_Pressure_Status::OK;
    }

    if (!(pkt.flags & UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_OIL_PRESSURE_SUPPORTED)) {
        istate.oil_pressure_status = Oil_Pressure_Status::NOT_SUPPORTED;
    } else if (pkt.flags & UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_OIL_PRESSURE_BELOW_NOMINAL) {
        istate.oil_pressure_status = Oil_Pressure_Status::BELOW_NOMINAL;
    } else if (pkt.flags & UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_OIL_PRESSURE_ABOVE_NOMINAL) {
        istate.oil_pressure_status = Oil_Pressure_Status::ABOVE_NOMINAL;
    } else {
        istate.oil_pressure_status = Oil_Pressure_Status::OK;
    }

    if (!(pkt.flags & UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_DETONATION_SUPPORTED)) {
        istate.detonation_status = Detonation_Status::NOT_SUPPORTED;
    } else if (pkt.flags & UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_DETONATION_OBSERVED) {
        istate.detonation_status = Detonation_Status::OBSERVED;
    } else {
        istate.detonation_status = Detonation_Status::NOT_OBSERVED;
    }

    if (!(pkt.flags & UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_MISFIRE_SUPPORTED)) {
        istate.misfire_status = Misfire_Status::NOT_SUPPORTED;
    } else if (pkt.flags & UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_MISFIRE_OBSERVED) {
        istate.misfire_status = Misfire_Status::OBSERVED;
    } else {
        istate.misfire_status = Misfire_Status::NOT_OBSERVED;
    }
    
    if (!(pkt.flags & UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_DEBRIS_SUPPORTED)) {
        istate.debris_status = Debris_Status::NOT_SUPPORTED;
    } else if (pkt.flags & UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_DEBRIS_DETECTED) {
        istate.debris_status = Debris_Status::DETECTED;
    } else {
        istate.debris_status = Debris_Status::NOT_DETECTED;
    }
    
    istate.engine_load_percent = pkt.engine_load_percent;
    istate.engine_speed_rpm = pkt.engine_speed_rpm;
    istate.spark_dwell_time_ms = pkt.spark_dwell_time_ms;
    istate.atmospheric_pressure_kpa = pkt.atmospheric_pressure_kpa;
    istate.intake_manifold_pressure_kpa = pkt.intake_manifold_pressure_kpa;
    istate.intake_manifold_temperature = pkt.intake_manifold_temperature;
    istate.coolant_temperature = pkt.coolant_temperature;
    istate.oil_pressure = pkt.oil_pressure;
    istate.oil_temperature = pkt.oil_temperature;
    istate.fuel_pressure = pkt.fuel_pressure;
    istate.fuel_consumption_rate_cm3pm = pkt.fuel_consumption_rate_cm3pm;
    istate.estimated_consumed_fuel_volume_cm3 = pkt.estimated_consumed_fuel_volume_cm3;
    istate.throttle_position_percent = pkt.throttle_position_percent;
    istate.ecu_index = pkt.ecu_index;

    // 1:1 for spark plug usage
    istate.spark_plug_usage = Spark_Plug_Usage(pkt.spark_plug_usage);

    // assume max one cylinder status
    if (pkt.cylinder_status.len > 0) {
        const auto &cs = pkt.cylinder_status.data[0];
        auto &c = istate.cylinder_status;
        c.ignition_timing_deg = cs.ignition_timing_deg;
        c.injection_time_ms = cs.injection_time_ms;
        c.cylinder_head_temperature = cs.cylinder_head_temperature;
        c.exhaust_gas_temperature = cs.exhaust_gas_temperature;
        c.lambda_coefficient = cs.lambda_coefficient;
    }

    copy_to_frontend();
}

#endif // AP_EFI_DRONECAN_ENABLED

