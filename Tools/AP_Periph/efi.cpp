#include "AP_Periph.h"

#ifdef HAL_PERIPH_ENABLE_EFI

/*
  EFI support
 */

#include <dronecan_msgs.h>

/*
  update CAN EFI
 */
void AP_Periph_FW::can_efi_update(void)
{
    if (!efi.enabled()) {
        return;
    }
    efi.update();
    const uint32_t update_ms = efi.get_last_update_ms();
    if (!efi.is_healthy() || efi_update_ms == update_ms) {
        return;
    }
    efi_update_ms = update_ms;
    EFI_State state;
    efi.get_state(state);

    {
        /*
          send status packet
        */
        uavcan_equipment_ice_reciprocating_Status pkt {};

        // state maps 1:1 from Engine_State
        pkt.state = uint8_t(state.engine_state);

        switch (state.crankshaft_sensor_status) {
        case Crankshaft_Sensor_Status::NOT_SUPPORTED:
            break;
        case Crankshaft_Sensor_Status::OK:
            pkt.flags |= UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_CRANKSHAFT_SENSOR_ERROR_SUPPORTED;
            break;
        case Crankshaft_Sensor_Status::ERROR:
            pkt.flags |=
                UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_CRANKSHAFT_SENSOR_ERROR_SUPPORTED |
                UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_CRANKSHAFT_SENSOR_ERROR;
            break;
        }

        switch (state.temperature_status) {
        case Temperature_Status::NOT_SUPPORTED:
            break;
        case Temperature_Status::OK:
            pkt.flags |= UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_TEMPERATURE_SUPPORTED;
            break;
        case Temperature_Status::BELOW_NOMINAL:
            pkt.flags |=
                UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_TEMPERATURE_SUPPORTED |
                UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_TEMPERATURE_BELOW_NOMINAL;
            break;
        case Temperature_Status::ABOVE_NOMINAL:
            pkt.flags |=
                UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_TEMPERATURE_SUPPORTED |
                UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_TEMPERATURE_ABOVE_NOMINAL;
            break;
        case Temperature_Status::OVERHEATING:
            pkt.flags |=
                UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_TEMPERATURE_SUPPORTED |
                UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_TEMPERATURE_OVERHEATING;
            break;
        case Temperature_Status::EGT_ABOVE_NOMINAL:
            pkt.flags |=
                UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_TEMPERATURE_SUPPORTED |
                UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_TEMPERATURE_EGT_ABOVE_NOMINAL;
            break;
        }

        switch (state.fuel_pressure_status) {
        case Fuel_Pressure_Status::NOT_SUPPORTED:
            break;
        case Fuel_Pressure_Status::OK:
            pkt.flags |= UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_FUEL_PRESSURE_SUPPORTED;
            break;
        case Fuel_Pressure_Status::BELOW_NOMINAL:
            pkt.flags |=
                UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_FUEL_PRESSURE_SUPPORTED |
                UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_FUEL_PRESSURE_BELOW_NOMINAL;
            break;
        case Fuel_Pressure_Status::ABOVE_NOMINAL:
            pkt.flags |=
                UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_FUEL_PRESSURE_SUPPORTED |
                UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_FUEL_PRESSURE_ABOVE_NOMINAL;
            break;
        }

        switch (state.oil_pressure_status) {
        case Oil_Pressure_Status::NOT_SUPPORTED:
            break;
        case Oil_Pressure_Status::OK:
            pkt.flags |= UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_OIL_PRESSURE_SUPPORTED;
            break;
        case Oil_Pressure_Status::BELOW_NOMINAL:
            pkt.flags |=
                UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_OIL_PRESSURE_SUPPORTED |
                UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_OIL_PRESSURE_BELOW_NOMINAL;
            break;
        case Oil_Pressure_Status::ABOVE_NOMINAL:
            pkt.flags |=
                UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_OIL_PRESSURE_SUPPORTED |
                UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_OIL_PRESSURE_ABOVE_NOMINAL;
            break;
        }

        switch (state.detonation_status) {
        case Detonation_Status::NOT_SUPPORTED:
            break;
        case Detonation_Status::NOT_OBSERVED:
            pkt.flags |=
                UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_DETONATION_SUPPORTED;
            break;
        case Detonation_Status::OBSERVED:
            pkt.flags |=
                UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_DETONATION_SUPPORTED |
                UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_DETONATION_OBSERVED;
            break;
        }

        switch (state.misfire_status) {
        case Misfire_Status::NOT_SUPPORTED:
            break;
        case Misfire_Status::NOT_OBSERVED:
            pkt.flags |=
                UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_MISFIRE_SUPPORTED;
            break;
        case Misfire_Status::OBSERVED:
            pkt.flags |=
                UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_MISFIRE_SUPPORTED |
                UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_MISFIRE_OBSERVED;
            break;
        }

        switch (state.debris_status) {
        case Debris_Status::NOT_SUPPORTED:
            break;
        case Debris_Status::NOT_DETECTED:
            pkt.flags |=
                UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_DEBRIS_SUPPORTED;
            break;
        case Debris_Status::DETECTED:
            pkt.flags |=
                UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_DEBRIS_SUPPORTED |
                UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_DEBRIS_DETECTED;
            break;
        }

        pkt.engine_load_percent = state.engine_load_percent;
        pkt.engine_speed_rpm = state.engine_speed_rpm;
        pkt.spark_dwell_time_ms = state.spark_dwell_time_ms;
        pkt.atmospheric_pressure_kpa = state.atmospheric_pressure_kpa;
        pkt.intake_manifold_pressure_kpa = state.intake_manifold_pressure_kpa;
        pkt.intake_manifold_temperature = state.intake_manifold_temperature;
        pkt.coolant_temperature = state.coolant_temperature;
        pkt.oil_pressure = state.oil_pressure;
        pkt.oil_temperature = state.oil_temperature;
        pkt.fuel_pressure = state.fuel_pressure;
        pkt.fuel_consumption_rate_cm3pm = state.fuel_consumption_rate_cm3pm;
        pkt.estimated_consumed_fuel_volume_cm3 = state.estimated_consumed_fuel_volume_cm3;
        pkt.throttle_position_percent = state.throttle_position_percent;
        pkt.ecu_index = state.ecu_index;
        pkt.spark_plug_usage = uint8_t(state.spark_plug_usage);

        // assume single set of cylinder status
        pkt.cylinder_status.len = 1;
        auto &c = pkt.cylinder_status.data[0];
        const auto &state_c = state.cylinder_status;
        c.ignition_timing_deg = state_c.ignition_timing_deg;
        c.injection_time_ms = state_c.injection_time_ms;
        c.cylinder_head_temperature = state_c.cylinder_head_temperature;
        c.exhaust_gas_temperature = state_c.exhaust_gas_temperature;
        c.lambda_coefficient = state_c.lambda_coefficient;

        uint8_t buffer[UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_MAX_SIZE] {};
        const uint16_t total_size = uavcan_equipment_ice_reciprocating_Status_encode(&pkt, buffer, !canfdout());

        canard_broadcast(UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_SIGNATURE,
                        UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_ID,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        &buffer[0],
                        total_size);
    }
}

#endif // HAL_PERIPH_ENABLE_EFI
