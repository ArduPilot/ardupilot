#include "AP_EFI_MAVLink.h"

#if HAL_EFI_MAVLINK_ENABLED

void AP_EFI_MAVLink::handle_mavlink_msg(const GCS_MAVLINK &channel, const mavlink_message_t &msg)
{
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_EFI_STATUS:
        mavlink_msg_efi_status_decode(&msg, &packet);

        struct EFI_State &s = internal_state;
        const mavlink_efi_status_t &p = packet;

        s.ecu_index = p.ecu_index;
        s.engine_speed_rpm = p.rpm;
        s.estimated_consumed_fuel_volume_cm3 = p.fuel_consumed;
        s.fuel_consumption_rate_cm3pm = p.fuel_flow;
        s.engine_load_percent = p.engine_load;
        s.throttle_position_percent = p.throttle_position;
        s.spark_dwell_time_ms = p.spark_dwell_time;
        s.atmospheric_pressure_kpa = p.barometric_pressure;
        s.intake_manifold_pressure_kpa = p.intake_manifold_pressure;
        s.intake_manifold_temperature = p.intake_manifold_temperature;
        s.cylinder_status[0].cylinder_head_temperature = p.cylinder_head_temperature + C_TO_KELVIN;
        s.cylinder_status[0].ignition_timing_deg = p.ignition_timing;
        s.cylinder_status[0].injection_time_ms = p.injection_time;

        // copy the data to the front end
        copy_to_frontend();

        gcs().send_text(MAV_SEVERITY_WARNING, "EFI_MAVLink: barometric_pressure: %f", packet.barometric_pressure);
    }
}

void AP_EFI_MAVLink::update()
{
}

#endif  // HAL_EFI_MAVLINK_ENABLED
