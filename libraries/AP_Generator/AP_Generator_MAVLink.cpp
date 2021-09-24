#include "AP_Generator_MAVLink.h"

#if HAL_GENERATOR_MAVLINK_ENABLED

void AP_Generator_MAVLink::update()
{
    update_frontend();
}

bool AP_Generator_MAVLink::healthy() const
{
    const uint32_t now = AP_HAL::millis();
    if (now - last_received_ms > 1000) {
        return false;
    }

    // this is a list of flags which are allowed to be set in status
    // which are *not* considered faults!
    const uint64_t not_error_flags =
        MAV_GENERATOR_STATUS_FLAG_READY |
        MAV_GENERATOR_STATUS_FLAG_GENERATING |
        MAV_GENERATOR_STATUS_FLAG_CHARGING |
        MAV_GENERATOR_STATUS_FLAG_MAXPOWER |
        MAV_GENERATOR_STATUS_FLAG_WARMING_UP |
        MAV_GENERATOR_STATUS_FLAG_IDLE;

    const uint64_t errors = packet.status & ~not_error_flags;
    if (errors != 0) {
        gcs().send_text(MAV_SEVERITY_WARNING, "errors present");
        return false;
    }

    return true;
}

void AP_Generator_MAVLink::handle_mavlink_msg(const GCS_MAVLINK &channel, const mavlink_message_t &msg)
{
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_GENERATOR_STATUS:
        // decode directly into our state:
        mavlink_msg_generator_status_decode(&msg, &packet);
        // fill in frontend-variables-to-copy
        _voltage = packet.bus_voltage;
        _current = packet.battery_current;
        _rpm = packet.generator_speed;
        last_received_ms = AP_HAL::millis();
        gcs().send_text(MAV_SEVERITY_WARNING, "Generator_MAVLink: rpm=%u", packet.generator_speed);
    }
}

void AP_Generator_MAVLink::send_generator_status(const GCS_MAVLINK &channel)
{
    mavlink_msg_generator_status_send(
        channel.get_chan(),
        packet.status,
        packet.generator_speed,
        packet.battery_current,
        packet.load_current,
        packet.power_generated,
        packet.bus_voltage,
        packet.rectifier_temperature,
        packet.bat_current_setpoint,
        packet.generator_temperature,
        packet.runtime,
        packet.time_until_maintenance
        );
}

#endif // HAL_GENERATOR_MAVLINK_ENABLED
