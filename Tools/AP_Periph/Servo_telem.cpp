
#include "AP_Periph.h"

#if AP_SERVO_TELEM_ENABLED

#include <dronecan_msgs.h>

// Send servo telem message occasionally
void AP_Periph_FW::servo_telem_update()
{
    const uint32_t now_ms = AP_HAL::millis();

    // Run update function at 50hz mirroring vehicle update rate
    if (now_ms - servo_telem.last_update_ms > 20) {
        // Update periph only actuator telem
#if AP_PERIPH_ACTUATOR_TELEM_ENABLED
        actuator_telem.update();
#endif

        // Update main servo telem lib
        servo_telem.lib.update();
        servo_telem.last_update_ms = now_ms;
    }

    // Reporting is disabled
    if (g.servo_telem_msg_rate <= 0) {
        return;
    }

    // Check if its time to send the next instance
    if (now_ms - servo_telem.last_send_ms < (1000U / g.servo_telem_msg_rate)) {
        return;
    }
    servo_telem.last_send_ms = now_ms;

    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        // Send each servo in turn
        const uint8_t index = (servo_telem.last_send_index + 1 + i) % NUM_SERVO_CHANNELS;

        // Move on to next instance if no data is available
        AP_Servo_Telem::TelemetryData data;
        if (!servo_telem.lib.get_telem(index, data)) {
            continue;
        }

        // Don't send stale data, timeout after five seconds
        if (AP_HAL::timeout_expired(data.last_update_ms, now_ms, 5000U)) {
            continue;
        }

        // Blank packet
        const float nan = nanf("");
        uavcan_equipment_actuator_Status pkt {
            actuator_id: (uint8_t)(index + 1), // One based indexing for DroneCAN transport
            position: nan,
            force: nan,
            speed: nan,
            power_rating_pct: UAVCAN_EQUIPMENT_ACTUATOR_STATUS_POWER_RATING_PCT_UNKNOWN
        };

        // Fill in present data
        if (data.present(AP_Servo_Telem::TelemetryData::Types::MEASURED_POSITION)) {
            pkt.position = radians(data.measured_position);
        }
        if (data.present(AP_Servo_Telem::TelemetryData::Types::FORCE)) {
            pkt.force = data.force;
        }
        if (data.present(AP_Servo_Telem::TelemetryData::Types::SPEED)) {
            pkt.speed = data.speed;
        }
        if (data.present(AP_Servo_Telem::TelemetryData::Types::DUTY_CYCLE)) {
            pkt.power_rating_pct = data.duty_cycle;
        }

        // Encode message into buffer
        uint8_t buffer[UAVCAN_EQUIPMENT_ACTUATOR_STATUS_MAX_SIZE];
        const uint16_t total_size = uavcan_equipment_actuator_Status_encode(&pkt, buffer, !canfdout());

        // Send message
        canard_broadcast(UAVCAN_EQUIPMENT_ACTUATOR_STATUS_SIGNATURE,
                         UAVCAN_EQUIPMENT_ACTUATOR_STATUS_ID,
                         CANARD_TRANSFER_PRIORITY_LOW,
                         &buffer[0],
                         total_size);

        servo_telem.last_send_index = index;
        break;
    }
}

#endif // AP_SERVO_TELEM_ENABLED
