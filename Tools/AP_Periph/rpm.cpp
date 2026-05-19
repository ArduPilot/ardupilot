#include "AP_Periph.h"

#if AP_PERIPH_RPM_STREAM_ENABLED

#include <dronecan_msgs.h>

// Send rpm message occasionally
void AP_Periph_FW::rpm_sensor_send(void)
{
    if (g.rpm_msg_rate <= 0) {
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - rpm_last_send_ms < (1000U / g.rpm_msg_rate)) {
        return;
    }
    rpm_last_send_ms = now_ms;

    {
        const uint8_t num_sensors = rpm_sensor.num_sensors();
        for (uint8_t i = 0; i < num_sensors; i++) {
            // Send each sensor in turn
            const uint8_t index = (rpm_last_sent_index + 1 + i) % num_sensors;

            const int8_t sensor_id = rpm_sensor.get_dronecan_sensor_id(index);
            if (sensor_id < 0) {
                // disabled or not configured to send
                continue;
            }

            dronecan_sensors_rpm_RPM pkt {};
            pkt.sensor_id = sensor_id;

            // Get rpm and set health flag
            if (!rpm_sensor.get_rpm(index, pkt.rpm)) {
                pkt.flags |= DRONECAN_SENSORS_RPM_RPM_FLAGS_UNHEALTHY;
            }

            uint8_t buffer[DRONECAN_SENSORS_RPM_RPM_MAX_SIZE];
            const uint16_t total_size = dronecan_sensors_rpm_RPM_encode(&pkt, buffer, !canfdout());

            canard_broadcast(DRONECAN_SENSORS_RPM_RPM_SIGNATURE,
                             DRONECAN_SENSORS_RPM_RPM_ID,
                             CANARD_TRANSFER_PRIORITY_LOW,
                             &buffer[0],
                             total_size);

            rpm_last_sent_index = index;
            break;
        }
    }
}

#endif // AP_PERIPH_RPM_STREAM_ENABLED
