#include "AP_Periph.h"

#if AP_PERIPH_DEVICE_TEMPERATURE_ENABLED

#include <dronecan_msgs.h>

// Send temperature message occasionally
void AP_Periph_FW::temperature_sensor_update(void)
{
    if (g.temperature_msg_rate <= 0) {
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - temperature_last_send_ms < (1000U / g.temperature_msg_rate)) {
        return;
    }
    temperature_last_send_ms = now_ms;

    {
        const uint8_t num_sensors = temperature_sensor.num_instances();
        for (uint8_t i = 0; i < num_sensors; i++) {
            // Send each sensor in turn
            const uint8_t index = (temperature_last_sent_index + 1 + i) % num_sensors;

            float temp_deg = 0.0;
            if ((temperature_sensor.get_source(index) != AP_TemperatureSensor_Params::Source::DroneCAN) ||
                !temperature_sensor.get_temperature(temp_deg, index)) {
                // not configured to send or Unhealthy
                continue;
            }

            uavcan_equipment_device_Temperature pkt {};
            pkt.temperature = C_TO_KELVIN(temp_deg);

            // Use source ID from temperature lib
            pkt.device_id = temperature_sensor.get_source_id(index);

            uint8_t buffer[UAVCAN_EQUIPMENT_DEVICE_TEMPERATURE_MAX_SIZE];
            const uint16_t total_size = uavcan_equipment_device_Temperature_encode(&pkt, buffer, !canfdout());

            canard_broadcast(UAVCAN_EQUIPMENT_DEVICE_TEMPERATURE_SIGNATURE,
                             UAVCAN_EQUIPMENT_DEVICE_TEMPERATURE_ID,
                             CANARD_TRANSFER_PRIORITY_LOW,
                             &buffer[0],
                             total_size);

            temperature_last_sent_index = index;
            break;
        }
    }
}

#endif // AP_PERIPH_DEVICE_TEMPERATURE_ENABLED
