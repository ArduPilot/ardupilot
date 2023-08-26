#include "AP_Periph.h"

#ifdef HAL_PERIPH_ENABLE_BATTERY

/*
  battery support
 */

#include <dronecan_msgs.h>

extern const AP_HAL::HAL &hal;

#ifndef AP_PERIPH_BATTERY_MODEL_NAME
#define AP_PERIPH_BATTERY_MODEL_NAME CAN_APP_NODE_NAME
#endif

/*
  update CAN battery monitor
 */
void AP_Periph_FW::can_battery_update(void)
{
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - battery.last_can_send_ms < 100) {
        return;
    }
    battery.last_can_send_ms = now_ms;

    const uint8_t battery_instances = battery_lib.num_instances();
    for (uint8_t i=0; i<battery_instances; i++) {
        if (!battery_lib.healthy(i)) {
            continue;
        }

        uavcan_equipment_power_BatteryInfo pkt {};

        // if a battery serial number is assigned, use that as the ID. Else, use the index.
        const int32_t serial_number = battery_lib.get_serial_number(i);
        pkt.battery_id = (serial_number >= 0) ? serial_number : i+1;

        pkt.voltage = battery_lib.voltage(i);

        float current;
        if (battery_lib.current_amps(current, i)) {
            pkt.current = current;
        }
        float temperature;
        if (battery_lib.get_temperature(temperature, i)) {
            // Battery lib reports temperature in Celsius.
            // Convert Celsius to Kelvin for transmission on CAN.
            pkt.temperature = C_TO_KELVIN(temperature);
        }

        pkt.state_of_health_pct = UAVCAN_EQUIPMENT_POWER_BATTERYINFO_STATE_OF_HEALTH_UNKNOWN;
        uint8_t percentage = 0;
        if (battery_lib.capacity_remaining_pct(percentage, i)) {
            pkt.state_of_charge_pct = percentage;
        }
        pkt.model_instance_id = i+1;

#if !defined(HAL_PERIPH_BATTERY_SKIP_NAME)
        // example model_name: "org.ardupilot.ap_periph SN 123"
        hal.util->snprintf((char*)pkt.model_name.data, sizeof(pkt.model_name.data), "%s %ld", AP_PERIPH_BATTERY_MODEL_NAME, (long int)serial_number);
        pkt.model_name.len = strnlen((char*)pkt.model_name.data, sizeof(pkt.model_name.data));
#endif //defined(HAL_PERIPH_BATTERY_SKIP_NAME)

        uint8_t buffer[UAVCAN_EQUIPMENT_POWER_BATTERYINFO_MAX_SIZE] {};
        const uint16_t total_size = uavcan_equipment_power_BatteryInfo_encode(&pkt, buffer, !periph.canfdout());

        canard_broadcast(UAVCAN_EQUIPMENT_POWER_BATTERYINFO_SIGNATURE,
                        UAVCAN_EQUIPMENT_POWER_BATTERYINFO_ID,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        &buffer[0],
                        total_size);
    }
}

#endif // HAL_PERIPH_ENABLE_BATTERY

