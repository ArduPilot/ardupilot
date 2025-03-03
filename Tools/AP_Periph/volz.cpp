#include "AP_Periph.h"

#if AP_PERIPH_VOLZ_ENABLED

#if AP_PERIPH_VOLZ_SEND_COM_VOLZ_SERVO_ACTUATORSTATUS_ENABLED
#include <AP_Servo_Telem/AP_Servo_Telem.h>
#include <dronecan_msgs.h>
#endif  // AP_PERIPH_VOLZ_SEND_COM_VOLZ_SERVO_ACTUATORSTATUS_ENABLED

#include <AP_HAL/HAL.h>

extern const AP_HAL::HAL& hal;

void AP_Periph_FW::volz_update()
{
    volz_protocol.update();

    volz_update_servo_telem();
}

void AP_Periph_FW::volz_update_servo_telem()
{
    AP_Servo_Telem *servo_telem = AP_Servo_Telem::get_singleton();
    if (servo_telem == nullptr) {
        return;
    }
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - volz.last_servo_telem_update_ms > 100) {
        servo_telem->update();
    }
    volz.last_servo_telem_update_ms = now_ms;

    // send a servo CAN message every 10ms
    if (now_ms - volz.last_actuator_status_send_ms < 10) {
        return;
    }
    volz.last_actuator_status_send_ms = now_ms;
    AP_Servo_Telem::TelemetryData telem_data;
    bool found = false;
    for (uint8_t first=volz.next_servo_to_send++; first != volz.next_servo_to_send; volz.next_servo_to_send++) {
        if (!servo_telem->get_telem(volz.next_servo_to_send, telem_data)) {
            continue;
        }
        if (telem_data.stale(now_ms)) {
            continue;
        }
        found = true;
        break;
    }
    if (!found) {
        return;
    }

#if AP_PERIPH_VOLZ_SEND_COM_VOLZ_SERVO_ACTUATORSTATUS_ENABLED
    send_com_volz_servo_ActuatorStatus(volz.next_servo_to_send, telem_data);
#endif
}

#if AP_PERIPH_VOLZ_SEND_COM_VOLZ_SERVO_ACTUATORSTATUS_ENABLED
void AP_Periph_FW::send_com_volz_servo_ActuatorStatus(uint8_t id, const AP_Servo_Telem::TelemetryData &telem_data)
{
    com_volz_servo_ActuatorStatus actuator_status {};
    actuator_status.actuator_id = id;
    actuator_status.actual_position = radians(telem_data.measured_position);
    actuator_status.current = MIN(telem_data.current / 0.025, 255);
    actuator_status.voltage = MIN(telem_data.voltage / 0.2, 255);
    actuator_status.motor_pwm = 0;  // actually power?
    actuator_status.motor_temperature = MIN(telem_data.motor_temperature_cdeg + 50, 255);

    uint8_t buffer[COM_VOLZ_SERVO_ACTUATORSTATUS_MAX_SIZE];
    const uint16_t total_size = com_volz_servo_ActuatorStatus_encode(&actuator_status, buffer, !canfdout());
    canard_broadcast(COM_VOLZ_SERVO_ACTUATORSTATUS_SIGNATURE,
                    COM_VOLZ_SERVO_ACTUATORSTATUS_ID,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    &buffer[0],
                    total_size);
}
#endif  // AP_PERIPH_VOLZ_SEND_COM_VOLZ_SERVO_ACTUATORSTATUS_ENABLED

#endif  // AP_PERIPH_VOLZ_ENABLED
