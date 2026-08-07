#include "AP_Periph.h"

#if AP_PERIPH_PWM_HARDPOINT_ENABLED

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL &hal;

/*
  hardpoint support
 */

#include <dronecan_msgs.h>

void AP_Periph_FW::pwm_hardpoint_init()
{
    hal.gpio->attach_interrupt(
        PWM_HARDPOINT_PIN,
        FUNCTOR_BIND_MEMBER(&AP_Periph_FW::pwm_irq_handler, void, uint8_t, bool, uint32_t), AP_HAL::GPIO::INTERRUPT_BOTH);
}

/*
  called on PWM pin transition
 */
void AP_Periph_FW::pwm_irq_handler(uint8_t pin, bool pin_state, uint32_t timestamp)
{
    if (pin_state == 0 && pwm_hardpoint.last_state == 1 && pwm_hardpoint.last_ts_us != 0) {
        uint32_t width = timestamp - pwm_hardpoint.last_ts_us;
        if (width > 500 && width < 2500) {
            pwm_hardpoint.pwm_value = width;
            if (width > pwm_hardpoint.highest_pwm) {
                pwm_hardpoint.highest_pwm = width;
            }
        }
    }
    pwm_hardpoint.last_state = pin_state;
    pwm_hardpoint.last_ts_us = timestamp;
}

void AP_Periph_FW::pwm_hardpoint_update()
{
    uint32_t now = AP_HAL::millis();
    // send at 10Hz
    void *save = hal.scheduler->disable_interrupts_save();
    uint16_t value = pwm_hardpoint.highest_pwm;
    pwm_hardpoint.highest_pwm = 0;
    hal.scheduler->restore_interrupts(save);
    float rate = g.hardpoint_rate;
    rate = constrain_float(rate, 10, 100);
    if (value > 0 && now - pwm_hardpoint.last_send_ms >= 1000U/rate) {
        pwm_hardpoint.last_send_ms = now;
        uavcan_equipment_hardpoint_Command cmd {};
        cmd.hardpoint_id = g.hardpoint_id;
        cmd.command = value;

        uint8_t buffer[UAVCAN_EQUIPMENT_HARDPOINT_COMMAND_MAX_SIZE];
        uint16_t total_size = uavcan_equipment_hardpoint_Command_encode(&cmd, buffer, !canfdout());
        canard_broadcast(UAVCAN_EQUIPMENT_HARDPOINT_COMMAND_SIGNATURE,
                        UAVCAN_EQUIPMENT_HARDPOINT_COMMAND_ID,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        &buffer[0],
                        total_size);
    }
}

#endif // AP_PERIPH_PWM_HARDPOINT_ENABLED
