#include "Copter.h"

// Run standby functions at approximately 100 Hz to limit maximum variable build up
//
// When standby is active:
//      all I terms are continually reset
//      heading error is reset to zero
//      position errors are reset to zero
//      crash_check is disabled
//      thrust_loss_check is disabled
//      parachute_check is disabled
//      and landing detection is disabled.
void Copter::standby_update()
{
    if(g2.standby_pin > -1) {
        standby_pin_active();
    }
    if (standby_last != standby_active) {
        if (standby_active) {
            AP::logger().Write_Event(LogEvent::STANDBY_ENABLE);
            gcs().send_text(MAV_SEVERITY_INFO, "Stand By Enabled");
        } else {
            AP::logger().Write_Event(LogEvent::STANDBY_DISABLE);
            gcs().send_text(MAV_SEVERITY_INFO, "Stand By Disabled");
        }
    }
    standby_last = standby_active;

    if (!standby_active) {
        return;
    }

    attitude_control->reset_rate_controller_I_terms();
    attitude_control->set_yaw_target_to_current_heading();
    pos_control->standby_xyz_reset();
}

//
// standby_enable - enable standby
//
void Copter::standby_enable()
{
    hal.gpio->pinMode(g2.standby_pin, HAL_GPIO_INPUT); // ensure we are in input mode
    hal.gpio->write(g2.standby_pin, 1);                // enable pullup
}

//
// check if standby pin is high
//
void Copter::standby_pin_active(void)
{
    uint8_t pin_state = hal.gpio->read(g2.standby_pin);
    uint8_t trigger_polarity = 1;
    if (pin_state == trigger_polarity) {
        standby_active = true;
    } else {
        standby_active = false;
    }
}
