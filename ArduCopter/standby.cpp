#include "Copter.h"

const AP_Param::GroupInfo Standby::var_info[] = {
    // @Param: PIN
    // @DisplayName: StandBy pin
    // @Description: pin number to used to enable Stand By state. If set to -1 then it will not be used.
    // @Values: -1:Disabled, 50:PX4 AUX1, 51:PX4 AUX2, 52:PX4 AUX3, 53:PX4 AUX4, 54:PX4 AUX5, 55:PX4 AUX6
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("PIN",  1, Standby, _pin, -1),

    AP_GROUPEND
};

Standby::Standby()
{
    AP_Param::setup_object_defaults(this, var_info);
}


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
void Standby::update()
{
    if (_pin > -1) {
        // a configured pin always takes precedence over an RC channel option
        uint8_t pin_state = hal.gpio->read(_pin);
        uint8_t trigger_polarity = 1;
        if (pin_state == trigger_polarity) {
            _active = true;
        } else {
            _active = false;
        }
    } else {
        switch (_switch_pos) {
        case RC_Channel::AuxSwitchPos::HIGH:
            _active = true;
            break;
        case RC_Channel::AuxSwitchPos::MIDDLE:
            // no change
            break;
        case RC_Channel::AuxSwitchPos::LOW:
            _active = false;
            break;
        }
    }

    if (_last_active != _active) {
        if (_active) {
            AP::logger().Write_Event(LogEvent::STANDBY_ENABLE);
            copter.gcs().send_text(MAV_SEVERITY_INFO, "Stand By Enabled");
        } else {
            AP::logger().Write_Event(LogEvent::STANDBY_DISABLE);
            copter.gcs().send_text(MAV_SEVERITY_INFO, "Stand By Disabled");
        }
    }
    _last_active = _active;

    if (!_active) {
        return;
    }

    copter.attitude_control->reset_rate_controller_I_terms();
    copter.attitude_control->set_yaw_target_to_current_heading();
    copter.pos_control->standby_xyz_reset();
}

//
// standby_enable - enable standby
//
void Standby::init()
{
    hal.gpio->pinMode(_pin, HAL_GPIO_INPUT); // ensure we are in input mode
    hal.gpio->write(_pin, 1);                // enable pullup
}
