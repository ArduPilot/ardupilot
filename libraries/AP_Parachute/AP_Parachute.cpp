// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_Parachute.h>
#include <AP_Relay.h>
#include <AP_Math.h>
#include <RC_Channel.h>
#include <AP_HAL.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Parachute::var_info[] PROGMEM = {

    // @Param: ENABLED
    // @DisplayName: Parachute release enabled or disabled
    // @Description: Parachute release enabled or disabled
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("ENABLED", 0, AP_Parachute, _enabled, 0),

    // @Param: TYPE
    // @DisplayName: Parachute release mechanism type (relay or servo)
    // @Description: Parachute release mechanism type (relay or servo)
    // @Values: 0:Relay 0,1:Relay 1,2:Relay 2,3:Relay 3,10:Servo
    // @User: Standard
    AP_GROUPINFO("TYPE", 1, AP_Parachute, _release_type, AP_PARACHUTE_TRIGGER_TYPE_RELAY_0),

    // @Param: SERVO_ON
    // @DisplayName: Parachute Servo ON PWM value
    // @Description: Parachute Servo PWM value when parachute is released
    // @Range: 1000 2000
    // @Units: pwm
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SERVO_ON", 2, AP_Parachute, _servo_on_pwm, AP_PARACHUTE_SERVO_ON_PWM_DEFAULT),

    // @Param: SERVO_OFF
    // @DisplayName: Servo OFF PWM value
    // @Description: Parachute Servo PWM value when parachute is not released
    // @Range: 1000 2000
    // @Units: pwm
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SERVO_OFF", 3, AP_Parachute, _servo_off_pwm, AP_PARACHUTE_SERVO_OFF_PWM_DEFAULT),

    // @Param: ALT_MIN
    // @DisplayName: Parachute min altitude in cm above home
    // @Description: Parachute min altitude above home.  When vehicle is below this alt parachute cannot be released
    // @Range: 0 10000
    // @Units: Centimeters
    // @Increment: 100
    // @User: Standard
    AP_GROUPINFO("ALT_MIN", 4, AP_Parachute, _alt_min_cm, AP_PARACHUTE_ALT_MIN_DEFAULT),

    AP_GROUPEND
};


/// release - release parachute
void AP_Parachute::release()
{
    // exit immediately if not enabled
    if (_enabled <= 0) {
        return;
    }

    // trigger the servo or relay
    if (_release_type == AP_PARACHUTE_TRIGGER_TYPE_SERVO) {
	    RC_Channel_aux::set_radio(RC_Channel_aux::k_parachute_release, _servo_on_pwm);
    }else if (_release_type <= AP_PARACHUTE_TRIGGER_TYPE_RELAY_3) {
        _relay.on(_release_type);
    }

	// leave a message that it should be active for this many loops (assumes 50hz loops)
    _release_time = hal.scheduler->millis();
}

/// update - shuts off the trigger should be called at about 10hz
void AP_Parachute::update()
{
    // exit immediately if not enabled or parachute not released
    if (_enabled <= 0 || _release_time == 0) {
        return;
    }

    // check if release mechanism has been triggered more than 1 second ago
    if (hal.scheduler->millis() - _release_time > AP_PARACHUTE_RELEASE_DURATION_MS) {
        // trigger the servo or relay
        if (_release_type == AP_PARACHUTE_TRIGGER_TYPE_SERVO) {
            // move servo back to off position
            RC_Channel_aux::set_radio(RC_Channel_aux::k_parachute_release, _servo_off_pwm);
        }else if (_release_type <= AP_PARACHUTE_TRIGGER_TYPE_RELAY_3) {
            // set relay back to zero volts
            _relay.on(_release_type);
        }

        // reset release_time
        _release_time = 0;
    }
}
