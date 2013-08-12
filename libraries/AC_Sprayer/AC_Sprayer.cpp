// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>
#include <AC_Sprayer.h>

extern const AP_HAL::HAL& hal;

// ------------------------------

const AP_Param::GroupInfo AC_Sprayer::var_info[] PROGMEM = {
    // @Param: ENABLE
    // @DisplayName: Sprayer enable/disable
    // @Description: Allows you to enable (1) or disable (0) the sprayer
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("ENABLE",      0,  AC_Sprayer, _enabled, 0),

    // @Param: PUMP_RATE
    // @DisplayName: Pump speed
    // @Description: Desired pump speed when travelling 1m/s expressed as a percentage
    // @Units: percentage
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("PUMP_RATE",   1, AC_Sprayer, _pump_pct_1ms, AC_SPRAYER_DEFAULT_PUMP_RATE),

    // @Param: SPINNER
    // @DisplayName: Spinner rotation speed
    // @Description: Spinner's rotation speed in PWM (a higher rate will disperse the spray over a wider area horizontally)
    // @Units: ms
    // @Range: 1000 2000
    // @User: Standard
    AP_GROUPINFO("SPINNER",     2, AC_Sprayer, _spinner_pwm, AC_SPRAYER_DEFAULT_SPINNER_PWM),

    // @Param: SPEED_MIN
    // @DisplayName: Speed minimum
    // @Description: Speed minimum at which we will begin spraying
    // @Units: cm/s
    // @Range: 0 1000
    // @User: Standard
    AP_GROUPINFO("SPEED_MIN",   3, AC_Sprayer, _speed_min, AC_SPRAYER_DEFAULT_SPEED_MIN),

    // @Param: PUMP_MIN
    // @DisplayName: Pump speed minimum
    // @Description: Minimum pump speed expressed as a percentage
    // @Units: percentage
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("PUMP_MIN",   4, AC_Sprayer, _pump_min_pct, AC_SPRAYER_DEFAULT_PUMP_MIN),

    AP_GROUPEND
};

AC_Sprayer::AC_Sprayer(const AP_InertialNav* inav) :
    _inav(inav),
    _speed_over_min_time(0),
    _speed_under_min_time(0)
{
    AP_Param::setup_object_defaults(this, var_info);

    // check for silly parameter values
    if (_pump_pct_1ms < 0 || _pump_pct_1ms > 100) {
        _pump_pct_1ms.set_and_save(AC_SPRAYER_DEFAULT_PUMP_RATE);
    }
    if (_spinner_pwm < 0) {
        _spinner_pwm.set_and_save(AC_SPRAYER_DEFAULT_SPINNER_PWM);
    }

    // To-Do: ensure that the pump and spinner servo channels are enabled
}

void AC_Sprayer::enable(bool true_false)
{
    // return immediately if no change
    if (true_false == _enabled) {
        return;
    }

    // set enabled/disabled parameter (in memory only)
    _enabled = true_false;

    // turn off the pump and spinner servos if necessary
    if (!_enabled) {
        // send output to pump channel
        // To-Do: change 0 below to radio_min of pump servo
        RC_Channel_aux::set_radio_to_min(RC_Channel_aux::k_sprayer_pump);

        // send output to spinner channel
        // To-Do: change 0 below to radio_min of spinner servo
        RC_Channel_aux::set_radio_to_min(RC_Channel_aux::k_sprayer_spinner);
    }
}

/// update - adjust pwm of servo controlling pump speed according to the desired quantity and our horizontal speed
void
AC_Sprayer::update()
{
    uint32_t now;
    float ground_speed;

    // exit immediately if we are disabled (perhaps set pwm values back to defaults)
    if (!_enabled) {
        return;
    }

    // exit immediately if the pump function has not been set-up for any servo
    if (!RC_Channel_aux::function_assigned(RC_Channel_aux::k_sprayer_pump)) {
        return;
    }

    // get horizontal velocity
    const Vector3f &velocity = _inav->get_velocity();
    ground_speed = pythagorous2(velocity.x,velocity.y);

    // get the current time
    now = hal.scheduler->millis();

    // check our speed vs the minimum
    if (ground_speed >= _speed_min) {
        // if we are not already spraying
        if (!_flags.spraying) {
            // set the timer if this is the first time we've surpassed the min speed
            if (_speed_over_min_time == 0) {
                _speed_over_min_time = now;
            }else{
                // check if we've been over the speed long enough to engage the sprayer
                if((now - _speed_over_min_time) > AC_SPRAYER_DEFAULT_TURN_ON_DELAY) {
                    _flags.spraying = true;
                    _speed_over_min_time = 0;
                }
            }
        }
        // reset the speed under timer
        _speed_under_min_time = 0;
    }else{
        // we are under the min speed.  If we are spraying
        if (_flags.spraying) {
            // set the timer if this is the first time we've dropped below the min speed
            if (_speed_under_min_time == 0) {
                _speed_under_min_time = now;
            }else{
                // check if we've been over the speed long enough to engage the sprayer
                if((now - _speed_under_min_time) > AC_SPRAYER_DEFAULT_SHUT_OFF_DELAY) {
                    _flags.spraying = false;
                    _speed_under_min_time = 0;
                }
            }
        }
        // reset the speed over timer
        _speed_over_min_time = 0;
    }

    // if testing pump output speed as if travelling at 1m/s
    if (_flags.testing) {
        ground_speed = 100;
    }

    // if spraying or testing update the pump servo position
    if (_flags.spraying || _flags.testing) {        
        RC_Channel_aux::move_servo(RC_Channel_aux::k_sprayer_pump, min(max(ground_speed * _pump_pct_1ms, 100 *_pump_min_pct),10000),0,10000);
        RC_Channel_aux::set_radio(RC_Channel_aux::k_sprayer_spinner, _spinner_pwm);
    }else{
        // ensure sprayer and spinner are off
        RC_Channel_aux::set_radio_to_min(RC_Channel_aux::k_sprayer_pump);
        RC_Channel_aux::set_radio_to_min(RC_Channel_aux::k_sprayer_spinner);
    }
}
