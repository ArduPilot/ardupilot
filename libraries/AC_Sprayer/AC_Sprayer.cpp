#include "AC_Sprayer.h"

#if HAL_SPRAYER_ENABLED

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL& hal;

// ------------------------------

const AP_Param::GroupInfo AC_Sprayer::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Sprayer enable/disable
    // @Description: Allows you to enable (1) or disable (0) the sprayer
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 0, AC_Sprayer, _enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: PUMP_RATE
    // @DisplayName: Pump speed
    // @Description: Desired pump speed when traveling 1m/s expressed as a percentage
    // @Units: %
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
    // @Units: %
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("PUMP_MIN",   4, AC_Sprayer, _pump_min_pct, AC_SPRAYER_DEFAULT_PUMP_MIN),

    AP_GROUPEND
};

AC_Sprayer::AC_Sprayer()
{
    if (_singleton) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Too many sprayers");
#endif
        return;
    }
    _singleton = this;

    AP_Param::setup_object_defaults(this, var_info);

    // check for silly parameter values
    if (_pump_pct_1ms < 0.0f || _pump_pct_1ms > 100.0f) {
        _pump_pct_1ms.set_and_save(AC_SPRAYER_DEFAULT_PUMP_RATE);
    }
    if (_spinner_pwm < 0) {
        _spinner_pwm.set_and_save(AC_SPRAYER_DEFAULT_SPINNER_PWM);
    }

    // To-Do: ensure that the pump and spinner servo channels are enabled
}

/*
 * Get the AP_Sprayer singleton
 */
AC_Sprayer *AC_Sprayer::_singleton;
AC_Sprayer *AC_Sprayer::get_singleton()
{
    return _singleton;
}

void AC_Sprayer::run(const bool activate)
{
    // return immediately if no change
    if (_flags.running == activate) {
        return;
    }

    // set flag indicate whether spraying is permitted:
    // do not allow running to be set to true if we are currently not enabled
    _flags.running = _enabled && activate;

    // turn off the pump and spinner servos if necessary
    if (!_flags.running) {
        _flags.spraying = false;
        stop_spraying();
    }
}

void AC_Sprayer::stop_spraying()
{
    SRV_Channels::set_output_limit(SRV_Channel::k_sprayer_pump, SRV_Channel::Limit::MIN);
    SRV_Channels::set_output_limit(SRV_Channel::k_sprayer_spinner, SRV_Channel::Limit::MIN);
}

// update sprayer pwm
void AC_Sprayer::update_servos(const float ground_speed)
{
    if (!_flags.spraying) {
        stop_spraying();
        return;
    }

    // set pwm according to ground speed
    float pos = ground_speed * _pump_pct_1ms;
    pos = MAX(pos, 100 *_pump_min_pct); // ensure min pump speed
    pos = MIN(pos,10000); // clamp to range
    SRV_Channels::move_servo(SRV_Channel::k_sprayer_pump, pos, 0, 10000);
    SRV_Channels::set_output_pwm(SRV_Channel::k_sprayer_spinner, _spinner_pwm);
}

// check if the current state is as per ground speed
// toggle the state when the sprayer has spent enough time in inconsistent state
void AC_Sprayer::update_sprayer_state(const float ground_speed)
{
    const bool speed_over_min = ground_speed >= _speed_min;
    // return early if we are already in required state
    if (speed_over_min == _flags.spraying) {
        _inconsistent_state_start_time = 0;
        return;
    }

    const uint32_t now = AP_HAL::millis();
    // the sprayer state is inconsistent with ground speed
    // start the timer if we have just entered inconsistent state
    if (_inconsistent_state_start_time == 0) {
        _inconsistent_state_start_time = now;
        return;
    }

    // set delay as per new state
    const uint32_t delay = _flags.spraying ? AC_SPRAYER_DEFAULT_SHUT_OFF_DELAY : AC_SPRAYER_DEFAULT_TURN_ON_DELAY;

    // return if we have not been in inconsistent state for long enough
    if ((now - _inconsistent_state_start_time) < delay) {
        return;
    }

    // toggle sprayer state
    _flags.spraying = !_flags.spraying;
}

/// update - adjust pwm of servo controlling pump speed according to the desired quantity and our horizontal speed
void AC_Sprayer::update()
{
    // exit immediately if we are disabled or shouldn't be running
    if (!_enabled || !running()) {
        run(false);
        return;
    }

    // exit immediately if the pump function has not been set-up for any servo
    if (!SRV_Channels::function_assigned(SRV_Channel::k_sprayer_pump)) {
        return;
    }

    // get horizontal velocity
    Vector3f velocity;
    if (!AP::ahrs().get_velocity_NED(velocity)) {
        // treat unknown velocity as zero which should lead to pump stopping
        // velocity will already be zero but this avoids a coverity warning
        velocity.zero();
    }

    // set the groundspeed to 1 m/s if we are testing pump 
    // get the groundspeed value from velocity vector otherwise
    float ground_speed = _flags.testing ? 100.0 : velocity.xy().length() * 100.0;

    update_sprayer_state(ground_speed); // check and update sprayer state as per vehicle speed
    update_servos(ground_speed);        // update servos pwm as per vehicle speed
}

namespace AP {

AC_Sprayer *sprayer()
{
    return AC_Sprayer::get_singleton();
}

};
#endif // HAL_SPRAYER_ENABLED
