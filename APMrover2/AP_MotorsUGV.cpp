/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>
#include "SRV_Channel/SRV_Channel.h"
#include "AP_MotorsUGV.h"
#include "Rover.h"

extern const AP_HAL::HAL& hal;

// parameters for the motor class
const AP_Param::GroupInfo AP_MotorsUGV::var_info[] = {
    // @Param: PWM_TYPE
    // @DisplayName: Output PWM type
    // @Description: This selects the output PWM type as regular PWM, OneShot, Brushed motor support using PWM (duty cycle) with separated direction signal, Brushed motor support with separate throttle and direction PWM (duty cyle)
    // @Values: 0:Normal,1:OneShot,2:OneShot125,3:Brushed,4:BrushedBiPolar
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("PWM_TYPE", 1, AP_MotorsUGV, _pwm_type, PWM_TYPE_NORMAL),

    // @Param: PWM_FREQ
    // @DisplayName: Output PWM freq for brushed motors
    // @Description: Output PWM freq for brushed motors
    // @Units: kHz
    // @Range: 1 20
    // @Increment: 1
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("PWM_FREQ", 2, AP_MotorsUGV, _pwm_freq, 16),

    // @Param: SAFE_DISARM
    // @DisplayName: Motor PWM output disabled when disarmed
    // @Description: Disables motor PWM output when disarmed
    // @Values: 0:PWM enabled while disarmed, 1:PWM disabled while disarmed
    // @User: Advanced
    AP_GROUPINFO("SAFE_DISARM", 3, AP_MotorsUGV, _disarm_disable_pwm, 0),

    // @Param: THR_SLEWRATE
    // @DisplayName: Throttle slew rate
    // @Description: maximum percentage change in throttle per second. A setting of 10 means to not change the throttle by more than 10% of the full throttle range in one second. A value of zero means no limit. A value of 100 means the throttle can change over its full range in one second. Note that for some NiMH powered rovers setting a lower value like 40 or 50 may be worthwhile as the sudden current demand on the battery of a big rise in throttle may cause a brownout.
    // @Units: %/s
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SLEWRATE", 4, AP_MotorsUGV, _slew_rate, 100),

    // @Param: THR_MIN
    // @DisplayName: Throttle minimum
    // @Description: Throttle minimum percentage the autopilot will apply. This is mostly useful for rovers with internal combustion motors, to prevent the motor from cutting out in auto mode.
    // @Units: %
    // @Range: 0 20
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("THR_MIN", 5, AP_MotorsUGV, _throttle_min, 0),

    // @Param: THR_MAX
    // @DisplayName: Throttle maximum
    // @Description: Throttle maximum percentage the autopilot will apply. This can be used to prevent overheating an ESC or motor on an electric rover
    // @Units: %
    // @Range: 30 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("THR_MAX", 6, AP_MotorsUGV, _throttle_max, 100),

    AP_GROUPEND
};

AP_MotorsUGV::AP_MotorsUGV(AP_ServoRelayEvents &relayEvents) :
        _relayEvents(relayEvents)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_MotorsUGV::init()
{
    // setup servo ouput
    setup_servo_output();

    // setup pwm type
    setup_pwm_type();

    // set safety output
    setup_safety_output();
}

// setup output in case of main CPU failure
void AP_MotorsUGV::setup_safety_output()
{
    if (_pwm_type == PWM_TYPE_BRUSHED) {
        // set trim to min to set duty cycle range (0 - 100%) to servo range
        SRV_Channels::set_trim_to_min_for(SRV_Channel::k_throttleLeft);
        SRV_Channels::set_trim_to_min_for(SRV_Channel::k_throttleRight);
    }

    if (_disarm_disable_pwm) {
        // throttle channels output zero pwm (i.e. no signal)
        SRV_Channels::set_safety_limit(SRV_Channel::k_throttle, SRV_Channel::SRV_CHANNEL_LIMIT_ZERO_PWM);
        SRV_Channels::set_safety_limit(SRV_Channel::k_throttleLeft, SRV_Channel::SRV_CHANNEL_LIMIT_ZERO_PWM);
        SRV_Channels::set_safety_limit(SRV_Channel::k_throttleRight, SRV_Channel::SRV_CHANNEL_LIMIT_ZERO_PWM);
    } else {
        // throttle channels output trim values (because rovers will go backwards if set to MIN)
        SRV_Channels::set_safety_limit(SRV_Channel::k_throttle, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
        SRV_Channels::set_safety_limit(SRV_Channel::k_throttleLeft, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
        SRV_Channels::set_safety_limit(SRV_Channel::k_throttleRight, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
    }

    // stop sending pwm if main CPU fails
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_throttle, SRV_Channel::SRV_CHANNEL_LIMIT_ZERO_PWM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_throttleLeft, SRV_Channel::SRV_CHANNEL_LIMIT_ZERO_PWM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_throttleRight, SRV_Channel::SRV_CHANNEL_LIMIT_ZERO_PWM);
}

// setup servo output ranges
void AP_MotorsUGV::setup_servo_output()
{
    // k_steering are limited to -45;45 degree
    SRV_Channels::set_angle(SRV_Channel::k_steering, SERVO_MAX);

    // k_throttle are in power percent so -100 ... 100
    SRV_Channels::set_angle(SRV_Channel::k_throttle, 100);

    // skid steering left/right throttle as -1000 to 1000 values
    SRV_Channels::set_angle(SRV_Channel::k_throttleLeft,  1000);
    SRV_Channels::set_angle(SRV_Channel::k_throttleRight, 1000);
}

// set steering as a value from -4500 to +4500
void AP_MotorsUGV::set_steering(float steering)
{
    _steering = constrain_float(steering, -4500.0f, 4500.0f);
}

// set throttle as a value from -100 to 100
void AP_MotorsUGV::set_throttle(float throttle)
{
    // sanity check throttle min and max
    _throttle_min = constrain_int16(_throttle_min, 0, 20);
    _throttle_max = constrain_int16(_throttle_max, 30, 100);

    // check throttle is between -_throttle_max ~ +_throttle_max but outside -throttle_min ~ +throttle_min
    _throttle = constrain_float(throttle, -_throttle_max, _throttle_max);
    if ((_throttle_min > 0) && (fabsf(_throttle) < _throttle_min)) {
        if (is_negative(_throttle)) {
            _throttle = -_throttle_min;
        } else {
            _throttle = _throttle_min;
        }
    }
}

/*
  work out if skid steering is available
 */
bool AP_MotorsUGV::have_skid_steering() const
{
    if (SRV_Channels::function_assigned(SRV_Channel::k_throttleLeft) &&
        SRV_Channels::function_assigned(SRV_Channel::k_throttleRight)) {
        return true;
    }
    return false;
}

void AP_MotorsUGV::output(bool armed, float dt)
{
    // soft-armed overrides passed in armed status
    if (!hal.util->get_soft_armed()) {
        armed = false;
    }

    slew_limit_throttle(dt);

    // clear and set limits based on input (limit flags may be set again by output_regular or output_skid_steering methods)
    set_limits_from_input(armed, _steering, _throttle);

    // output for regular steering/throttle style frames
    output_regular(armed, _steering, _throttle);

    // output for skid steering style frames
    output_skid_steering(armed, _steering, _throttle);

    // send values to the PWM timers for output
    SRV_Channels::calc_pwm();
    hal.rcout->cork();
    SRV_Channels::output_ch_all();
    hal.rcout->push();
    _last_throttle = _throttle;
}

// output to regular steering and throttle channels
void AP_MotorsUGV::output_regular(bool armed, float steering, float throttle)
{
    // always allow steering to move
    SRV_Channels::set_output_scaled(SRV_Channel::k_steering, steering);

    // output to throttle channels
    if (armed) {
        // handle armed case
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle);
    } else {
        // handle disarmed case
        if (_disarm_disable_pwm) {
            SRV_Channels::set_output_limit(SRV_Channel::k_throttle, SRV_Channel::SRV_CHANNEL_LIMIT_ZERO_PWM);
        } else {
            SRV_Channels::set_output_limit(SRV_Channel::k_throttle, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
        }
    }
}

// output to skid steering channels
void AP_MotorsUGV::output_skid_steering(bool armed, float steering, float throttle)
{
    if (!have_skid_steering()) {
        return;
    }

    // handle simpler disarmed case
    if (!armed) {
        if (_disarm_disable_pwm) {
            SRV_Channels::set_output_limit(SRV_Channel::k_throttleLeft, SRV_Channel::SRV_CHANNEL_LIMIT_ZERO_PWM);
            SRV_Channels::set_output_limit(SRV_Channel::k_throttleRight, SRV_Channel::SRV_CHANNEL_LIMIT_ZERO_PWM);
        } else {
            SRV_Channels::set_output_limit(SRV_Channel::k_throttleLeft, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
            SRV_Channels::set_output_limit(SRV_Channel::k_throttleRight, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
        }
        return;
    }

    // skid steering mixer
    float steering_scaled = steering / 4500.0f; // steering scaled -1 to +1
    float throttle_scaled = throttle / 100.0f;  // throttle scaled -1 to +1

    // apply constraints
    steering_scaled = constrain_float(steering_scaled, -1.0f, 1.0f);
    throttle_scaled = constrain_float(throttle_scaled, -1.0f, 1.0f);

    // check for saturation and scale back throttle and steering proportionally
    const float saturation_value = fabsf(steering_scaled) + fabsf(throttle_scaled);
    if (saturation_value > 1.0f) {
        steering_scaled = steering_scaled / saturation_value;
        throttle_scaled = throttle_scaled / saturation_value;
        // set limits
        if (is_negative(steering)) {
            limit.steer_left = true;
        } else {
            limit.steer_right = true;
        }
        if (is_negative(throttle)) {
            limit.throttle_lower = true;
        } else {
            limit.throttle_upper = true;
        }
    }

    // add in throttle
    float motor_left = throttle_scaled;
    float motor_right = throttle_scaled;

    // deal with case of turning on the spot
    if (is_zero(throttle_scaled)) {
        // full possible range is not used to keep response equivalent to non-zero throttle case
        motor_left += steering_scaled * 0.5f;
        motor_right -= steering_scaled * 0.5f;
    } else {
        // add in steering
        const float dir = is_positive(throttle_scaled) ? 1.0f : -1.0f;
        if (is_negative(steering_scaled)) {
            // moving left all steering to right wheel
            motor_right -= dir * steering_scaled;
        } else {
            // turning right, all steering to left wheel
            motor_left += dir * steering_scaled;
        }
    }

    // send pwm value to each motor
    output_throttle(SRV_Channel::k_throttleLeft, 100.0f * motor_left);
    output_throttle(SRV_Channel::k_throttleRight, 100.0f * motor_right);
}

// output throttle value to main throttle channel, left throttle or right throttle.  throttle should be scaled from -100 to 100
void AP_MotorsUGV::output_throttle(SRV_Channel::Aux_servo_function_t function, float throttle)
{
    // sanity check servo function
    if (function != SRV_Channel::k_throttle && function != SRV_Channel::k_throttleLeft && function != SRV_Channel::k_throttleRight) {
        return;
    }

    // constrain output
    throttle = constrain_float(throttle, -100.0f, 100.0f);

    // set relay if necessary
    if (_pwm_type == PWM_TYPE_BRUSHED) {
        // find the output channel, if not found return
        const SRV_Channel *out_chan = SRV_Channels::get_channel_for(function);
        if (out_chan == nullptr) {
            return;
        }
        bool relay_high = out_chan->get_reversed() ? !is_negative(throttle) : is_negative(throttle);
        switch (function) {
            case SRV_Channel::k_throttle:
            case SRV_Channel::k_throttleLeft:
                _relayEvents.do_set_relay(0, relay_high);
                break;
            case SRV_Channel::k_throttleRight:
                _relayEvents.do_set_relay(1, relay_high);
                break;
            default:
                // do nothing
                break;
        }
        // brushed-with-relay motors should receive positive values
        throttle = fabsf(throttle);
    }

    // output to servo channel
    switch (function) {
        case SRV_Channel::k_throttle:
            SRV_Channels::set_output_scaled(function,  throttle);
            break;
        case SRV_Channel::k_throttleLeft:
        case SRV_Channel::k_throttleRight:
            SRV_Channels::set_output_scaled(function,  throttle*10.0f);
            break;
        default:
            // do nothing
            break;
    }
}

// slew limit throttle for one iteration
void AP_MotorsUGV::slew_limit_throttle(float dt)
{
    if (_use_slew_rate && (_slew_rate > 0)) {
        float temp = _slew_rate * dt * 0.01f * 100.0f;  // TODO : get THROTTLE MIN and THROTTLE MAX
        if (temp < 1.0f) {
            temp = 1.0f;
        }
        _throttle = constrain_int16(_throttle, _last_throttle - temp, _last_throttle + temp);
    }
}

// set limits based on steering and throttle input
void AP_MotorsUGV::set_limits_from_input(bool armed, float steering, float throttle)
{
    // set limits based on inputs
    limit.steer_left = !armed || (steering <= -4500.0f);
    limit.steer_right = !armed || (steering >= 4500.0f);
    limit.throttle_lower = !armed || (throttle <= -_throttle_max);
    limit.throttle_upper = !armed || (throttle >= _throttle_max);
}

// setup pwm output type
void AP_MotorsUGV::setup_pwm_type()
{
    switch (_pwm_type) {
    case PWM_TYPE_ONESHOT:
    case PWM_TYPE_ONESHOT125:
        // tell HAL to do immediate output
        hal.rcout->set_output_mode(AP_HAL::RCOutput::MODE_PWM_ONESHOT);
        break;
    case PWM_TYPE_BRUSHED:
    case PWM_TYPE_BRUSHEDBIPOLAR:
        hal.rcout->set_output_mode(AP_HAL::RCOutput::MODE_PWM_BRUSHED);
        /*
         * Group 0: channels 0 1
         * Group 1: channels 4 5 6 7
         * Group 2: channels 2 3
         */
        // TODO : See if we can seperate frequency between groups
        hal.rcout->set_freq((1UL << 0), static_cast<uint16_t>(_pwm_freq * 1000));  // Steering group
        hal.rcout->set_freq((1UL << 2), static_cast<uint16_t>(_pwm_freq * 1000));  // Throttle group
        break;
    default:
        // do nothing
        break;
    }
}

// test steering or throttle output as a percentage of the total (range -100 to +100)
// used in response to DO_MOTOR_TEST mavlink command
bool AP_MotorsUGV::output_test_pct(motor_test_order motor_seq, float pct)
{
    // check if the motor_seq is valid
    if (motor_seq > MOTOR_TEST_THROTTLE_RIGHT) {
        return false;
    }
    pct = constrain_float(pct, -100.0f, 100.0f);

    switch (motor_seq) {
        case MOTOR_TEST_THROTTLE: {
            if (!SRV_Channels::function_assigned(SRV_Channel::k_throttle)) {
                return false;
            }
            output_throttle(SRV_Channel::k_throttle, pct);
            break;
        }
        case MOTOR_TEST_STEERING: {
            if (!SRV_Channels::function_assigned(SRV_Channel::k_steering)) {
                return false;
            }
            SRV_Channels::set_output_scaled(SRV_Channel::k_steering, pct * 45.0f);
            break;
        }
        case MOTOR_TEST_THROTTLE_LEFT: {
            if (!SRV_Channels::function_assigned(SRV_Channel::k_throttleLeft)) {
                return false;
            }
            output_throttle(SRV_Channel::k_throttleLeft, pct);
            break;
        }
        case MOTOR_TEST_THROTTLE_RIGHT: {
            if (!SRV_Channels::function_assigned(SRV_Channel::k_throttleRight)) {
                return false;
            }
            output_throttle(SRV_Channel::k_throttleRight, pct);
            break;
        }
        default:
            return false;
    }
    SRV_Channels::calc_pwm();
    hal.rcout->cork();
    SRV_Channels::output_ch_all();
    hal.rcout->push();
    return true;
}

// test steering or throttle output using a pwm value
bool AP_MotorsUGV::output_test_pwm(motor_test_order motor_seq, float pwm)
{
    // check if the motor_seq is valid
    if (motor_seq > MOTOR_TEST_THROTTLE_RIGHT) {
        return false;
    }
    switch (motor_seq) {
        case MOTOR_TEST_THROTTLE: {
            if (!SRV_Channels::function_assigned(SRV_Channel::k_throttle)) {
                return false;
            }
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttle, pwm);
            break;
        }
        case MOTOR_TEST_STEERING: {
            if (!SRV_Channels::function_assigned(SRV_Channel::k_steering)) {
                return false;
            }
            SRV_Channels::set_output_pwm(SRV_Channel::k_steering, pwm);
            break;
        }
        case MOTOR_TEST_THROTTLE_LEFT: {
            if (!SRV_Channels::function_assigned(SRV_Channel::k_throttleLeft)) {
                return false;
            }
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft, pwm);
            break;
        }
        case MOTOR_TEST_THROTTLE_RIGHT: {
            if (!SRV_Channels::function_assigned(SRV_Channel::k_throttleRight)) {
                return false;
            }
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, pwm);
            break;
        }
        default:
            return false;
    }
    SRV_Channels::calc_pwm();
    hal.rcout->cork();
    SRV_Channels::output_ch_all();
    hal.rcout->push();
    return true;
}
