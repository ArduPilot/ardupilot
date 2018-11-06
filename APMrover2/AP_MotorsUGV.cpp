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
#include <AP_Math/AP_Math.h>
#include "SRV_Channel/SRV_Channel.h"
#include "AP_MotorsUGV.h"
#include "Rover.h"

extern const AP_HAL::HAL& hal;

// parameters for the motor class
const AP_Param::GroupInfo AP_MotorsUGV::var_info[] = {
    // @Param: PWM_TYPE
    // @DisplayName: Motor Output PWM type
    // @Description: This selects the output PWM type as regular PWM, OneShot, Brushed motor support using PWM (duty cycle) with separated direction signal, Brushed motor support with separate throttle and direction PWM (duty cyle)
    // @Values: 0:Normal,1:OneShot,2:OneShot125,3:BrushedWithRelay,4:BrushedBiPolar
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("PWM_TYPE", 1, AP_MotorsUGV, _pwm_type, PWM_TYPE_NORMAL),

    // @Param: PWM_FREQ
    // @DisplayName: Motor Output PWM freq for brushed motors
    // @Description: Motor Output PWM freq for brushed motors
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

    // @Param: THR_MIN
    // @DisplayName: Throttle minimum
    // @Description: Throttle minimum percentage the autopilot will apply. This is useful for handling a deadzone around low throttle and for preventing internal combustion motors cutting out during missions.
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

    // @Param: SLEWRATE
    // @DisplayName: Throttle slew rate
    // @Description: Throttle slew rate as a percentage of total range per second. A value of 100 allows the motor to change over its full range in one second.  A value of zero disables the limit.  Note some NiMH powered rovers require a lower setting of 40 to reduce current demand to avoid brownouts.
    // @Units: %/s
    // @Range: 0 1000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SLEWRATE", 8, AP_MotorsUGV, _slew_rate, 100),

    // @Param: THST_EXPO
    // @DisplayName: Thrust Curve Expo
    // @Description: Thrust curve exponent (-1 to +1 with 0 being linear)
    // @Range: -1.0 1.0
    // @User: Advanced
    AP_GROUPINFO("THST_EXPO", 9, AP_MotorsUGV, _thrust_curve_expo, 0.0f),

    // @Param: VEC_THR_BASE
    // @DisplayName: Vector thrust throttle base
    // @Description: Throttle level above which steering is scaled down when using vector thrust.  zero to disable vectored thrust
    // @Units: %
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("VEC_THR_BASE", 10, AP_MotorsUGV, _vector_throttle_base, 0.0f),

    // @Param: SPD_SCA_BASE
    // @DisplayName: Motor speed scaling base speed
    // @Description: Speed above which steering is scaled down when using regular steering/throttle vehicles.  zero to disable speed scaling
    // @Units: m/s
    // @Range: 0 10
    // @User: Advanced
    AP_GROUPINFO("SPD_SCA_BASE", 11, AP_MotorsUGV, _speed_scale_base, 1.0f),

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

    // setup motors for custom configs
    if (rover.get_frame_type() != FRAME_TYPE_UNDEFINED) {
        setup_motors();
    }
}

// setup output in case of main CPU failure
void AP_MotorsUGV::setup_safety_output()
{
    if (_pwm_type == PWM_TYPE_BRUSHED_WITH_RELAY) {
        // set trim to min to set duty cycle range (0 - 100%) to servo range
        SRV_Channels::set_trim_to_min_for(SRV_Channel::k_throttle);
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

    // custom config motors set in power percent so -100 ... 100
    for (uint8_t i=0; i<AP_MOTORS_NUM_MOTORS_MAX; i++) {
        SRV_Channel::Aux_servo_function_t function = SRV_Channels::get_motor_function(i);
        SRV_Channels::set_angle(function, 100);
    }

    // mainsail range from 0 to 100
    SRV_Channels::set_range(SRV_Channel::k_mainsail_sheet, 100);
}

// config for frames with vectored motors and custom motor configurations
void AP_MotorsUGV::setup_motors()
{
    // remove existing motors
    for (int8_t i=0; i<AP_MOTORS_NUM_MOTORS_MAX; i++) {
        clear_motors(i);
    }

    // hard coded factor configuration
    switch (rover.get_frame_type()) {

    //   FRAME TYPE NAME
    case FRAME_TYPE_UNDEFINED:
        break;

    case FRAME_TYPE_OMNI3:
        _motors_num = 3;
        add_motor(0, 1.0f, 1.0f, -1.0f);
        add_motor(1, 0.0f, 1.0f, 1.0f);
        add_motor(2, 1.0f, 1.0f, 1.0f);
        break;

    case FRAME_TYPE_OMNIX:
        _motors_num = 4,
        add_motor(0, 1.0f, -1.0f, -1.0f);
        add_motor(1, 1.0f, -1.0f, 1.0f);
        add_motor(2, 1.0f, 1.0f, -1.0f);
        add_motor(3, 1.0f, 1.0f, 1.0f);
        break;

    case FRAME_TYPE_OMNIPLUS:
        _motors_num = 4;
        add_motor(0, 0.0f, 1.0f, 1.0f);
        add_motor(1, 1.0f, 0.0f, 0.0f);
        add_motor(2, 0.0f, -1.0f, 1.0f);
        add_motor(3, 1.0f, 0.0f, 0.0f);
        break;
    }
}

// add motor using separate throttle, steering and lateral factors for frames with custom motor configurations
void AP_MotorsUGV::add_motor(int8_t motor_num, float throttle_factor, float steering_factor, float lateral_factor)
{
    // ensure valid motor number is provided
    if (motor_num >= 0 && motor_num < AP_MOTORS_NUM_MOTORS_MAX) {

        // set throttle, steering and lateral factors
        _throttle_factor[motor_num] = throttle_factor;
        _steering_factor[motor_num] = steering_factor;
        _lateral_factor[motor_num] = lateral_factor;

        add_motor_num(motor_num);
    }
}

// add a motor and set up default output function
void AP_MotorsUGV::add_motor_num(int8_t motor_num)
{
    // ensure a valid motor number is provided
    if (motor_num >= 0 && motor_num < AP_MOTORS_NUM_MOTORS_MAX) {
        uint8_t chan;
        SRV_Channel::Aux_servo_function_t function = SRV_Channels::get_motor_function(motor_num);
        SRV_Channels::set_aux_channel_default(function, motor_num);
        if (!SRV_Channels::find_channel(function, chan)) {
            gcs().send_text(MAV_SEVERITY_ERROR, "Motors: unable to setup motor %u", motor_num);
        }
    }
}

// disable motor and remove all throttle, steering and lateral factor for this motor
void AP_MotorsUGV::clear_motors(int8_t motor_num)
{
    // ensure valid motor number is provided
    if (motor_num >= 0 && motor_num < AP_MOTORS_NUM_MOTORS_MAX) {
        // disable the motor and set factors to zero
        _throttle_factor[motor_num] = 0;
        _steering_factor[motor_num] = 0;
        _lateral_factor[motor_num] = 0;
    }
}

// set steering as a value from -4500 to +4500
//   apply_scaling should be set to false for manual modes where
//   no scaling by speed or angle should be performed
void AP_MotorsUGV::set_steering(float steering, bool apply_scaling)
{
    _steering = steering;
    _scale_steering = apply_scaling;
}

// set throttle as a value from -100 to 100
void AP_MotorsUGV::set_throttle(float throttle)
{
    // only allow setting throttle if armed
    if (!hal.util->get_soft_armed()) {
        return;
    }

    // check throttle is between -_throttle_max and  +_throttle_max
    _throttle = constrain_float(throttle, -_throttle_max, _throttle_max);
}

// set lateral input as a value from -100 to +100
void AP_MotorsUGV::set_lateral(float lateral)
{
    _lateral = constrain_float(lateral, -100.0f, 100.0f);
}

// set mainsail input as a value from 0 to 100
void AP_MotorsUGV::set_mainsail(float mainsail)
{
    _mainsail = constrain_float(mainsail, 0.0f, 100.0f);
}

// get slew limited throttle
// used by manual mode to avoid bad steering behaviour during transitions from forward to reverse
// same as private slew_limit_throttle method (see below) but does not update throttle state
float AP_MotorsUGV::get_slew_limited_throttle(float throttle, float dt) const
{
    if (_slew_rate <= 0) {
        return throttle;
    }

    const float throttle_change_max = MAX(1.0f, (float)_slew_rate * dt);
    return constrain_float(throttle, _throttle_prev - throttle_change_max, _throttle_prev + throttle_change_max);
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

// true if the vehicle has a mainsail
bool AP_MotorsUGV::has_sail() const
{
    return SRV_Channels::function_assigned(SRV_Channel::k_mainsail_sheet);
}

void AP_MotorsUGV::output(bool armed, float ground_speed, float dt)
{
    // soft-armed overrides passed in armed status
    if (!hal.util->get_soft_armed()) {
        armed = false;
        _throttle = 0.0f;
    }

    // sanity check parameters
    sanity_check_parameters();

    // slew limit throttle
    slew_limit_throttle(dt);

    // output for regular steering/throttle style frames
    output_regular(armed, ground_speed, _steering, _throttle);

    // output for skid steering style frames
    output_skid_steering(armed, _steering, _throttle, dt);

    // output for frames with vectored and custom motor configurations
    output_custom_config(armed, _steering, _throttle, _lateral);

    // output to mainsail
    output_mainsail();

    // send values to the PWM timers for output
    SRV_Channels::calc_pwm();
    SRV_Channels::cork();
    SRV_Channels::output_ch_all();
    SRV_Channels::push();
}

// test steering or throttle output as a percentage of the total (range -100 to +100)
// used in response to DO_MOTOR_TEST mavlink command
bool AP_MotorsUGV::output_test_pct(motor_test_order motor_seq, float pct)
{
    // check if the motor_seq is valid
    if (motor_seq >= MOTOR_TEST_LAST) {
        return false;
    }
    pct = constrain_float(pct, -100.0f, 100.0f);

    switch (motor_seq) {
        case MOTOR_TEST_THROTTLE: {
            if (SRV_Channels::function_assigned(SRV_Channel::k_motor1)) {
                output_throttle(SRV_Channel::k_motor1, pct);
            }
            if (SRV_Channels::function_assigned(SRV_Channel::k_throttle)) {
                output_throttle(SRV_Channel::k_throttle, pct);
            }
            break;
        }
        case MOTOR_TEST_STEERING: {
            if (SRV_Channels::function_assigned(SRV_Channel::k_motor2)) {
                output_throttle(SRV_Channel::k_motor2, pct);
            }
            if (SRV_Channels::function_assigned(SRV_Channel::k_steering)) {
                SRV_Channels::set_output_scaled(SRV_Channel::k_steering, pct * 45.0f);
            }
            break;
        }
        case MOTOR_TEST_THROTTLE_LEFT: {
            if (SRV_Channels::function_assigned(SRV_Channel::k_motor3)) {
                output_throttle(SRV_Channel::k_motor3, pct);
            }
            if (SRV_Channels::function_assigned(SRV_Channel::k_throttleLeft)) {
                output_throttle(SRV_Channel::k_throttleLeft, pct);
            }
            break;
        }
        case MOTOR_TEST_THROTTLE_RIGHT: {
            if (SRV_Channels::function_assigned(SRV_Channel::k_motor4)) {
                output_throttle(SRV_Channel::k_motor4, pct);
            }
            if (SRV_Channels::function_assigned(SRV_Channel::k_throttleRight)) {
                output_throttle(SRV_Channel::k_throttleRight, pct);
            }
            break;
        }
        case MOTOR_TEST_MAINSAIL: {
            if (SRV_Channels::function_assigned(SRV_Channel::k_mainsail_sheet)) {
                SRV_Channels::set_output_scaled(SRV_Channel::k_mainsail_sheet, pct);
            }
            break;
        }
        case MOTOR_TEST_LAST:
            return false;
    }
    SRV_Channels::calc_pwm();
    SRV_Channels::cork();
    SRV_Channels::output_ch_all();
    SRV_Channels::push();
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
            if (SRV_Channels::function_assigned(SRV_Channel::k_motor1)) {
                SRV_Channels::set_output_pwm(SRV_Channel::k_motor1, pwm);
            }
            if (SRV_Channels::function_assigned(SRV_Channel::k_throttle)) {
                SRV_Channels::set_output_pwm(SRV_Channel::k_throttle, pwm);
            }
            break;
        }
        case MOTOR_TEST_STEERING: {
            if (SRV_Channels::function_assigned(SRV_Channel::k_motor2)) {
                SRV_Channels::set_output_pwm(SRV_Channel::k_motor2, pwm);
            }
            if (SRV_Channels::function_assigned(SRV_Channel::k_steering)) {
                SRV_Channels::set_output_pwm(SRV_Channel::k_steering, pwm);
            }
            break;
        }
        case MOTOR_TEST_THROTTLE_LEFT: {
            if (SRV_Channels::function_assigned(SRV_Channel::k_motor3)) {
                SRV_Channels::set_output_pwm(SRV_Channel::k_motor3, pwm);
            }
            if (SRV_Channels::function_assigned(SRV_Channel::k_throttleLeft)) {
                SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft, pwm);
            }
            break;
        }
        case MOTOR_TEST_THROTTLE_RIGHT: {
            if (SRV_Channels::function_assigned(SRV_Channel::k_motor4)) {
                SRV_Channels::set_output_pwm(SRV_Channel::k_motor4, pwm);
            }
            if (SRV_Channels::function_assigned(SRV_Channel::k_throttleRight)) {
                SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, pwm);
            }
            break;
        }
        case MOTOR_TEST_MAINSAIL: {
            if (SRV_Channels::function_assigned(SRV_Channel::k_mainsail_sheet)) {
                SRV_Channels::set_output_pwm(SRV_Channel::k_mainsail_sheet, pwm);
            }
            break;
        }
        default:
            return false;
    }
    SRV_Channels::calc_pwm();
    SRV_Channels::cork();
    SRV_Channels::output_ch_all();
    SRV_Channels::push();
    return true;
}

//  returns true if checks pass, false if they fail.  report should be true to send text messages to GCS
bool AP_MotorsUGV::pre_arm_check(bool report) const
{
    // check if both regular and skid steering functions have been defined
    if (SRV_Channels::function_assigned(SRV_Channel::k_throttleLeft) &&
        SRV_Channels::function_assigned(SRV_Channel::k_throttleRight) &&
        SRV_Channels::function_assigned(SRV_Channel::k_throttle) &&
        SRV_Channels::function_assigned(SRV_Channel::k_steering)) {
        if (report) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: regular AND skid steering configured");
        }
        return false;
    }
    // check if only one of skid-steering output has been configured
    if (SRV_Channels::function_assigned(SRV_Channel::k_throttleLeft) != SRV_Channels::function_assigned(SRV_Channel::k_throttleRight)) {
        if (report) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: check skid steering config");
        }
        return false;
    }
    // check if only one of throttle or steering outputs has been configured, if has a sail allow no throttle
    if ((has_sail() || SRV_Channels::function_assigned(SRV_Channel::k_throttle)) != SRV_Channels::function_assigned(SRV_Channel::k_steering)) {
        if (report) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: check steering and throttle config");
        }
        return false;
    }
    // check if one of custom config motors hasn't been configured
    for (uint8_t i=0; i<_motors_num; i++)
    {
        SRV_Channel::Aux_servo_function_t function = SRV_Channels::get_motor_function(i);
        if (SRV_Channels::function_assigned(function)) {
            return true;
        } else {
            if (report) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: check %u", function);
                return false;
            }
        }
    }
    return true;
}

// sanity check parameters
void AP_MotorsUGV::sanity_check_parameters()
{
    _throttle_min = constrain_int16(_throttle_min, 0, 20);
    _throttle_max = constrain_int16(_throttle_max, 30, 100);
    _vector_throttle_base = constrain_float(_vector_throttle_base, 0.0f, 100.0f);
}

// setup pwm output type
void AP_MotorsUGV::setup_pwm_type()
{
    uint16_t motor_mask = 0;

    // work out mask of channels assigned to motors
    motor_mask |= SRV_Channels::get_output_channel_mask(SRV_Channel::k_throttle);
    motor_mask |= SRV_Channels::get_output_channel_mask(SRV_Channel::k_throttleLeft);
    motor_mask |= SRV_Channels::get_output_channel_mask(SRV_Channel::k_throttleRight);
    for (uint8_t i=0; i<_motors_num; i++) {
        motor_mask |= SRV_Channels::get_output_channel_mask(SRV_Channels::get_motor_function(i));
    }
    
    switch (_pwm_type) {
    case PWM_TYPE_ONESHOT:
        hal.rcout->set_output_mode(motor_mask, AP_HAL::RCOutput::MODE_PWM_ONESHOT);
        break;
    case PWM_TYPE_ONESHOT125:
        hal.rcout->set_output_mode(motor_mask, AP_HAL::RCOutput::MODE_PWM_ONESHOT125);
        break;
    case PWM_TYPE_BRUSHED_WITH_RELAY:
    case PWM_TYPE_BRUSHED_BIPOLAR:
        hal.rcout->set_output_mode(motor_mask, AP_HAL::RCOutput::MODE_PWM_BRUSHED);
        hal.rcout->set_freq(motor_mask, uint16_t(_pwm_freq * 1000));
        break;
    default:
        // do nothing
        break;
    }
}

// output to regular steering and throttle channels
void AP_MotorsUGV::output_regular(bool armed, float ground_speed, float steering, float throttle)
{
    // output to throttle channels
    if (armed) {
        if (_scale_steering) {
            // vectored thrust handling
            if (have_vectored_thrust()) {
                if (fabsf(throttle) > _vector_throttle_base) {
                    // scale steering down linearly as throttle increases above _vector_throttle_base
                    steering *= constrain_float(_vector_throttle_base / fabsf(throttle), 0.0f, 1.0f);
                }
            } else {
                // scale steering down as speed increase above MOT_SPD_SCA_BASE (1 m/s default)
                if (is_positive(_speed_scale_base) && (fabsf(ground_speed) > _speed_scale_base)) {
                    steering *= (_speed_scale_base / fabsf(ground_speed));
                } else {
                    // regular steering rover at low speed so set limits to stop I-term build-up in controllers
                    if (!have_skid_steering()) {
                        limit.steer_left = true;
                        limit.steer_right = true;
                    }
                }
                // reverse steering direction when backing up
                if (is_negative(ground_speed)) {
                    steering *= -1.0f;
                }
            }
        } else {
            // reverse steering direction when backing up
            if (is_negative(throttle)) {
                steering *= -1.0f;
            }
        }
        output_throttle(SRV_Channel::k_throttle, throttle);
    } else {
        // handle disarmed case
        if (_disarm_disable_pwm) {
            SRV_Channels::set_output_limit(SRV_Channel::k_throttle, SRV_Channel::SRV_CHANNEL_LIMIT_ZERO_PWM);
        } else {
            SRV_Channels::set_output_limit(SRV_Channel::k_throttle, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
        }
    }

    // clear and set limits based on input
    // we do this here because vectored thrust or speed scaling may have reduced steering request
    set_limits_from_input(armed, steering, throttle);

    // constrain steering
    steering = constrain_float(steering, -4500.0f, 4500.0f);

    // always allow steering to move
    SRV_Channels::set_output_scaled(SRV_Channel::k_steering, steering);
}

// output to skid steering channels
void AP_MotorsUGV::output_skid_steering(bool armed, float steering, float throttle, float dt)
{
    if (!have_skid_steering()) {
        return;
    }

    // clear and set limits based on input
    set_limits_from_input(armed, steering, throttle);

    // constrain steering
    steering = constrain_float(steering, -4500.0f, 4500.0f);

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
    }

    // add in throttle and steering
    const float motor_left = throttle_scaled + steering_scaled;
    const float motor_right = throttle_scaled - steering_scaled;

    // send pwm value to each motor
    output_throttle(SRV_Channel::k_throttleLeft, 100.0f * motor_left, dt);
    output_throttle(SRV_Channel::k_throttleRight, 100.0f * motor_right, dt);
}

// output for custom configurations
void AP_MotorsUGV::output_custom_config(bool armed, float steering, float throttle, float lateral)
{
    // exit immediately if the frame type is set to UNDEFINED
    if (rover.get_frame_type() == FRAME_TYPE_UNDEFINED) {
        return;
    }

    if (armed) {
        // clear and set limits based on input
        set_limits_from_input(armed, steering, throttle);

        // constrain steering
        steering = constrain_float(steering, -4500.0f, 4500.0f);

        // scale throttle, steering and lateral inputs to -1 to 1
        const float scaled_throttle = throttle / 100.0f;
        const float scaled_steering = steering / 4500.0f;
        const float scaled_lateral = lateral / 100.0f;

        float thr_str_ltr_out;
        float thr_str_ltr_max = 1;
        for (uint8_t i=0; i<AP_MOTORS_NUM_MOTORS_MAX; i++) {
            thr_str_ltr_out = (scaled_throttle * _throttle_factor[i]) +
                              (scaled_steering * _steering_factor[i]) +
                              (scaled_lateral * _lateral_factor[i]);
            if (fabsf(thr_str_ltr_out) > thr_str_ltr_max) {
                thr_str_ltr_max = fabsf(thr_str_ltr_out);
            }

            float output_vectored = (thr_str_ltr_out / thr_str_ltr_max);

            // send output for each motor
            output_throttle(SRV_Channels::get_motor_function(i), 100.0f * output_vectored);
        }
    } else {
        // handle disarmed case
        if (_disarm_disable_pwm) {
            for (uint8_t i=0; i<_motors_num; i++) {
                SRV_Channels::set_output_limit(SRV_Channels::get_motor_function(i), SRV_Channel::SRV_CHANNEL_LIMIT_ZERO_PWM);
            }
        } else {
            for (uint8_t i=0; i<_motors_num; i++) {
                SRV_Channels::set_output_limit(SRV_Channels::get_motor_function(i), SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
            }
        }
    }
}

// output throttle value to main throttle channel, left throttle or right throttle.  throttle should be scaled from -100 to 100
void AP_MotorsUGV::output_throttle(SRV_Channel::Aux_servo_function_t function, float throttle, float dt)
{
    // sanity check servo function
    if (function != SRV_Channel::k_throttle && function != SRV_Channel::k_throttleLeft && function != SRV_Channel::k_throttleRight && function != SRV_Channel::k_motor1 && function != SRV_Channel::k_motor2 && function != SRV_Channel::k_motor3 && function!= SRV_Channel::k_motor4) {
        return;
    }

    // constrain and scale output
    throttle = get_scaled_throttle(throttle);

    // apply rate control
    throttle = get_rate_controlled_throttle(function, throttle, dt);

    // set relay if necessary
    if (_pwm_type == PWM_TYPE_BRUSHED_WITH_RELAY) {
        // find the output channel, if not found return
        const SRV_Channel *out_chan = SRV_Channels::get_channel_for(function);
        if (out_chan == nullptr) {
            return;
        }
        const int8_t reverse_multiplier = out_chan->get_reversed() ? -1 : 1;
        bool relay_high = is_negative(reverse_multiplier * throttle);

        switch (function) {
            case SRV_Channel::k_throttle:
            case SRV_Channel::k_throttleLeft:
            case SRV_Channel::k_motor1:
                _relayEvents.do_set_relay(0, relay_high);
                break;
            case SRV_Channel::k_throttleRight:
            case SRV_Channel::k_motor2:
                _relayEvents.do_set_relay(1, relay_high);
                break;
            case SRV_Channel::k_motor3:
                _relayEvents.do_set_relay(2, relay_high);
                break;
            case SRV_Channel::k_motor4:
                _relayEvents.do_set_relay(3, relay_high);
                break;
            default:
                // do nothing
                break;
        }
        // invert the output to always have positive value calculated by calc_pwm
        throttle = reverse_multiplier * fabsf(throttle);
    }

    // output to servo channel
    switch (function) {
        case SRV_Channel::k_throttle:
        case SRV_Channel::k_motor1:
        case SRV_Channel::k_motor2:
        case SRV_Channel::k_motor3:
        case SRV_Channel::k_motor4:
            SRV_Channels::set_output_scaled(function,  throttle);
            break;
        case SRV_Channel::k_throttleLeft:
        case SRV_Channel::k_throttleRight:
            SRV_Channels::set_output_scaled(function,  throttle * 10.0f);
            break;
        default:
            // do nothing
            break;
    }
}

// output for sailboat's mainsail
void AP_MotorsUGV::output_mainsail()
{
    if (!has_sail()) {
        return;
    }

    SRV_Channels::set_output_scaled(SRV_Channel::k_mainsail_sheet, _mainsail);
}

// slew limit throttle for one iteration
void AP_MotorsUGV::slew_limit_throttle(float dt)
{
    const float throttle_orig = _throttle;
    _throttle = get_slew_limited_throttle(_throttle, dt);
    if (throttle_orig > _throttle) {
        limit.throttle_upper = true;
    } else if (throttle_orig < _throttle) {
        limit.throttle_lower = true;
    }
    _throttle_prev = _throttle;
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

// scale a throttle using the _throttle_min and _thrust_curve_expo parameters.  throttle should be in the range -100 to +100
float AP_MotorsUGV::get_scaled_throttle(float throttle) const
{
    // exit immediately if throttle is zero
    if (is_zero(throttle)) {
        return throttle;
    }

    // scale using throttle_min
    if (_throttle_min > 0) {
        if (is_negative(throttle)) {
            throttle = -_throttle_min + (throttle * ((100.0f - _throttle_min) / 100.0f));
        } else {
            throttle = _throttle_min + (throttle * ((100.0f - _throttle_min) / 100.0f));
        }
    }

    // skip further scaling if thrust curve disabled or invalid
    if (is_zero(_thrust_curve_expo) || (_thrust_curve_expo > 1.0f) || (_thrust_curve_expo < -1.0f)) {
        return throttle;
    }

    // calculate scaler
    const float sign = (throttle < 0.0f) ? -1.0f : 1.0f;
    const float throttle_pct = constrain_float(throttle, -100.0f, 100.0f) / 100.0f;
    return 100.0f * sign * ((_thrust_curve_expo - 1.0f) + safe_sqrt((1.0f - _thrust_curve_expo) * (1.0f - _thrust_curve_expo) + 4.0f * _thrust_curve_expo * fabsf(throttle_pct))) / (2.0f * _thrust_curve_expo);
}

// use rate controller to achieve desired throttle
float AP_MotorsUGV::get_rate_controlled_throttle(SRV_Channel::Aux_servo_function_t function, float throttle, float dt)
{
    // require non-zero dt
    if (!is_positive(dt)) {
        return throttle;
    }

    // attempt to rate control left throttle
    if ((function == SRV_Channel::k_throttleLeft) && rover.get_wheel_rate_control().enabled(0)) {
        return rover.get_wheel_rate_control().get_rate_controlled_throttle(0, throttle, dt);
    }

    // rate control right throttle
    if ((function == SRV_Channel::k_throttleRight) && rover.get_wheel_rate_control().enabled(1)) {
        return rover.get_wheel_rate_control().get_rate_controlled_throttle(1, throttle, dt);
    }

    // return throttle unchanged
    return throttle;
}

// return true if motors are moving
bool AP_MotorsUGV::active() const
{
    // if soft disarmed, motors not active
    if (!hal.util->get_soft_armed()) {
        return false;
    }

    // check throttle is active
    if (!is_zero(get_throttle())) {
        return true;
    }

    // skid-steering vehicles active when steering
    if (have_skid_steering() && !is_zero(get_steering())) {
        return true;
    }

    return false;
}
