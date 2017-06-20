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
        // @Description: This selects the output PWM type, allowing for normal PWM continuous output, OneShot or brushed motor output
        // @Values: 0:Normal,1:OneShot,2:OneShot125,3:Brushed
        // @User: Advanced
        // @RebootRequired: True
        AP_GROUPINFO("PWM_TYPE", 1, AP_MotorsUGV, _pwm_type, PWM_TYPE_NORMAL),

        // @Param: PWM_FREQ
        // @DisplayName: Output PWM freq
        // @Description: This selects the output PWM freq
        // @Values: 0:Normal,1:OneShot,2:OneShot125,3:Brushed
        // @Units: kHz
        // @Range: 1 20
        // @Increment: 1
        // @User: Advanced
        // @RebootRequired: True
        AP_GROUPINFO("PWM_FREQ", 2, AP_MotorsUGV, _pwm_freq, 16),

        AP_GROUPEND
};

void AP_MotorsUGV::failsafeThrottle(bool isFailsafe)
{
    _isFailsafeThrottle = isFailsafe;
}

void AP_MotorsUGV::autoMode(enum mode currentMode)
{
    _isAutoMode = (currentMode > AUTO);
}

void AP_MotorsUGV::setup_motors() const
{
    // k_steering are limited to -45;45 degree
    SRV_Channels::set_angle(SRV_Channel::k_steering, SERVO_MAX);
    // k_throttle are in power percent so -100 ... 100
    SRV_Channels::set_angle(SRV_Channel::k_throttle, 100);

    // left/right throttle as -1000 to 1000 values
    SRV_Channels::set_angle(SRV_Channel::k_throttleLeft,  1000);
    SRV_Channels::set_angle(SRV_Channel::k_throttleRight, 1000);
}

void AP_MotorsUGV::setup_motors_type() const
{
    if (_pwm_type != PWM_TYPE_NORMAL) {
        if ((_pwm_type == PWM_TYPE_ONESHOT || _pwm_type == PWM_TYPE_ONESHOT125)) {
            // tell HAL to do immediate output
            // hal.rcout->set_output_mode(AP_HAL::RCOutput::MODE_PWM_ONESHOT);
        } else if (_pwm_type == PWM_TYPE_BRUSHED) {
            hal.rcout->set_output_mode(AP_HAL::RCOutput::MODE_PWM_BRUSHED);
            /*
             * Group 0: channels 0 1
             * Group 1: channels 4 5 6 7
             * Group 2: channels 2 3
             */
            // TODO : See if we can seperate frequency between groups
            hal.rcout->set_freq((1UL << 0), static_cast<uint16_t>(_pwm_freq * 1000));  // Steering group
            hal.rcout->set_freq((1UL << 2), static_cast<uint16_t>(_pwm_freq * 1000));  // Throttle group
            }
        }
}

void AP_MotorsUGV::set_safety_to_trim() const
{
    // output throttle trim when safety off if arming
    // is setup for min on disarm.  MIN is from plane where MIN is effectively no throttle.
    // For Rover's no throttle means TRIM as rovers can go backwards i.e. MIN throttle is
    // full speed backward.
    if (_arming.arming_required() == AP_Arming::YES_MIN_PWM) {
        SRV_Channels::set_safety_limit(SRV_Channel::k_throttle, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
        if (have_skid_steering()) {
            SRV_Channels::set_safety_limit(SRV_Channel::k_steering, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
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

void AP_MotorsUGV::output_mixing() const
{
    // Apply skid steering mixing
    if (have_skid_steering()) {
        const float steering_scaled = SRV_Channels::get_output_scaled(SRV_Channel::k_steering) / 4500.0f;
        const float throttle_scaled = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) / 100.0f;
        float motor1 = throttle_scaled + 0.5f * steering_scaled;
        float motor2 = throttle_scaled - 0.5f * steering_scaled;
        // Check that we are doing on spot turn
        if (fabsf(throttle_scaled) <= 0.01f) {
            // Use full range for on spot turn
            motor1 = steering_scaled;
            motor2 = -steering_scaled;
        }

        SRV_Channels::set_output_scaled(SRV_Channel::k_throttleLeft,  1000.0f * motor1);
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttleRight, 1000.0f * motor2);
    }
}

bool AP_MotorsUGV::check_status() const
{
    // Check Throttle failsafe in non auto mode. Suppress all ouput
    if (_isFailsafeThrottle && !_isAutoMode) {
        // suppress throttle if in failsafe
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0);
        // suppress steer if in failsafe and skid steer mode
        if (have_skid_steering()) {
            SRV_Channels::set_output_scaled(SRV_Channel::k_steering, 0);
        }
        return false;
    }
    // Check if soft arm. Suppress all ouput
    if (!hal.util->get_soft_armed()) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0);
        // suppress steer if in failsafe and skid steer mode
        if (have_skid_steering()) {
            SRV_Channels::set_output_scaled(SRV_Channel::k_steering, 0);
        }
        return false;
    }
    // Check if armed
    if (!_arming.is_armed()) {
        // Some ESCs get noisy (beep error msgs) if PWM == 0.
        // This little segment aims to avoid this.
        switch (_arming.arming_required()) {
            case AP_Arming::NO:
                // keep existing behavior: do nothing to radio_out
                // (don't disarm throttle channel even if AP_Arming class is)
                break;

            case AP_Arming::YES_ZERO_PWM:
                SRV_Channels::set_output_limit(SRV_Channel::k_throttle, SRV_Channel::SRV_CHANNEL_LIMIT_ZERO_PWM);
                SRV_Channels::set_output_limit(SRV_Channel::k_throttleLeft, SRV_Channel::SRV_CHANNEL_LIMIT_ZERO_PWM);
                SRV_Channels::set_output_limit(SRV_Channel::k_throttleRight, SRV_Channel::SRV_CHANNEL_LIMIT_ZERO_PWM);
                if (have_skid_steering()) {
                    SRV_Channels::set_output_limit(SRV_Channel::k_steering, SRV_Channel::SRV_CHANNEL_LIMIT_ZERO_PWM);
                }
                break;

            case AP_Arming::YES_MIN_PWM:
            default:
                SRV_Channels::set_output_limit(SRV_Channel::k_throttle, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
                SRV_Channels::set_output_limit(SRV_Channel::k_throttleLeft, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
                SRV_Channels::set_output_limit(SRV_Channel::k_throttleRight, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
                if (have_skid_steering()) {
                    SRV_Channels::set_output_limit(SRV_Channel::k_steering, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
                }
                break;
        }
        return false;
    }
    return true;
}

void AP_MotorsUGV::output() const
{
    if (check_status()) {
        output_mixing();
    }

    output_to_motors();
}

void AP_MotorsUGV::output_to_motors() const
{
    SRV_Channels::calc_pwm();

    // send values to the PWM timers for output
    // ----------------------------------------
    hal.rcout->cork();
    SRV_Channels::output_ch_all();
    hal.rcout->push();
}
