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

#include "Rover.h"

/*
  set frequency of a set of channels
 */
void Rover::set_freq_group(uint8_t group, uint16_t freq_hz)
{
    // TODO :get group mask
    /*hal.rcout->set_freq(mask, freq_hz);
    if ((_pwm_type == PWM_TYPE_ONESHOT ||
         _pwm_type == PWM_TYPE_ONESHOT125) &&
        freq_hz > 50 &&
        mask != 0) {
        // tell HAL to do immediate output
        hal.rcout->set_output_mode(AP_HAL::RCOutput::MODE_PWM_ONESHOT);
    } else if (_pwm_type == PWM_TYPE_BRUSHED16kHz) {
        hal.rcout->set_output_mode(AP_HAL::RCOutput::MODE_PWM_BRUSHED16KHZ);
    }*/

    /*if (rover.g2.pwm_type != PWM_TYPE_NORMAL) {
        if ((rover.g2.pwm_type == PWM_TYPE_ONESHOT || g2.pwm_type == PWM_TYPE_ONESHOT125)) {
            // tell HAL to do immediate output
            // hal.rcout->set_output_mode(AP_HAL::RCOutput::MODE_PWM_ONESHOT);
        } else if (rover.g2.pwm_type == PWM_TYPE_BRUSHED16kHz) {
            hal.rcout->set_output_mode(AP_HAL::RCOutput::MODE_PWM_BRUSHED16KHZ);
            hal.rcout->set_freq((1UL << 0), static_cast<uint16_t>(2000.0f * 0.0625f * g2.pwm_freq));  // Steering group
            hal.rcout->set_freq((1UL << 2), static_cast<uint16_t>(2000.0f * 0.0625f * g2.pwm_freq));  // Throttle group
        }
    }*/
}

// set frame class (i.e. rover, robot, etc.) and type (i.e. rover, tank, skidsteering, 6-wheels etc.)
bool Rover::setup_type_class(ugv_type_class type_class)
{
    // exit immediately if no change
    if (motor_type_class == type_class) {
        return false;
    }
    if (arming.is_armed()) {
        gcs_send_text(MAV_SEVERITY_WARNING, "Cannot change frame when armed");
        return false;
    }
    motor_type_class = type_class;

    if (motor_type_class == Rover::UGV_TYPE_TANK) {
        isTypeTank = true;
    } else {
        isTypeTank = false;
    }

    setup_default_function(motor_type_class);
    // setup the motors
    setup_motors();

    return true;
}

void Rover::output_to_motors()
{
    if (control_mode != MANUAL && control_mode != LEARNING) {
        // limit throttle movement speed
        if (g.throttle_slewrate > 0) {
            SRV_Channels::limit_slew_rate(SRV_Channel::k_throttle, g.throttle_slewrate, G_Dt);
            if (isTypeTank) {
                // when skid steering also limit 2nd channel
                SRV_Channels::limit_slew_rate(SRV_Channel::k_steering, g.throttle_slewrate, G_Dt);
            }
        }
    }

    if (isTypeTank) {
        const float steering_scaled = SRV_Channels::get_output_scaled(SRV_Channel::k_steering) / 4500.0f;
        const float throttle_scaled = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) / 100.0f;
        float motor1 = throttle_scaled + 0.5f * steering_scaled;
        float motor2 = throttle_scaled - 0.5f * steering_scaled;

        if (fabsf(throttle_scaled) <= 0.01f) {  // Use full range for on spot turn
            motor1 = steering_scaled;
            motor2 = -steering_scaled;
        }

        SRV_Channels::set_output_scaled(SRV_Channel::k_throttleLeft,  1000.0f * motor1);
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttleRight, 1000.0f * motor2);
    }

    if (!arming.is_armed()) {
        // Some ESCs get noisy (beep error msgs) if PWM == 0.
        // This little segment aims to avoid this.
        switch (arming.arming_required()) {
            case AP_Arming::NO:
                // keep existing behavior: do nothing to radio_out
                // (don't disarm throttle channel even if AP_Arming class is)
                break;

            case AP_Arming::YES_ZERO_PWM:
                SRV_Channels::set_output_limit(SRV_Channel::k_throttle, SRV_Channel::SRV_CHANNEL_LIMIT_ZERO_PWM);
                SRV_Channels::set_output_limit(SRV_Channel::k_throttleLeft, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
                SRV_Channels::set_output_limit(SRV_Channel::k_throttleRight, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
                if (isTypeTank) {
                    SRV_Channels::set_output_limit(SRV_Channel::k_steering, SRV_Channel::SRV_CHANNEL_LIMIT_ZERO_PWM);
                }
                break;

            case AP_Arming::YES_MIN_PWM:
            default:
                SRV_Channels::set_output_limit(SRV_Channel::k_throttle, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
                SRV_Channels::set_output_limit(SRV_Channel::k_throttleLeft, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
                SRV_Channels::set_output_limit(SRV_Channel::k_throttleRight, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
                if (isTypeTank) {
                    SRV_Channels::set_output_limit(SRV_Channel::k_steering, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
                }
                break;
        }
    }

    SRV_Channels::calc_pwm();

    // send values to the PWM timers for output
    // ----------------------------------------
    SRV_Channels::output_ch_all();
}

/***
 * Set default function for each channels if not already assigned
 */
void Rover::setup_default_function(ugv_type_class type_class)
{
    bool hasChange = false;
    switch (type_class) {
        case UGV_TYPE_ROVER:
            if (!SRV_Channels::function_assigned(SRV_Channel::k_steering)) {
                SRV_Channels::set_default_function(CH_1, SRV_Channel::k_steering);
                hasChange = true;
            }
            if (!SRV_Channels::function_assigned(SRV_Channel::k_throttle)) {
                SRV_Channels::set_default_function(CH_3, SRV_Channel::k_throttle);
                hasChange = true;
            }
            break;
        case UGV_TYPE_TANK:
        case UGV_TYPE_BOAT:
            if (!SRV_Channels::function_assigned(SRV_Channel::k_throttleLeft)) {
                SRV_Channels::set_default_function(CH_1, SRV_Channel::k_throttleLeft);
                hasChange = true;
            }
            if (!SRV_Channels::function_assigned(SRV_Channel::k_throttleRight)) {
                SRV_Channels::set_default_function(CH_3, SRV_Channel::k_throttleRight);
                hasChange = true;
            }
            break;
        case UGV_TYPE_UNDEFINED:
        default:
            break;
    }
    if (hasChange) {
        gcs_send_text(MAV_SEVERITY_WARNING, "TYPE_CLASS changed, please reboot");
    }
}

/***
 * Associate the different ouput pins to their functions
 * @param type_class
 */
void Rover::setup_motors()
{
    // k_steering are limited to -45;45 degree
    SRV_Channels::set_angle(SRV_Channel::k_steering, SERVO_MAX);
    // k_throttle are in power percent so 0 ... 100%
    SRV_Channels::set_angle(SRV_Channel::k_throttle, 100);

    // left/right throttle as -1000 to 1000 values
    SRV_Channels::set_angle(SRV_Channel::k_throttleLeft,  1000);
    SRV_Channels::set_angle(SRV_Channel::k_throttleRight, 1000);
}
