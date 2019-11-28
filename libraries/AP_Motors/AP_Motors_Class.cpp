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

/*
 *       AP_Motors.cpp - ArduCopter motors library
 *       Code by RandyMackay. DIYDrones.com
 *
 */

#include "AP_Motors_Class.h"
#include <AP_HAL/AP_HAL.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

// singleton instance
AP_Motors *AP_Motors::_singleton;

// Constructor
AP_Motors::AP_Motors(uint16_t loop_rate, uint16_t speed_hz) :
    _loop_rate(loop_rate),
    _speed_hz(speed_hz),
    _throttle_filter(),
    _spool_desired(DesiredSpoolState::SHUT_DOWN),
    _spool_state(SpoolState::SHUT_DOWN),
    _air_density_ratio(1.0f)
{
    _singleton = this;

    // setup throttle filtering
    _throttle_filter.set_cutoff_frequency(0.0f);
    _throttle_filter.reset(0.0f);

    // init limit flags
    limit.roll = true;
    limit.pitch = true;
    limit.yaw = true;
    limit.throttle_lower = true;
    limit.throttle_upper = true;
    _thrust_boost = false;
    _thrust_balanced = true;
};

void AP_Motors::armed(bool arm)
{
    if (_flags.armed != arm) {
        _flags.armed = arm;
        AP_Notify::flags.armed = arm;
        if (!arm) {
            save_params_on_disarm();
        }
    }
};

void AP_Motors::set_desired_spool_state(DesiredSpoolState spool)
{
    if (_flags.armed || (spool == DesiredSpoolState::SHUT_DOWN)) {
        _spool_desired = spool;
    }
};

// pilot input in the -1 ~ +1 range for roll, pitch and yaw. 0~1 range for throttle
void AP_Motors::set_radio_passthrough(float roll_input, float pitch_input, float throttle_input, float yaw_input)
{
    _roll_radio_passthrough = roll_input;
    _pitch_radio_passthrough = pitch_input;
    _throttle_radio_passthrough = throttle_input;
    _yaw_radio_passthrough = yaw_input;
}

/*
  write to an output channel
 */
void AP_Motors::rc_write(uint8_t chan, uint16_t pwm)
{
    SRV_Channel::Aux_servo_function_t function = SRV_Channels::get_motor_function(chan);
    SRV_Channels::set_output_pwm(function, pwm);
}

/*
  write to an output channel for an angle actuator
 */
void AP_Motors::rc_write_angle(uint8_t chan, int16_t angle_cd)
{
    SRV_Channel::Aux_servo_function_t function = SRV_Channels::get_motor_function(chan);
    SRV_Channels::set_output_scaled(function, angle_cd);
}

/*
  set frequency of a set of channels
 */
void AP_Motors::rc_set_freq(uint32_t mask, uint16_t freq_hz)
{
    if (freq_hz > 50) {
        _motor_fast_mask |= mask;
    }

    mask = rc_map_mask(mask);
    hal.rcout->set_freq(mask, freq_hz);

    switch (pwm_type(_pwm_type.get())) {
    case PWM_TYPE_ONESHOT:
        if (freq_hz > 50 && mask != 0) {
            hal.rcout->set_output_mode(mask, AP_HAL::RCOutput::MODE_PWM_ONESHOT);
        }
        break;
    case PWM_TYPE_ONESHOT125:
        if (freq_hz > 50 && mask != 0) {
            hal.rcout->set_output_mode(mask, AP_HAL::RCOutput::MODE_PWM_ONESHOT125);
        }
        break;
    case PWM_TYPE_BRUSHED:
        hal.rcout->set_output_mode(mask, AP_HAL::RCOutput::MODE_PWM_BRUSHED);
        break;
    case PWM_TYPE_DSHOT150:
        hal.rcout->set_output_mode(mask, AP_HAL::RCOutput::MODE_PWM_DSHOT150);
        break;
    case PWM_TYPE_DSHOT300:
        hal.rcout->set_output_mode(mask, AP_HAL::RCOutput::MODE_PWM_DSHOT300);
        break;
    case PWM_TYPE_DSHOT600:
        hal.rcout->set_output_mode(mask, AP_HAL::RCOutput::MODE_PWM_DSHOT600);
        break;
    case PWM_TYPE_DSHOT1200:
        hal.rcout->set_output_mode(mask, AP_HAL::RCOutput::MODE_PWM_DSHOT1200);
        break;
    default:
        hal.rcout->set_output_mode(mask, AP_HAL::RCOutput::MODE_PWM_NORMAL);
        break;
    }
}

/*
  map an internal motor mask to real motor mask, accounting for
  SERVOn_FUNCTION mappings, and allowing for multiple outputs per
  motor number
 */
uint32_t AP_Motors::rc_map_mask(uint32_t mask) const
{
    uint32_t mask2 = 0;
    for (uint8_t i = 0; i < 32; i++) {
        uint32_t bit = 1UL << i;
        if (mask & bit) {
            SRV_Channel::Aux_servo_function_t function = SRV_Channels::get_motor_function(i);
            mask2 |= SRV_Channels::get_output_channel_mask(function);
        }
    }
    return mask2;
}

/*
  add a motor, setting up default output function as needed
 */
void AP_Motors::add_motor_num(int8_t motor_num)
{
    // ensure valid motor number is provided
    if (motor_num >= 0 && motor_num < AP_MOTORS_MAX_NUM_MOTORS) {
        uint8_t chan;
        SRV_Channel::Aux_servo_function_t function = SRV_Channels::get_motor_function(motor_num);
        SRV_Channels::set_aux_channel_default(function, motor_num);
        if (!SRV_Channels::find_channel(function, chan)) {
            gcs().send_text(MAV_SEVERITY_ERROR, "Motors: unable to setup motor %u", motor_num);
        }
    }
}

namespace AP {
    AP_Motors *motors()
    {
        return AP_Motors::get_singleton();
    }
}