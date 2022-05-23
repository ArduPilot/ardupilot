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

void AP_Motors::get_frame_and_type_string(char *buffer, uint8_t buflen) const
{
    const char *frame_str = get_frame_string();
    const char *type_str = get_type_string();
    if (type_str != nullptr && strlen(type_str)
#if AP_SCRIPTING_ENABLED
        && custom_frame_string == nullptr
#endif
    ) {
        hal.util->snprintf(buffer, buflen, "Frame: %s/%s", frame_str, type_str);
    } else {
        hal.util->snprintf(buffer, buflen, "Frame: %s", frame_str);
    }
}

void AP_Motors::armed(bool arm)
{
    if (_armed != arm) {
        _armed = arm;
        AP_Notify::flags.armed = arm;
        if (!arm) {
            save_params_on_disarm();
        }
    }
};

void AP_Motors::set_desired_spool_state(DesiredSpoolState spool)
{
    if (_armed || (spool == DesiredSpoolState::SHUT_DOWN)) {
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
    if ((1U<<chan) & _motor_pwm_range_mask) {
        // note that PWM_MIN/MAX has been forced to 1000/2000
        SRV_Channels::set_output_scaled(function, pwm - 1000);
    } else {
        SRV_Channels::set_output_pwm(function, pwm);
    }
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
void AP_Motors::rc_set_freq(uint32_t motor_mask, uint16_t freq_hz)
{
    if (freq_hz > 50) {
        _motor_fast_mask |= motor_mask;
    }

    const uint32_t mask = motor_mask_to_srv_channel_mask(motor_mask);
    hal.rcout->set_freq(mask, freq_hz);
    hal.rcout->set_dshot_esc_type(SRV_Channels::get_dshot_esc_type());

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
    case PWM_TYPE_PWM_RANGE:
        /*
          this is a motor output type for multirotors which honours
          the SERVOn_MIN/MAX values per channel
         */
        _motor_pwm_range_mask |= motor_mask;
        for (uint8_t i=0; i<16; i++) {
            if ((1U<<i) & motor_mask) {
                SRV_Channels::set_range(SRV_Channels::get_motor_function(i), 1000);
            }
        }
        hal.rcout->set_output_mode(mask, AP_HAL::RCOutput::MODE_PWM_NORMAL);
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
uint32_t AP_Motors::motor_mask_to_srv_channel_mask(uint32_t mask) const
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

    // set limit flag for pitch, roll and yaw
void AP_Motors::set_limit_flag_pitch_roll_yaw(bool flag)
{
    limit.roll = flag;
    limit.pitch = flag;
    limit.yaw = flag;
}

// returns true if the configured PWM type is digital and should have fixed endpoints
bool AP_Motors::is_digital_pwm_type() const
{
    switch (_pwm_type) {
        case PWM_TYPE_DSHOT150:
        case PWM_TYPE_DSHOT300:
        case PWM_TYPE_DSHOT600:
        case PWM_TYPE_DSHOT1200:
            return true;
        case PWM_TYPE_NORMAL:
        case PWM_TYPE_ONESHOT:
        case PWM_TYPE_ONESHOT125:
        case PWM_TYPE_BRUSHED:
        case PWM_TYPE_PWM_RANGE:
            break;
    }
    return false;
}

// return string corresponding to frame_class
const char* AP_Motors::get_frame_string() const
{
#if AP_SCRIPTING_ENABLED
    if (custom_frame_string != nullptr) {
        return custom_frame_string;
    }
#endif
    return _get_frame_string();
}

#if AP_SCRIPTING_ENABLED
// set custom frame string
void AP_Motors::set_frame_string(const char * str) {
    if (custom_frame_string != nullptr) {
        return;
    }
    const size_t len = strlen(str)+1;
    custom_frame_string = new char[len];
    if (custom_frame_string != nullptr) {
        strncpy(custom_frame_string, str, len);
    }
}
#endif

// output_test_seq - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_Motors::output_test_seq(uint8_t motor_seq, int16_t pwm)
{
    if (armed() && _interlock) {
        _output_test_seq(motor_seq, pwm);
    }
}

namespace AP {
    AP_Motors *motors()
    {
        return AP_Motors::get_singleton();
    }
}
