// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
extern const AP_HAL::HAL& hal;

// Constructor
AP_Motors::AP_Motors(uint16_t loop_rate, uint16_t speed_hz) :
    _loop_rate(loop_rate),
    _speed_hz(speed_hz),
    _roll_in(0.0f),
    _pitch_in(0.0f),
    _yaw_in(0.0f),
    _throttle_in(0.0f),
    _throttle_avg_max(0.0f),
    _throttle_filter(),
    _spool_desired(DESIRED_SHUT_DOWN),
    _batt_voltage(0.0f),
    _batt_current(0.0f),
    _air_density_ratio(1.0f),
    _motor_map_mask(0),
    _motor_fast_mask(0)
{
    // init other flags
    _flags.armed = false;
    _flags.frame_orientation = AP_MOTORS_X_FRAME;
    _flags.interlock = false;

    // setup throttle filtering
    _throttle_filter.set_cutoff_frequency(0.0f);
    _throttle_filter.reset(0.0f);

    // init limit flags
    limit.roll_pitch = true;
    limit.yaw = true;
    limit.throttle_lower = true;
    limit.throttle_upper = true;
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
    if (_motor_map_mask & (1U<<chan)) {
        // we have a mapped motor number for this channel
        chan = _motor_map[chan];
    }
    if (_pwm_type == PWM_TYPE_ONESHOT125 && (_motor_fast_mask & (1U<<chan))) {
        // OneShot125 uses a PWM range from 125 to 250 usec
        pwm /= 8;
        /*
          OneShot125 ESCs can be confused by pulses below 125 or above
          250, making them fail the pulse type auto-detection. This
          happens at least with BLHeli
        */
        if (pwm < 125) {
            pwm = 125;
        } else if (pwm > 250) {
            pwm = 250;
        }
    }
    hal.rcout->write(chan, pwm);
}

/*
  set frequency of a set of channels
 */
void AP_Motors::rc_set_freq(uint32_t mask, uint16_t freq_hz)
{
    mask = rc_map_mask(mask);
    if (freq_hz > 50) {
        _motor_fast_mask |= mask;
    }
    hal.rcout->set_freq(mask, freq_hz);
    if ((_pwm_type == PWM_TYPE_ONESHOT ||
         _pwm_type == PWM_TYPE_ONESHOT125) &&
        freq_hz > 50 &&
        mask != 0) {
        // tell HAL to do immediate output
        hal.rcout->set_output_mode(AP_HAL::RCOutput::MODE_PWM_ONESHOT);
    }
}

void AP_Motors::rc_enable_ch(uint8_t chan)
{
    if (_motor_map_mask & (1U<<chan)) {
        // we have a mapped motor number for this channel
        chan = _motor_map[chan];
    }
    hal.rcout->enable_ch(chan);
}

/*
  map an internal motor mask to real motor mask
 */
uint32_t AP_Motors::rc_map_mask(uint32_t mask) const
{
    uint32_t mask2 = 0;
    for (uint8_t i=0; i<32; i++) {
        uint32_t bit = 1UL<<i;
        if (mask & bit) {
            if ((i < AP_MOTORS_MAX_NUM_MOTORS) && (_motor_map_mask & bit)) {
                // we have a mapped motor number for this channel
                mask2 |= (1UL << _motor_map[i]);
            } else {
                mask2 |= bit;
            }
        }
    }
    return mask2;
}

// convert input in -1 to +1 range to pwm output
int16_t AP_Motors::calc_pwm_output_1to1(float input, const RC_Channel& servo)
{
    int16_t ret;

    input = constrain_float(input, -1.0f, 1.0f);

    if (servo.get_reverse()) {
        input = -input;
    }

    if (input >= 0.0f) {
        ret = ((input * (servo.get_radio_max() - servo.get_radio_trim())) + servo.get_radio_trim());
    } else {
        ret = ((input * (servo.get_radio_trim() - servo.get_radio_min())) + servo.get_radio_trim());
    }

    return constrain_int16(ret, servo.get_radio_min(), servo.get_radio_max());
}

// convert input in 0 to +1 range to pwm output
int16_t AP_Motors::calc_pwm_output_0to1(float input, const RC_Channel& servo)
{
    int16_t ret;

    input = constrain_float(input, 0.0f, 1.0f);

    if (servo.get_reverse()) {
        input = 1.0f-input;
    }

    ret = input * (servo.get_radio_max() - servo.get_radio_min()) + servo.get_radio_min();

    return constrain_int16(ret, servo.get_radio_min(), servo.get_radio_max());
}

/*
  add a motor, setting up _motor_map and _motor_map_mask as needed
 */
void AP_Motors::add_motor_num(int8_t motor_num)
{
    // ensure valid motor number is provided
    if( motor_num >= 0 && motor_num < AP_MOTORS_MAX_NUM_MOTORS ) {
        uint8_t chan;
        if (RC_Channel_aux::find_channel((RC_Channel_aux::Aux_servo_function_t)(RC_Channel_aux::k_motor1+motor_num),
                                         chan)) {
            _motor_map[motor_num] = chan;
            _motor_map_mask |= 1U<<motor_num;
        } else {
            // disable this channel from being used by RC_Channel_aux
            RC_Channel_aux::disable_aux_channel(motor_num);
        }
    }
}
