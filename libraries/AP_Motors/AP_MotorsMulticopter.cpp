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
 *       AP_MotorsMulticopter.cpp - ArduCopter multicopter motors library
 *       Code by Randy Mackay and Robert Lefebvre. DIYDrones.com
 *
 */

#include "AP_MotorsMulticopter.h"
#include <AP_HAL/AP_HAL.h>
extern const AP_HAL::HAL& hal;

// parameters for the motor class
const AP_Param::GroupInfo AP_MotorsMulticopter::var_info[] = {
    // 0 was used by TB_RATIO
    // 1,2,3 were used by throttle curve

    // @Param: SPIN_ARMED
    // @DisplayName: Motors always spin when armed
    // @Description: Controls whether motors always spin when armed (must be below THR_MIN)
    // @Values: 0:Do Not Spin,70:VerySlow,100:Slow,130:Medium,150:Fast
    // @User: Standard
    AP_GROUPINFO("SPIN_ARMED", 5, AP_MotorsMulticopter, _spin_when_armed, AP_MOTORS_SPIN_WHEN_ARMED),

    // @Param: YAW_HEADROOM
    // @DisplayName: Matrix Yaw Min
    // @Description: Yaw control is given at least this pwm range
    // @Range: 0 500
    // @Units: pwm
    // @User: Advanced
    AP_GROUPINFO("YAW_HEADROOM", 6, AP_MotorsMulticopter, _yaw_headroom, AP_MOTORS_YAW_HEADROOM_DEFAULT),

    // 7 was THR_LOW_CMP

    // @Param: THST_EXPO
    // @DisplayName: Thrust Curve Expo
    // @Description: Motor thrust curve exponent (from 0 for linear to 1.0 for second order curve)
    // @Range: 0.25 0.8
    // @User: Advanced
    AP_GROUPINFO("THST_EXPO", 8, AP_MotorsMulticopter, _thrust_curve_expo, AP_MOTORS_THST_EXPO_DEFAULT),

    // @Param: THST_MAX
    // @DisplayName: Thrust Curve Max
    // @Description: Point at which the thrust saturates
    // @Values: 0.9:Low, 1.0:High
    // @User: Advanced
    AP_GROUPINFO("THST_MAX", 9, AP_MotorsMulticopter, _thrust_curve_max, AP_MOTORS_THST_MAX_DEFAULT),

    // @Param: THST_BAT_MAX
    // @DisplayName: Battery voltage compensation maximum voltage
    // @Description: Battery voltage compensation maximum voltage (voltage above this will have no additional scaling effect on thrust).  Recommend 4.4 * cell count, 0 = Disabled
    // @Range: 6 35
    // @Units: Volts
    // @User: Advanced
    AP_GROUPINFO("THST_BAT_MAX", 10, AP_MotorsMulticopter, _batt_voltage_max, AP_MOTORS_THST_BAT_MAX_DEFAULT),

    // @Param: THST_BAT_MIN
    // @DisplayName: Battery voltage compensation minimum voltage
    // @Description: Battery voltage compensation minimum voltage (voltage below this will have no additional scaling effect on thrust).  Recommend 3.5 * cell count, 0 = Disabled
    // @Range: 6 35
    // @Units: Volts
    // @User: Advanced
    AP_GROUPINFO("THST_BAT_MIN", 11, AP_MotorsMulticopter, _batt_voltage_min, AP_MOTORS_THST_BAT_MIN_DEFAULT),

    // @Param: CURR_MAX
    // @DisplayName: Motor Current Max
    // @Description: Maximum current over which maximum throttle is limited (0 = Disabled)
    // @Range: 0 200
    // @Units: Amps
    // @User: Advanced
    AP_GROUPINFO("CURR_MAX", 12, AP_MotorsMulticopter, _batt_current_max, AP_MOTORS_CURR_MAX_DEFAULT),

    // @Param: THR_MIX_MIN
    // @DisplayName: Throttle Mix Minimum
    // @Description: Throttle vs attitude control prioritisation used when landing (higher values mean we prioritise attitude control over throttle)
    // @Range: 0.1 0.25
    // @User: Advanced
    AP_GROUPINFO("THR_MIX_MIN", 13, AP_MotorsMulticopter, _thr_mix_min, AP_MOTORS_THR_MIX_MIN_DEFAULT),

    // @Param: THR_MIX_MAX
    // @DisplayName: Throttle Mix Maximum
    // @Description: Throttle vs attitude control prioritisation used during active flight (higher values mean we prioritise attitude control over throttle)
    // @Range: 0.5 0.9
    // @User: Advanced
    AP_GROUPINFO("THR_MIX_MAX", 14, AP_MotorsMulticopter, _thr_mix_max, AP_MOTORS_THR_MIX_MAX_DEFAULT),

    AP_GROUPEND
};

// Constructor
AP_MotorsMulticopter::AP_MotorsMulticopter(uint16_t loop_rate, uint16_t speed_hz) :
    AP_Motors(loop_rate, speed_hz),
    _spin_when_armed_ramped(0),
    _throttle_thr_mix_desired(AP_MOTORS_THR_LOW_CMP_DEFAULT),
    _throttle_thr_mix(AP_MOTORS_THR_LOW_CMP_DEFAULT),
    _min_throttle(AP_MOTORS_DEFAULT_MIN_THROTTLE),
    _max_throttle(AP_MOTORS_DEFAULT_MAX_THROTTLE),
    _hover_out(AP_MOTORS_DEFAULT_MID_THROTTLE),
    _batt_voltage_resting(0.0f),
    _batt_current_resting(0.0f),
    _batt_resistance(0.0f),
    _batt_timer(0),
    _lift_max(1.0f),
    _throttle_limit(1.0f)
{
    AP_Param::setup_object_defaults(this, var_info);

    // disable all motors by default
    memset(motor_enabled, false, sizeof(motor_enabled));

    // init flags
    _multicopter_flags.slow_start = false;
    _multicopter_flags.slow_start_low_end = true;

    // setup battery voltage filtering
    _batt_voltage_filt.set_cutoff_frequency(AP_MOTORS_BATT_VOLT_FILT_HZ);
    _batt_voltage_filt.reset(1.0f);
};

// output - sends commands to the motors
void AP_MotorsMulticopter::output()
{
    // update throttle filter
    update_throttle_filter();

    // update max throttle
    update_max_throttle();

    // update battery resistance
    update_battery_resistance();

    // calc filtered battery voltage and lift_max
    update_lift_max_from_batt_voltage();

    // move throttle_low_comp towards desired throttle low comp
    update_throttle_thr_mix();

    if (_flags.armed) {
        if (!_flags.interlock) {
            output_armed_zero_throttle();
        } else if (_flags.stabilizing) {
            output_armed_stabilizing();
        } else {
            output_armed_not_stabilizing();
        }
    } else {
        _multicopter_flags.slow_start_low_end = true;
        output_disarmed();
    }
};

// update the throttle input filter
void AP_MotorsMulticopter::update_throttle_filter()
{
    if (armed()) {
        _throttle_filter.apply(_throttle_in, 1.0f/_loop_rate);
    } else {
        _throttle_filter.reset(0.0f);
    }

    // constrain throttle signal to 0-1000
    _throttle_control_input = constrain_float(_throttle_filter.get(),0.0f,1000.0f);
}

// update_max_throttle - updates the limits on _max_throttle if necessary taking into account slow_start_throttle flag
void AP_MotorsMulticopter::update_max_throttle()
{
    // ramp up minimum spin speed if necessary
    if (_multicopter_flags.slow_start_low_end) {
        _spin_when_armed_ramped += AP_MOTOR_SLOW_START_LOW_END_INCREMENT;
        if (_spin_when_armed_ramped >= _spin_when_armed) {
            _spin_when_armed_ramped = _spin_when_armed;
            _multicopter_flags.slow_start_low_end = false;
        }
    }

    // implement slow start
    if (_multicopter_flags.slow_start) {
        // increase slow start throttle
        _max_throttle += AP_MOTOR_SLOW_START_INCREMENT;

        // turn off slow start if we've reached max throttle
        if (_max_throttle >= _throttle_control_input) {
            _max_throttle = AP_MOTORS_DEFAULT_MAX_THROTTLE;
            _multicopter_flags.slow_start = false;
        }
        return;
    }

    // current limit throttle
    current_limit_max_throttle();
}

// current_limit_max_throttle - limits maximum throttle based on current
void AP_MotorsMulticopter::current_limit_max_throttle()
{
    // return maximum if current limiting is disabled
    if (_batt_current_max <= 0) {
        _throttle_limit = 1.0f;
        _max_throttle = AP_MOTORS_DEFAULT_MAX_THROTTLE;
        return;
    }

    // remove throttle limit if throttle is at zero or disarmed
    if(_throttle_control_input <= 0 || !_flags.armed) {
        _throttle_limit = 1.0f;
    }

    // limit throttle if over current
    if (_batt_current > _batt_current_max*1.25f) {
        // Fast drop for extreme over current (1 second)
        _throttle_limit -= 1.0f/_loop_rate;
    } else if(_batt_current > _batt_current_max) {
        // Slow drop for extreme over current (5 second)
        _throttle_limit -= 0.2f/_loop_rate;
    } else {
        // Increase throttle limit (2 second)
        _throttle_limit += 0.5f/_loop_rate;
    }

    // throttle limit drops to 20% between hover and full throttle
    _throttle_limit = constrain_float(_throttle_limit, 0.2f, 1.0f);

    // limit max throttle
    _max_throttle = _hover_out + ((1000-_hover_out)*_throttle_limit);
}

// apply_thrust_curve_and_volt_scaling - returns throttle curve adjusted pwm value (i.e. 1000 ~ 2000)
int16_t AP_MotorsMulticopter::apply_thrust_curve_and_volt_scaling(int16_t pwm_out, int16_t pwm_min, int16_t pwm_max) const
{
    // convert to 0.0 to 1.0 ratio
    float throttle_ratio = ((float)(pwm_out-pwm_min))/((float)(pwm_max-pwm_min));

    // apply thrust curve - domain 0.0 to 1.0, range 0.0 to 1.0
    if (_thrust_curve_expo > 0.0f){
        throttle_ratio = ((_thrust_curve_expo-1.0f) + safe_sqrt((1.0f-_thrust_curve_expo)*(1.0f-_thrust_curve_expo) + 4.0f*_thrust_curve_expo*_lift_max*throttle_ratio))/(2.0f*_thrust_curve_expo*_batt_voltage_filt.get());
    }

    // scale to maximum thrust point
    throttle_ratio *= _thrust_curve_max;

    // convert back to pwm range, constrain and return
    return (int16_t)constrain_float(throttle_ratio*(pwm_max-pwm_min)+pwm_min, pwm_min, (pwm_max-pwm_min)*_thrust_curve_max+pwm_min);
}

// update_lift_max from battery voltage - used for voltage compensation
void AP_MotorsMulticopter::update_lift_max_from_batt_voltage()
{
    // sanity check battery_voltage_min is not too small
    // if disabled or misconfigured exit immediately
    if((_batt_voltage_max <= 0) || (_batt_voltage_min >= _batt_voltage_max) || (_batt_voltage < 0.25f*_batt_voltage_min)) {
        _batt_voltage_filt.reset(1.0f);
        _lift_max = 1.0f;
        return;
    }

    _batt_voltage_min = MAX(_batt_voltage_min, _batt_voltage_max * 0.6f);

    // add current based voltage sag to battery voltage
    float batt_voltage = _batt_voltage + _batt_current * _batt_resistance;
    batt_voltage = constrain_float(batt_voltage, _batt_voltage_min, _batt_voltage_max);

    // filter at 0.5 Hz
    float bvf = _batt_voltage_filt.apply(batt_voltage/_batt_voltage_max, 1.0f/_loop_rate);

    // calculate lift max
    _lift_max = bvf*(1-_thrust_curve_expo) + _thrust_curve_expo*bvf*bvf;
}

// update_battery_resistance - calculate battery resistance when throttle is above hover_out
void AP_MotorsMulticopter::update_battery_resistance()
{
    // if motors are stopped, reset resting voltage and current
    if (_throttle_control_input <= 0 || !_flags.armed) {
        _batt_voltage_resting = _batt_voltage;
        _batt_current_resting = _batt_current;
        _batt_timer = 0;
    } else {
        // update battery resistance when throttle is over hover throttle
        if ((_batt_timer < 400) && ((_batt_current_resting*2.0f) < _batt_current)) {
            if (_throttle_control_input >= _hover_out) {
                // filter reaches 90% in 1/4 the test time
                _batt_resistance += 0.05f*(( (_batt_voltage_resting-_batt_voltage)/(_batt_current-_batt_current_resting) ) - _batt_resistance);
                _batt_timer += 1;
            } else {
                // initialize battery resistance to prevent change in resting voltage estimate
                _batt_resistance = ((_batt_voltage_resting-_batt_voltage)/(_batt_current-_batt_current_resting));
            }
        }
    }
}

// update_throttle_thr_mix - slew set_throttle_thr_mix to requested value
void AP_MotorsMulticopter::update_throttle_thr_mix()
{
    // slew _throttle_thr_mix to _throttle_thr_mix_desired
    if (_throttle_thr_mix < _throttle_thr_mix_desired) {
        // increase quickly (i.e. from 0.1 to 0.9 in 0.4 seconds)
        _throttle_thr_mix += MIN(2.0f/_loop_rate, _throttle_thr_mix_desired-_throttle_thr_mix);
    } else if (_throttle_thr_mix > _throttle_thr_mix_desired) {
        // reduce more slowly (from 0.9 to 0.1 in 1.6 seconds)
        _throttle_thr_mix -= MIN(0.5f/_loop_rate, _throttle_thr_mix-_throttle_thr_mix_desired);
    }
    _throttle_thr_mix = constrain_float(_throttle_thr_mix, 0.1f, 1.0f);
}

// get_hover_throttle_as_pwm - converts hover throttle to pwm (i.e. range 1000 ~ 2000)
int16_t AP_MotorsMulticopter::get_hover_throttle_as_pwm() const
{
    return (_throttle_radio_min + (float)(_throttle_radio_max - _throttle_radio_min) * _hover_out / 1000.0f);
}

float AP_MotorsMulticopter::get_compensation_gain() const
{
    // avoid divide by zero
    if (_lift_max <= 0.0f) {
        return 1.0f;
    }

    float ret = 1.0f / _lift_max;

#if AP_MOTORS_DENSITY_COMP == 1
    // air density ratio is increasing in density / decreasing in altitude
    if (_air_density_ratio > 0.3f && _air_density_ratio < 1.5f) {
        ret *= 1.0f / constrain_float(_air_density_ratio,0.5f,1.25f);
    }
#endif
    return ret;
}

float AP_MotorsMulticopter::rel_pwm_to_thr_range(float pwm) const
{
    return pwm/_throttle_pwm_scalar;
}

float AP_MotorsMulticopter::thr_range_to_rel_pwm(float thr) const
{
    return _throttle_pwm_scalar*thr;
}

// set_throttle_range - sets the minimum throttle that will be sent to the engines when they're not off (i.e. to prevents issues with some motors spinning and some not at very low throttle)
// also sets throttle channel minimum and maximum pwm
void AP_MotorsMulticopter::set_throttle_range(uint16_t min_throttle, int16_t radio_min, int16_t radio_max)
{
    _throttle_radio_min = radio_min;
    _throttle_radio_max = radio_max;
    _throttle_pwm_scalar = (_throttle_radio_max - _throttle_radio_min) / 1000.0f;
    _rpy_pwm_scalar = (_throttle_radio_max - (_throttle_radio_min + _min_throttle)) / 9000.0f;
    _min_throttle = (float)min_throttle * _throttle_pwm_scalar;   
}

// slow_start - set to true to slew motors from current speed to maximum
// Note: this must be set immediately before a step up in throttle
void AP_MotorsMulticopter::slow_start(bool true_false)
{
    // set slow_start flag
    _multicopter_flags.slow_start = true;

    // initialise maximum throttle to current throttle
    _max_throttle = constrain_int16(_throttle_control_input, 0, AP_MOTORS_DEFAULT_MAX_THROTTLE);
}

// throttle_pass_through - passes provided pwm directly to all motors - dangerous but used for initialising ESCs
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsMulticopter::throttle_pass_through(int16_t pwm)
{
    if (armed()) {
        // send the pilot's input directly to each enabled motor
        hal.rcout->cork();
        for (uint16_t i=0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                rc_write(i, pwm);
            }
        }
        hal.rcout->push();
    }
}
