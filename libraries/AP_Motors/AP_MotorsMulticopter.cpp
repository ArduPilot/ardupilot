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

    // @Param: SPIN_MAX
    // @DisplayName: Motor Spin maximum
    // @Description: Point at which the thrust saturates expressed as a number from 0 to 1 in the entire output range
    // @Values: 0.9:Low, 0.95:Default, 1.0:High
    // @User: Advanced
    AP_GROUPINFO("SPIN_MAX", 9, AP_MotorsMulticopter, _thrust_curve_max, AP_MOTORS_SPIN_MAX_DEFAULT),

    // @Param: BAT_VOLT_MAX
    // @DisplayName: Battery voltage compensation maximum voltage
    // @Description: Battery voltage compensation maximum voltage (voltage above this will have no additional scaling effect on thrust).  Recommend 4.4 * cell count, 0 = Disabled
    // @Range: 6 35
    // @Units: Volts
    // @User: Advanced
    AP_GROUPINFO("BAT_VOLT_MAX", 10, AP_MotorsMulticopter, _batt_voltage_max, AP_MOTORS_BAT_VOLT_MAX_DEFAULT),

    // @Param: BAT_VOLT_MIN
    // @DisplayName: Battery voltage compensation minimum voltage
    // @Description: Battery voltage compensation minimum voltage (voltage below this will have no additional scaling effect on thrust).  Recommend 3.5 * cell count, 0 = Disabled
    // @Range: 6 35
    // @Units: Volts
    // @User: Advanced
    AP_GROUPINFO("BAT_VOLT_MIN", 11, AP_MotorsMulticopter, _batt_voltage_min, AP_MOTORS_BAT_VOLT_MIN_DEFAULT),

    // @Param: BAT_CURR_MAX
    // @DisplayName: Motor Current Max
    // @Description: Maximum current over which maximum throttle is limited (0 = Disabled)
    // @Range: 0 200
    // @Units: Amps
    // @User: Advanced
    AP_GROUPINFO("BAT_CURR_MAX", 12, AP_MotorsMulticopter, _batt_current_max, AP_MOTORS_BAT_CURR_MAX_DEFAULT),

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

    // @Param: PWM_TYPE
    // @DisplayName: Output PWM type
    // @Description: This selects the output PWM type, allowing for normal PWM continuous output or OneShot125
    // @Values: 0:Normal,1:OneShot,2:OneShot125
    // @User: Advanced
    AP_GROUPINFO("PWM_TYPE", 15, AP_MotorsMulticopter, _pwm_type, PWM_TYPE_NORMAL),

    // @Param: PWM_MIN
    // @DisplayName: PWM output miniumum
    // @Description: This sets the min PWM output value that will ever be output to the motors, 0 = use input RC3_MIN
    // @Range: 0 2000
    // @User: Advanced
    AP_GROUPINFO("PWM_MIN", 16, AP_MotorsMulticopter, _pwm_min, 0),

    // @Param: PWM_MAX
    // @DisplayName: PWM output maximum
    // @Description: This sets the max PWM value that will ever be output to the motors, 0 = use input RC3_MAX
    // @Range: 0 2000
    // @User: Advanced
    AP_GROUPINFO("PWM_MAX", 17, AP_MotorsMulticopter, _pwm_max, 0),

    AP_GROUPEND
};

// Constructor
AP_MotorsMulticopter::AP_MotorsMulticopter(uint16_t loop_rate, uint16_t speed_hz) :
    AP_Motors(loop_rate, speed_hz),
    _throttle_rpy_mix_desired(AP_MOTORS_THR_LOW_CMP_DEFAULT),
    _throttle_rpy_mix(AP_MOTORS_THR_LOW_CMP_DEFAULT),
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

    // setup battery voltage filtering
    _batt_voltage_filt.set_cutoff_frequency(AP_MOTORS_BATT_VOLT_FILT_HZ);
    _batt_voltage_filt.reset(1.0f);

    // default throttle ranges (i.e. _min_throttle, _throttle_radio_min, _throttle_radio_max)
    set_throttle_range(130, 1100, 1900);
};

// output - sends commands to the motors
void AP_MotorsMulticopter::output()
{
    // update throttle filter
    update_throttle_filter();

    // update battery resistance
    update_battery_resistance();

    // calc filtered battery voltage and lift_max
    update_lift_max_from_batt_voltage();

    // run spool logic
    output_logic();

    // calculate thrust
    output_armed_stabilizing();

    // apply any thrust compensation for the frame
    thrust_compensation();
    
    // convert rpy_thrust values to pwm
    output_to_motors();
};

// sends minimum values out to the motors
void AP_MotorsMulticopter::output_min()
{
    set_desired_spool_state(DESIRED_SHUT_DOWN);
    _multicopter_flags.spool_mode = SHUT_DOWN;
    output();
}

// update the throttle input filter
void AP_MotorsMulticopter::update_throttle_filter()
{
    if (armed()) {
        _throttle_filter.apply(_throttle_in, 1.0f/_loop_rate);
        // constrain filtered throttle
        if (_throttle_filter.get() < 0.0f) {
            _throttle_filter.reset(0.0f);
        }
        if (_throttle_filter.get() > 1.0f) {
            _throttle_filter.reset(1.0f);
        }
    } else {
        _throttle_filter.reset(0.0f);
    }
}

// return current_limit as a number from 0 ~ 1 in the range throttle_min to throttle_max
//todo: replace this with a variable P term
float AP_MotorsMulticopter::get_current_limit_max_throttle()
{
    // return maximum if current limiting is disabled
    if (_batt_current_max <= 0) {
        _throttle_limit = 1.0f;
        return 1.0f;
    }

    // remove throttle limit if disarmed
    if (!_flags.armed) {
        _throttle_limit = 1.0f;
        return 1.0f;
    }

    float batt_current_ratio = _batt_current/_batt_current_max;

    _throttle_limit += AP_MOTORS_CURRENT_LIMIT_P*(1.0f - batt_current_ratio)/_loop_rate;

    // throttle limit drops to 20% between hover and full throttle
    _throttle_limit = constrain_float(_throttle_limit, 0.2f, 1.0f);

    // limit max throttle
    float throttle_thrust_hover = get_hover_throttle_as_high_end_pct();
    return throttle_thrust_hover + ((1.0-throttle_thrust_hover)*_throttle_limit);
}

// apply_thrust_curve_and_volt_scaling - returns throttle in the range 0 ~ 1
float AP_MotorsMulticopter::apply_thrust_curve_and_volt_scaling(float thrust) const
{
    float throttle_ratio = thrust;
    // apply thrust curve - domain 0.0 to 1.0, range 0.0 to 1.0
    if (_thrust_curve_expo > 0.0f && !is_zero(_batt_voltage_filt.get())){
        throttle_ratio = ((_thrust_curve_expo-1.0f) + safe_sqrt((1.0f-_thrust_curve_expo)*(1.0f-_thrust_curve_expo) + 4.0f*_thrust_curve_expo*_lift_max*thrust))/(2.0f*_thrust_curve_expo*_batt_voltage_filt.get());
    }

    // scale to maximum thrust point
    throttle_ratio *= _thrust_curve_max;

    return constrain_float(throttle_ratio, 0.0f, _thrust_curve_max);
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
    // if disarmed reset resting voltage and current
    if (!_flags.armed) {
        _batt_voltage_resting = _batt_voltage;
        _batt_current_resting = _batt_current;
        _batt_timer = 0;
    } else {
        // update battery resistance when throttle is over hover throttle
        if ((_batt_timer < 400) && ((_batt_current_resting*2.0f) < _batt_current)) {
            if (get_throttle() >= _hover_out) {
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

// update_throttle_rpy_mix - slew set_throttle_rpy_mix to requested value
void AP_MotorsMulticopter::update_throttle_rpy_mix()
{
    // slew _throttle_rpy_mix to _throttle_rpy_mix_desired
    if (_throttle_rpy_mix < _throttle_rpy_mix_desired) {
        // increase quickly (i.e. from 0.1 to 0.9 in 0.4 seconds)
        _throttle_rpy_mix += MIN(2.0f/_loop_rate, _throttle_rpy_mix_desired-_throttle_rpy_mix);
    } else if (_throttle_rpy_mix > _throttle_rpy_mix_desired) {
        // reduce more slowly (from 0.9 to 0.1 in 1.6 seconds)
        _throttle_rpy_mix -= MIN(0.5f/_loop_rate, _throttle_rpy_mix-_throttle_rpy_mix_desired);
    }
    _throttle_rpy_mix = constrain_float(_throttle_rpy_mix, 0.1f, 1.0f);
}

float AP_MotorsMulticopter::get_hover_throttle_as_high_end_pct() const
{
    return (MAX(0,(float)_hover_out-_min_throttle) / (float)(1000-_min_throttle));
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

int16_t AP_MotorsMulticopter::calc_thrust_to_pwm(float thrust_in) const
{
    thrust_in = constrain_float(thrust_in, 0, 1);
    return constrain_int16((get_pwm_output_min() + _min_throttle + apply_thrust_curve_and_volt_scaling(thrust_in) *
            (get_pwm_output_max() - (get_pwm_output_min() + _min_throttle))), get_pwm_output_min() + _min_throttle, get_pwm_output_max());
}

// get minimum or maximum pwm value that can be output to motors
int16_t AP_MotorsMulticopter::get_pwm_output_min() const
{
    // return _pwm_min if both PWM_MIN and PWM_MAX parameters are defined and valid
    if ((_pwm_min > 0) && (_pwm_max > 0) && (_pwm_max > _pwm_min)) {
        return _pwm_min;
    }
    return _throttle_radio_min;
}

// get maximum pwm value that can be output to motors
int16_t AP_MotorsMulticopter::get_pwm_output_max() const
{
    // return _pwm_max if both PWM_MIN and PWM_MAX parameters are defined and valid
    if ((_pwm_min > 0) && (_pwm_max > 0) && (_pwm_max > _pwm_min)) {
        return _pwm_max;
    }
    return _throttle_radio_max;
}

// set_throttle_range - sets the minimum throttle that will be sent to the engines when they're not off (i.e. to prevents issues with some motors spinning and some not at very low throttle)
// also sets throttle channel minimum and maximum pwm
void AP_MotorsMulticopter::set_throttle_range(uint16_t min_throttle, int16_t radio_min, int16_t radio_max)
{
    // sanity check
    if ((radio_max > radio_min) && (min_throttle < (radio_max - radio_min))) {
        _throttle_radio_min = radio_min;
        _throttle_radio_max = radio_max;
    }
    // update _min_throttle
    _min_throttle = (float)min_throttle * ((get_pwm_output_max() - get_pwm_output_min()) / 1000.0f);
}

void AP_MotorsMulticopter::output_logic()
{
    // force desired and current spool mode if disarmed or not interlocked
    if (!_flags.armed || !_flags.interlock) {
        _spool_desired = DESIRED_SHUT_DOWN;
        _multicopter_flags.spool_mode = SHUT_DOWN;
    }

    switch (_multicopter_flags.spool_mode) {
        case SHUT_DOWN:
            // Motors should be stationary.
            // Servos set to their trim values or in a test condition.

            // set limits flags
            limit.roll_pitch = true;
            limit.yaw = true;
            limit.throttle_lower = true;
            limit.throttle_upper = true;

            // make sure the motors are spooling in the correct direction
            if (_spool_desired != DESIRED_SHUT_DOWN) {
                _multicopter_flags.spool_mode = SPIN_WHEN_ARMED;
                break;
            }

            // set and increment ramp variables
            _throttle_low_end_pct = 0.0f;
            _throttle_thrust_max = 0.0f;
            _throttle_rpy_mix = 0.0f;
            _throttle_rpy_mix_desired = 0.0f;
            break;

        case SPIN_WHEN_ARMED: {
            // Motors should be stationary or at spin when armed.
            // Servos should be moving to correct the current attitude.

            // set limits flags
            limit.roll_pitch = true;
            limit.yaw = true;
            limit.throttle_lower = true;
            limit.throttle_upper = true;

            // set and increment ramp variables
            float spool_step = 1.0f/(AP_MOTORS_SPOOL_UP_TIME*_loop_rate);
            if (_spool_desired == DESIRED_SHUT_DOWN){
                _throttle_low_end_pct -= spool_step;
                // constrain ramp value and update mode
                if (_throttle_low_end_pct <= 0.0f) {
                    _throttle_low_end_pct = 0.0f;
                    _multicopter_flags.spool_mode = SHUT_DOWN;
                }
            } else if(_spool_desired == DESIRED_THROTTLE_UNLIMITED) {
                _throttle_low_end_pct += spool_step;
                // constrain ramp value and update mode
                if (_throttle_low_end_pct >= 1.0f) {
                    _throttle_low_end_pct = 1.0f;
                    _multicopter_flags.spool_mode = SPOOL_UP;
                }
            } else {    // _spool_desired == SPIN_WHEN_ARMED
                float spin_when_armed_low_end_pct = 0.0f;
                if (_min_throttle > 0) {
                    spin_when_armed_low_end_pct = (float)_spin_when_armed / _min_throttle;
                }
                _throttle_low_end_pct += constrain_float(spin_when_armed_low_end_pct-_throttle_low_end_pct, -spool_step, spool_step);
            }
            _throttle_thrust_max = 0.0f;
            _throttle_rpy_mix = 0.0f;
            _throttle_rpy_mix_desired = 0.0f;
            break;
        }
        case SPOOL_UP:
            // Maximum throttle should move from minimum to maximum.
            // Servos should exhibit normal flight behavior.

            // initialize limits flags
            limit.roll_pitch = false;
            limit.yaw = false;
            limit.throttle_lower = false;
            limit.throttle_upper = false;

            // make sure the motors are spooling in the correct direction
            if (_spool_desired != DESIRED_THROTTLE_UNLIMITED ){
                _multicopter_flags.spool_mode = SPOOL_DOWN;
                break;
            }

            // set and increment ramp variables
            _throttle_low_end_pct = 1.0f;
            _throttle_thrust_max += 1.0f/(AP_MOTORS_SPOOL_UP_TIME*_loop_rate);
            _throttle_rpy_mix = 0.0f;
            _throttle_rpy_mix_desired = 0.0f;

            // constrain ramp value and update mode
            if (_throttle_thrust_max >= MIN(get_throttle(), get_current_limit_max_throttle())) {
                _throttle_thrust_max = get_current_limit_max_throttle();
                _multicopter_flags.spool_mode = THROTTLE_UNLIMITED;
            } else if (_throttle_thrust_max < 0.0f) {
                _throttle_thrust_max = 0.0f;
            }
            break;

        case THROTTLE_UNLIMITED:
            // Throttle should exhibit normal flight behavior.
            // Servos should exhibit normal flight behavior.

            // initialize limits flags
            limit.roll_pitch = false;
            limit.yaw = false;
            limit.throttle_lower = false;
            limit.throttle_upper = false;

            // make sure the motors are spooling in the correct direction
            if (_spool_desired != DESIRED_THROTTLE_UNLIMITED) {
                _multicopter_flags.spool_mode = SPOOL_DOWN;
                break;
            }

            // set and increment ramp variables
            _throttle_low_end_pct = 1.0f;
            _throttle_thrust_max = get_current_limit_max_throttle();
            update_throttle_rpy_mix();
            break;

        case SPOOL_DOWN:
            // Maximum throttle should move from maximum to minimum.
            // Servos should exhibit normal flight behavior.

            // initialize limits flags
            limit.roll_pitch = false;
            limit.yaw = false;
            limit.throttle_lower = false;
            limit.throttle_upper = false;

            // make sure the motors are spooling in the correct direction
            if (_spool_desired == DESIRED_THROTTLE_UNLIMITED) {
                _multicopter_flags.spool_mode = SPOOL_UP;
                break;
            }

            // set and increment ramp variables
            _throttle_low_end_pct = 1.0f;
            _throttle_thrust_max -= 1.0f/(AP_MOTORS_SPOOL_UP_TIME*_loop_rate);
            _throttle_rpy_mix -= 1.0f/(AP_MOTORS_SPOOL_UP_TIME*_loop_rate);
            _throttle_rpy_mix_desired = _throttle_rpy_mix;

            // constrain ramp value and update mode
            if (_throttle_thrust_max <= 0.0f){
                _throttle_thrust_max = 0.0f;
            }
            if (_throttle_rpy_mix <= 0.0f){
                _throttle_rpy_mix = 0.0f;
            }
            if (_throttle_thrust_max >= get_current_limit_max_throttle()) {
                _throttle_thrust_max = get_current_limit_max_throttle();
            } else if (is_zero(_throttle_thrust_max) && is_zero(_throttle_rpy_mix)) {
                _multicopter_flags.spool_mode = SPIN_WHEN_ARMED;
            }
            break;
    }
}

// passes throttle directly to all motors for ESC calibration.
//   throttle_input is in the range of 0 ~ 1 where 0 will send get_pwm_output_min() and 1 will send get_pwm_output_max()
void AP_MotorsMulticopter::set_throttle_passthrough_for_esc_calibration(float throttle_input)
{
    if (armed()) {
        uint16_t pwm_out = get_pwm_output_min() + constrain_float(throttle_input, 0.0f, 1.0f) * (get_pwm_output_max() - get_pwm_output_min());
        // send the pilot's input directly to each enabled motor
        hal.rcout->cork();
        for (uint16_t i=0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                rc_write(i, pwm_out);
            }
        }
        hal.rcout->push();
    }
}

// output a thrust to all motors that match a given motor mask. This
// is used to control tiltrotor motors in forward flight. Thrust is in
// the range 0 to 1
void AP_MotorsMulticopter::output_motor_mask(float thrust, uint8_t mask)
{
    hal.rcout->cork();
    for (uint8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            int16_t motor_out;
            if (mask & (1U<<i)) {
                motor_out = calc_thrust_to_pwm(thrust);
            } else {
                motor_out = get_pwm_output_min();
            }
            rc_write(i, motor_out);
        }
    }
    hal.rcout->push();
}
