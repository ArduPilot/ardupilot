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

#include "AP_MotorsMulticopter.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

// parameters for the motor class
const AP_Param::GroupInfo AP_MotorsMulticopter::var_info[] = {
    // 0 was used by TB_RATIO
    // 1,2,3 were used by throttle curve
    // 5 was SPIN_ARMED

    // @Param: YAW_HEADROOM
    // @DisplayName: Matrix Yaw Min
    // @Description: Yaw control is given at least this pwm in microseconds range
    // @Range: 0 500
    // @Units: PWM
    // @User: Advanced
    AP_GROUPINFO("YAW_HEADROOM", 6, AP_MotorsMulticopter, _yaw_headroom, AP_MOTORS_YAW_HEADROOM_DEFAULT),

    // 7 was THR_LOW_CMP

    // @Param: THST_EXPO
    // @DisplayName: Thrust Curve Expo
    // @Description: Motor thrust curve exponent (0.0 for linear to 1.0 for second order curve)
    // @Range: -1.0 1.0
    // @User: Advanced
    AP_GROUPINFO("THST_EXPO", 8, AP_MotorsMulticopter, _thrust_curve_expo, AP_MOTORS_THST_EXPO_DEFAULT),

    // @Param: SPIN_MAX
    // @DisplayName: Motor Spin maximum
    // @Description: Point at which the thrust saturates expressed as a number from 0 to 1 in the entire output range
    // @Values: 0.9:Low, 0.95:Default, 1.0:High
    // @User: Advanced
    AP_GROUPINFO("SPIN_MAX", 9, AP_MotorsMulticopter, _spin_max, AP_MOTORS_SPIN_MAX_DEFAULT),

    // @Param: BAT_VOLT_MAX
    // @DisplayName: Battery voltage compensation maximum voltage
    // @Description: Battery voltage compensation maximum voltage (voltage above this will have no additional scaling effect on thrust).  Recommend 4.2 * cell count, 0 = Disabled
    // @Range: 6 53
    // @Units: V
    // @User: Advanced
    AP_GROUPINFO("BAT_VOLT_MAX", 10, AP_MotorsMulticopter, _batt_voltage_max, AP_MOTORS_BAT_VOLT_MAX_DEFAULT),

    // @Param: BAT_VOLT_MIN
    // @DisplayName: Battery voltage compensation minimum voltage
    // @Description: Battery voltage compensation minimum voltage (voltage below this will have no additional scaling effect on thrust).  Recommend 3.3 * cell count, 0 = Disabled
    // @Range: 6 42
    // @Units: V
    // @User: Advanced
    AP_GROUPINFO("BAT_VOLT_MIN", 11, AP_MotorsMulticopter, _batt_voltage_min, AP_MOTORS_BAT_VOLT_MIN_DEFAULT),

    // @Param: BAT_CURR_MAX
    // @DisplayName: Motor Current Max
    // @Description: Maximum current over which maximum throttle is limited (0 = Disabled)
    // @Range: 0 200
    // @Units: A
    // @User: Advanced
    AP_GROUPINFO("BAT_CURR_MAX", 12, AP_MotorsMulticopter, _batt_current_max, AP_MOTORS_BAT_CURR_MAX_DEFAULT),

    // 13, 14 were used by THR_MIX_MIN, THR_MIX_MAX

    // @Param: PWM_TYPE
    // @DisplayName: Output PWM type
    // @Description: This selects the output PWM type, allowing for normal PWM continuous output, OneShot, brushed or DShot motor output
    // @Values: 0:Normal,1:OneShot,2:OneShot125,3:Brushed,4:DShot150,5:DShot300,6:DShot600,7:DShot1200,8:PWMRange
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("PWM_TYPE", 15, AP_MotorsMulticopter, _pwm_type, PWM_TYPE_NORMAL),

    // @Param: PWM_MIN
    // @DisplayName: PWM output minimum
    // @Description: This sets the min PWM output value in microseconds that will ever be output to the motors
    // @Units: PWM
    // @Range: 0 2000
    // @User: Advanced
    AP_GROUPINFO("PWM_MIN", 16, AP_MotorsMulticopter, _pwm_min, 1000),

    // @Param: PWM_MAX
    // @DisplayName: PWM output maximum
    // @Description: This sets the max PWM value in microseconds that will ever be output to the motors
    // @Units: PWM
    // @Range: 0 2000
    // @User: Advanced
    AP_GROUPINFO("PWM_MAX", 17, AP_MotorsMulticopter, _pwm_max, 2000),

    // @Param: SPIN_MIN
    // @DisplayName: Motor Spin minimum
    // @Description: Point at which the thrust starts expressed as a number from 0 to 1 in the entire output range.  Should be higher than MOT_SPIN_ARM.
    // @Values: 0.0:Low, 0.15:Default, 0.3:High
    // @User: Advanced
    AP_GROUPINFO("SPIN_MIN", 18, AP_MotorsMulticopter, _spin_min, AP_MOTORS_SPIN_MIN_DEFAULT),

    // @Param: SPIN_ARM
    // @DisplayName: Motor Spin armed
    // @Description: Point at which the motors start to spin expressed as a number from 0 to 1 in the entire output range.  Should be lower than MOT_SPIN_MIN.
    // @Values: 0.0:Low, 0.1:Default, 0.2:High
    // @User: Advanced
    AP_GROUPINFO("SPIN_ARM", 19, AP_MotorsMulticopter, _spin_arm, AP_MOTORS_SPIN_ARM_DEFAULT),

    // @Param: BAT_CURR_TC
    // @DisplayName: Motor Current Max Time Constant
    // @Description: Time constant used to limit the maximum current
    // @Range: 0 10
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("BAT_CURR_TC", 20, AP_MotorsMulticopter, _batt_current_time_constant, AP_MOTORS_BAT_CURR_TC_DEFAULT),

    // @Param: THST_HOVER
    // @DisplayName: Thrust Hover Value
    // @Description: Motor thrust needed to hover expressed as a number from 0 to 1
    // @Range: 0.2 0.8
    // @User: Advanced
    AP_GROUPINFO("THST_HOVER", 21, AP_MotorsMulticopter, _throttle_hover, AP_MOTORS_THST_HOVER_DEFAULT),

    // @Param: HOVER_LEARN
    // @DisplayName: Hover Value Learning
    // @Description: Enable/Disable automatic learning of hover throttle
    // @Values{Copter}: 0:Disabled, 1:Learn, 2:Learn and Save
    // @Values{Sub}: 0:Disabled
    // @Values{Plane}: 0:Disabled, 1:Learn, 2:Learn and Save
    // @User: Advanced
    AP_GROUPINFO("HOVER_LEARN", 22, AP_MotorsMulticopter, _throttle_hover_learn, HOVER_LEARN_AND_SAVE),

    // @Param: SAFE_DISARM
    // @DisplayName: Motor PWM output disabled when disarmed
    // @Description: Disables motor PWM output when disarmed
    // @Values: 0:PWM enabled while disarmed, 1:PWM disabled while disarmed
    // @User: Advanced
    AP_GROUPINFO("SAFE_DISARM", 23, AP_MotorsMulticopter, _disarm_disable_pwm, 0),

    // @Param: YAW_SV_ANGLE
    // @DisplayName: Yaw Servo Max Lean Angle
    // @Description: Yaw servo's maximum lean angle (Tricopter only)
    // @Range: 5 80
    // @Units: deg
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO_FRAME("YAW_SV_ANGLE", 35, AP_MotorsMulticopter, _yaw_servo_angle_max_deg, 30, AP_PARAM_FRAME_TRICOPTER),

    // @Param: SPOOL_TIME
    // @DisplayName: Spool up time
    // @Description: Time in seconds to spool up the motors from zero to min throttle. 
    // @Range: 0 2
    // @Units: s
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("SPOOL_TIME", 36, AP_MotorsMulticopter, _spool_up_time, AP_MOTORS_SPOOL_UP_TIME_DEFAULT),

    // @Param: BOOST_SCALE
    // @DisplayName: Motor boost scale
    // @Description: Booster motor output scaling factor vs main throttle.  The output to the BoostThrottle servo will be the main throttle times this scaling factor. A higher scaling factor will put more of the load on the booster motor. A value of 1 will set the BoostThrottle equal to the main throttle.
    // @Range: 0 5
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("BOOST_SCALE", 37, AP_MotorsMulticopter, _boost_scale, 0),

    // 38 RESERVED for BAT_POW_MAX
    
    // @Param: BAT_IDX
    // @DisplayName: Battery compensation index
    // @Description: Which battery monitor should be used for doing compensation
    // @Values: 0:First battery, 1:Second battery
    // @User: Advanced
    AP_GROUPINFO("BAT_IDX", 39, AP_MotorsMulticopter, _batt_idx, 0),

    // @Param: SLEW_UP_TIME
    // @DisplayName: Output slew time for increasing throttle
    // @Description: Time in seconds to slew output from zero to full. This is used to limit the rate at which output can change. Range is constrained between 0 and 0.5.
    // @Range: 0 .5
    // @Units: s
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("SLEW_UP_TIME", 40, AP_MotorsMulticopter, _slew_up_time, AP_MOTORS_SLEW_TIME_DEFAULT),

    // @Param: SLEW_DN_TIME
    // @DisplayName: Output slew time for decreasing throttle
    // @Description: Time in seconds to slew output from full to zero. This is used to limit the rate at which output can change.  Range is constrained between 0 and 0.5.
    // @Range: 0 .5
    // @Units: s
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("SLEW_DN_TIME", 41, AP_MotorsMulticopter, _slew_dn_time, AP_MOTORS_SLEW_TIME_DEFAULT),

    // @Param: SAFE_TIME
    // @DisplayName: Time taken to disable and enable the motor PWM output when disarmed and armed.
    // @Description: Time taken to disable and enable the motor PWM output when disarmed and armed.
    // @Range: 0 5
    // @Units: s
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("SAFE_TIME", 42, AP_MotorsMulticopter, _safe_time, AP_MOTORS_SAFE_TIME_DEFAULT),

    AP_GROUPEND
};

// Constructor
AP_MotorsMulticopter::AP_MotorsMulticopter(uint16_t loop_rate, uint16_t speed_hz) :
                AP_Motors(loop_rate, speed_hz),
                _lift_max(1.0f),
                _throttle_limit(1.0f)
{
    AP_Param::setup_object_defaults(this, var_info);

    // setup battery voltage filtering
    _batt_voltage_filt.set_cutoff_frequency(AP_MOTORS_BATT_VOLT_FILT_HZ);
    _batt_voltage_filt.reset(1.0f);

};

// output - sends commands to the motors
void AP_MotorsMulticopter::output()
{
    // update throttle filter
    update_throttle_filter();

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

    // output any booster throttle
    output_boost_throttle();

    // output raw roll/pitch/yaw/thrust
    output_rpyt();
};

// output booster throttle, if any
void AP_MotorsMulticopter::output_boost_throttle(void)
{
    if (_boost_scale > 0) {
        float throttle = constrain_float(get_throttle() * _boost_scale, 0, 1);
        SRV_Channels::set_output_scaled(SRV_Channel::k_boost_throttle, throttle * 1000);
    } else {
        SRV_Channels::set_output_scaled(SRV_Channel::k_boost_throttle, 0);
    }
}

// output roll/pitch/yaw/thrust
void AP_MotorsMulticopter::output_rpyt(void)
{
    SRV_Channels::set_output_scaled(SRV_Channel::k_roll_out, _roll_in_ff * 4500);
    SRV_Channels::set_output_scaled(SRV_Channel::k_pitch_out, _pitch_in_ff * 4500);
    SRV_Channels::set_output_scaled(SRV_Channel::k_yaw_out, _yaw_in_ff * 4500);
    SRV_Channels::set_output_scaled(SRV_Channel::k_thrust_out, get_throttle() * 1000);
}

// sends minimum values out to the motors
void AP_MotorsMulticopter::output_min()
{
    set_desired_spool_state(DesiredSpoolState::SHUT_DOWN);
    _spool_state = SpoolState::SHUT_DOWN;
    output();
}

// update the throttle input filter
void AP_MotorsMulticopter::update_throttle_filter()
{
    if (armed()) {
        _throttle_filter.apply(_throttle_in, 1.0f / _loop_rate);
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
float AP_MotorsMulticopter::get_current_limit_max_throttle()
{
    AP_BattMonitor &battery = AP::battery();

    float _batt_current;

    if (_batt_current_max <= 0 || // return maximum if current limiting is disabled
        !armed() || // remove throttle limit if disarmed
        !battery.current_amps(_batt_current, _batt_idx)) { // no current monitoring is available
        _throttle_limit = 1.0f;
        return 1.0f;
    }

    float _batt_resistance = battery.get_resistance(_batt_idx);

    if (is_zero(_batt_resistance)) {
        _throttle_limit = 1.0f;
        return 1.0f;
    }

    // calculate the maximum current to prevent voltage sag below _batt_voltage_min
    float batt_current_max = MIN(_batt_current_max, _batt_current + (battery.voltage(_batt_idx) - _batt_voltage_min) / _batt_resistance);

    float batt_current_ratio = _batt_current / batt_current_max;

    float loop_interval = 1.0f / _loop_rate;
    _throttle_limit += (loop_interval / (loop_interval + _batt_current_time_constant)) * (1.0f - batt_current_ratio);

    // throttle limit drops to 20% between hover and full throttle
    _throttle_limit = constrain_float(_throttle_limit, 0.2f, 1.0f);

    // limit max throttle
    return get_throttle_hover() + ((1.0 - get_throttle_hover()) * _throttle_limit);
}

// apply_thrust_curve_and_volt_scaling - returns throttle in the range 0 ~ 1
float AP_MotorsMulticopter::apply_thrust_curve_and_volt_scaling(float thrust) const
{
    float battery_scale = 1.0;
    if (is_positive(_batt_voltage_filt.get())) {
        battery_scale = 1.0 / _batt_voltage_filt.get();
    }
    // apply thrust curve - domain -1.0 to 1.0, range -1.0 to 1.0
    float thrust_curve_expo = constrain_float(_thrust_curve_expo, -1.0f, 1.0f);
    if (is_zero(thrust_curve_expo)) {
        // zero expo means linear, avoid floating point exception for small values
        return _lift_max * thrust * battery_scale;
    }
    float throttle_ratio = ((thrust_curve_expo - 1.0f) + safe_sqrt((1.0f - thrust_curve_expo) * (1.0f - thrust_curve_expo) + 4.0f * thrust_curve_expo * _lift_max * thrust)) / (2.0f * thrust_curve_expo);
    return constrain_float(throttle_ratio * battery_scale, 0.0f, 1.0f);
}

// inverse of above, tested with AP_Motors/examples/expo_inverse_test
// used to calculate equivelent motor throttle level to direct ouput, used in tailsitter transtions
float AP_MotorsMulticopter::remove_thrust_curve_and_volt_scaling(float throttle) const
{
    float battery_scale = 1.0;
    if (is_positive(_batt_voltage_filt.get())) {
        battery_scale = 1.0 / _batt_voltage_filt.get();
    }
    // apply thrust curve - domain -1.0 to 1.0, range -1.0 to 1.0
    float thrust_curve_expo = constrain_float(_thrust_curve_expo, -1.0f, 1.0f);
    if (is_zero(thrust_curve_expo)) {
        // zero expo means linear, avoid floating point exception for small values
        return  throttle / (_lift_max * battery_scale);
    }
    float thrust = ((throttle / battery_scale) * (2.0f * thrust_curve_expo)) - (thrust_curve_expo - 1.0f);
    thrust = (thrust * thrust) - ((1.0f - thrust_curve_expo) * (1.0f - thrust_curve_expo));
    thrust /=  4.0f * thrust_curve_expo * _lift_max;
    return constrain_float(thrust, 0.0f, 1.0f);
}

// update_lift_max from battery voltage - used for voltage compensation
void AP_MotorsMulticopter::update_lift_max_from_batt_voltage()
{
    // sanity check battery_voltage_min is not too small
    // if disabled or misconfigured exit immediately
    float _batt_voltage_resting_estimate = AP::battery().voltage_resting_estimate(_batt_idx);
    if ((_batt_voltage_max <= 0) || (_batt_voltage_min >= _batt_voltage_max) || (_batt_voltage_resting_estimate < 0.25f * _batt_voltage_min)) {
        _batt_voltage_filt.reset(1.0f);
        _lift_max = 1.0f;
        return;
    }

    _batt_voltage_min.set(MAX(_batt_voltage_min, _batt_voltage_max * 0.6f));

    // contrain resting voltage estimate (resting voltage is actual voltage with sag removed based on current draw and resistance)
    _batt_voltage_resting_estimate = constrain_float(_batt_voltage_resting_estimate, _batt_voltage_min, _batt_voltage_max);

    // filter at 0.5 Hz
    float batt_voltage_filt = _batt_voltage_filt.apply(_batt_voltage_resting_estimate / _batt_voltage_max, 1.0f / _loop_rate);

    // calculate lift max
    float thrust_curve_expo = constrain_float(_thrust_curve_expo, -1.0f, 1.0f);
    _lift_max = batt_voltage_filt * (1 - thrust_curve_expo) + thrust_curve_expo * batt_voltage_filt * batt_voltage_filt;
}

// 10hz logging of voltage scaling and max trust
void AP_MotorsMulticopter::Log_Write()
{
    const struct log_MotBatt pkt_mot {
        LOG_PACKET_HEADER_INIT(LOG_MOTBATT_MSG),
        time_us         : AP_HAL::micros64(),
        lift_max        : _lift_max,
        bat_volt        : _batt_voltage_filt.get(),
        th_limit        : _throttle_limit,
        th_average_max  : _throttle_avg_max,
        mot_fail_flags  : (uint8_t)(_thrust_boost | (_thrust_balanced << 1U)),
    };
    AP::logger().WriteBlock(&pkt_mot, sizeof(pkt_mot));
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
        ret *= 1.0f / constrain_float(_air_density_ratio, 0.5f, 1.25f);
    }
#endif
    return ret;
}

// convert actuator output (0~1) range to pwm range
int16_t AP_MotorsMulticopter::output_to_pwm(float actuator)
{
    float pwm_output;
    if (_spool_state == SpoolState::SHUT_DOWN) {
        // in shutdown mode, use PWM 0 or minimum PWM
        if (_disarm_disable_pwm && !armed()) {
            pwm_output = 0;
        } else {
            pwm_output = get_pwm_output_min();
        }
    } else {
        // in all other spool modes, covert to desired PWM
        pwm_output = get_pwm_output_min() + (get_pwm_output_max() - get_pwm_output_min()) * actuator;
    }

    return pwm_output;
}

// converts desired thrust to linearized actuator output in a range of 0~1
float AP_MotorsMulticopter::thrust_to_actuator(float thrust_in) const
{
    thrust_in = constrain_float(thrust_in, 0.0f, 1.0f);
    return _spin_min + (_spin_max - _spin_min) * apply_thrust_curve_and_volt_scaling(thrust_in);
}

// inverse of above, tested with AP_Motors/examples/expo_inverse_test
// used to calculate equivelent motor throttle level to direct ouput, used in tailsitter transtions
float AP_MotorsMulticopter::actuator_to_thrust(float actuator) const
{
    actuator = (actuator - _spin_min) /  (_spin_max - _spin_min);
    return constrain_float(remove_thrust_curve_and_volt_scaling(actuator), 0.0f, 1.0f);
}

// adds slew rate limiting to actuator output
void AP_MotorsMulticopter::set_actuator_with_slew(float& actuator_output, float input)
{
    /*
    If MOT_SLEW_UP_TIME is 0 (default), no slew limit is applied to increasing output.
    If MOT_SLEW_DN_TIME is 0 (default), no slew limit is applied to decreasing output.
    MOT_SLEW_UP_TIME and MOT_SLEW_DN_TIME are constrained to 0.0~0.5 for sanity.
    If spool mode is shutdown, no slew limit is applied to allow immediate disarming of motors.
    */

    // Output limits with no slew time applied
    float output_slew_limit_up = 1.0f;
    float output_slew_limit_dn = 0.0f;

    // If MOT_SLEW_UP_TIME is set, calculate the highest allowed new output value, constrained 0.0~1.0
    if (is_positive(_slew_up_time)) {
        float output_delta_up_max = 1.0f / (constrain_float(_slew_up_time, 0.0f, 0.5f) * _loop_rate);
        output_slew_limit_up = constrain_float(actuator_output + output_delta_up_max, 0.0f, 1.0f);
    }

    // If MOT_SLEW_DN_TIME is set, calculate the lowest allowed new output value, constrained 0.0~1.0
    if (is_positive(_slew_dn_time)) {
        float output_delta_dn_max = 1.0f / (constrain_float(_slew_dn_time, 0.0f, 0.5f) * _loop_rate);
        output_slew_limit_dn = constrain_float(actuator_output - output_delta_dn_max, 0.0f, 1.0f);
    }

    // Constrain change in output to within the above limits
    actuator_output = constrain_float(input, output_slew_limit_dn, output_slew_limit_up);
}

// gradually increase actuator output to spin_min
float AP_MotorsMulticopter::actuator_spin_up_to_ground_idle() const
{
    return constrain_float(_spin_up_ratio, 0.0f, 1.0f) * _spin_min;
}

// parameter checks for MOT_PWM_MIN/MAX, returns true if parameters are valid
bool AP_MotorsMulticopter::check_mot_pwm_params() const
{
    // _pwm_min is a value greater than or equal to 1.
    // _pwm_max is greater than _pwm_min.
    // The values of _pwm_min and _pwm_max are positive values.
    if (_pwm_min < 1 || _pwm_min >= _pwm_max) {
        return false;
    }
    return true;
}

// update_throttle_range - update throttle endpoints
void AP_MotorsMulticopter::update_throttle_range()
{
    // if all outputs are digital adjust the range. We also do this for type PWM_RANGE, as those use the
    // scaled output, which is then mapped to PWM via the SRV_Channel library
    if (SRV_Channels::have_digital_outputs(get_motor_mask()) || (_pwm_type == PWM_TYPE_PWM_RANGE)) {
        _pwm_min.set_and_default(1000);
        _pwm_max.set_and_default(2000);
    }

    hal.rcout->set_esc_scaling(get_pwm_output_min(), get_pwm_output_max());
}

// update the throttle input filter.  should be called at 100hz
void AP_MotorsMulticopter::update_throttle_hover(float dt)
{
    if (_throttle_hover_learn != HOVER_LEARN_DISABLED) {
        // we have chosen to constrain the hover throttle to be within the range reachable by the third order expo polynomial.
        _throttle_hover.set(constrain_float(_throttle_hover + (dt / (dt + AP_MOTORS_THST_HOVER_TC)) * (get_throttle() - _throttle_hover), AP_MOTORS_THST_HOVER_MIN, AP_MOTORS_THST_HOVER_MAX));
    }
}

// run spool logic
void AP_MotorsMulticopter::output_logic()
{
    if (armed()) {
        if (_disarm_disable_pwm && (_disarm_safe_timer < _safe_time)) {
            _disarm_safe_timer += 1.0f/_loop_rate;
        } else {
            _disarm_safe_timer = _safe_time;
        }
    } else {
           _disarm_safe_timer = 0.0f;
    }

    // force desired and current spool mode if disarmed or not interlocked
    if (!armed() || !get_interlock()) {
        _spool_desired = DesiredSpoolState::SHUT_DOWN;
        _spool_state = SpoolState::SHUT_DOWN;
    }

    if (_spool_up_time < 0.05) {
        // prevent float exception
        _spool_up_time.set(0.05);
    }

    const float spool_step = 1.0f / (_spool_up_time * _loop_rate);
    switch (_spool_state) {
    case SpoolState::SHUT_DOWN:
        // Motors should be stationary.
        // Servos set to their trim values or in a test condition.

        // set limits flags
        limit.roll = true;
        limit.pitch = true;
        limit.yaw = true;
        limit.throttle_lower = true;
        limit.throttle_upper = true;

        // make sure the motors are spooling in the correct direction
        if (_spool_desired != DesiredSpoolState::SHUT_DOWN && _disarm_safe_timer >= _safe_time.get()) {
            _spool_state = SpoolState::GROUND_IDLE;
            break;
        }

        // set and increment ramp variables
        _spin_up_ratio = 0.0f;
        _throttle_thrust_max = 0.0f;

        // initialise motor failure variables
        _thrust_boost = false;
        _thrust_boost_ratio = 0.0f;
        break;

    case SpoolState::GROUND_IDLE: {
        // Motors should be stationary or at ground idle.
        // Servos should be moving to correct the current attitude.

        // set limits flags
        limit.roll = true;
        limit.pitch = true;
        limit.yaw = true;
        limit.throttle_lower = true;
        limit.throttle_upper = true;

        // set and increment ramp variables
        switch (_spool_desired) {
        case DesiredSpoolState::SHUT_DOWN:
            _spin_up_ratio -= spool_step;
            // constrain ramp value and update mode
            if (_spin_up_ratio <= 0.0f) {
                _spin_up_ratio = 0.0f;
                _spool_state = SpoolState::SHUT_DOWN;
            }
            break;

        case DesiredSpoolState::THROTTLE_UNLIMITED:
            _spin_up_ratio += spool_step;
            // constrain ramp value and update mode
            if (_spin_up_ratio >= 1.0f) {
                _spin_up_ratio = 1.0f;
                _spool_state = SpoolState::SPOOLING_UP;
            }
            break;

        case DesiredSpoolState::GROUND_IDLE:
            float spin_up_armed_ratio = 0.0f;
            if (_spin_min > 0.0f) {
                spin_up_armed_ratio = _spin_arm / _spin_min;
            }
            _spin_up_ratio += constrain_float(spin_up_armed_ratio - _spin_up_ratio, -spool_step, spool_step);
            break;
        }
        _throttle_thrust_max = 0.0f;

        // initialise motor failure variables
        _thrust_boost = false;
        _thrust_boost_ratio = 0.0f;
        break;
    }
    case SpoolState::SPOOLING_UP:
        // Maximum throttle should move from minimum to maximum.
        // Servos should exhibit normal flight behavior.

        // initialize limits flags
        limit.roll = false;
        limit.pitch = false;
        limit.yaw = false;
        limit.throttle_lower = false;
        limit.throttle_upper = false;

        // make sure the motors are spooling in the correct direction
        if (_spool_desired != DesiredSpoolState::THROTTLE_UNLIMITED) {
            _spool_state = SpoolState::SPOOLING_DOWN;
            break;
        }

        // set and increment ramp variables
        _spin_up_ratio = 1.0f;
        _throttle_thrust_max += spool_step;

        // constrain ramp value and update mode
        if (_throttle_thrust_max >= MIN(get_throttle(), get_current_limit_max_throttle())) {
            _throttle_thrust_max = get_current_limit_max_throttle();
            _spool_state = SpoolState::THROTTLE_UNLIMITED;
        } else if (_throttle_thrust_max < 0.0f) {
            _throttle_thrust_max = 0.0f;
        }

        // initialise motor failure variables
        _thrust_boost = false;
        _thrust_boost_ratio = MAX(0.0, _thrust_boost_ratio - spool_step);
        break;

    case SpoolState::THROTTLE_UNLIMITED:
        // Throttle should exhibit normal flight behavior.
        // Servos should exhibit normal flight behavior.

        // initialize limits flags
        limit.roll = false;
        limit.pitch = false;
        limit.yaw = false;
        limit.throttle_lower = false;
        limit.throttle_upper = false;

        // make sure the motors are spooling in the correct direction
        if (_spool_desired != DesiredSpoolState::THROTTLE_UNLIMITED) {
            _spool_state = SpoolState::SPOOLING_DOWN;
            break;
        }

        // set and increment ramp variables
        _spin_up_ratio = 1.0f;
        _throttle_thrust_max = get_current_limit_max_throttle();

        if (_thrust_boost && !_thrust_balanced) {
            _thrust_boost_ratio = MIN(1.0, _thrust_boost_ratio + spool_step);
        } else {
            _thrust_boost_ratio = MAX(0.0, _thrust_boost_ratio - spool_step);
        }
        break;

    case SpoolState::SPOOLING_DOWN:
        // Maximum throttle should move from maximum to minimum.
        // Servos should exhibit normal flight behavior.

        // initialize limits flags
        limit.roll = false;
        limit.pitch = false;
        limit.yaw = false;
        limit.throttle_lower = false;
        limit.throttle_upper = false;

        // make sure the motors are spooling in the correct direction
        if (_spool_desired == DesiredSpoolState::THROTTLE_UNLIMITED) {
            _spool_state = SpoolState::SPOOLING_UP;
            break;
        }

        // set and increment ramp variables
        _spin_up_ratio = 1.0f;
        _throttle_thrust_max -= spool_step;

        // constrain ramp value and update mode
        if (_throttle_thrust_max <= 0.0f) {
            _throttle_thrust_max = 0.0f;
        }
        if (_throttle_thrust_max >= get_current_limit_max_throttle()) {
            _throttle_thrust_max = get_current_limit_max_throttle();
        } else if (is_zero(_throttle_thrust_max)) {
            _spool_state = SpoolState::GROUND_IDLE;
        }

        _thrust_boost_ratio = MAX(0.0, _thrust_boost_ratio - spool_step);
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
        for (uint16_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                rc_write(i, pwm_out);
            }
        }
        // send pwm output to channels used by bicopter
        SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, pwm_out);
        SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft, pwm_out);
    }
}

// output a thrust to all motors that match a given motor mask. This
// is used to control tiltrotor motors in forward flight. Thrust is in
// the range 0 to 1
void AP_MotorsMulticopter::output_motor_mask(float thrust, uint8_t mask, float rudder_dt)
{
    const int16_t pwm_min = get_pwm_output_min();
    const int16_t pwm_range = get_pwm_output_max() - pwm_min;

    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            if ((mask & (1U << i)) && armed() && get_interlock()) {
                /*
                 apply rudder mixing differential thrust
                 copter frame roll is plane frame yaw as this only
                 apples to either tilted motors or tailsitters
                 */
                float diff_thrust = get_roll_factor(i) * rudder_dt * 0.5f;
                set_actuator_with_slew(_actuator[i], thrust + diff_thrust);
                int16_t pwm_output = pwm_min + pwm_range * _actuator[i];
                rc_write(i, pwm_output);
            } else {
                rc_write(i, pwm_min);
            }
        }
    }
}

// get_motor_mask - returns a bitmask of which outputs are being used for motors (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint32_t AP_MotorsMulticopter::get_motor_mask()
{
    return SRV_Channels::get_output_channel_mask(SRV_Channel::k_boost_throttle);
}

// save parameters as part of disarming
void AP_MotorsMulticopter::save_params_on_disarm()
{
    // save hover throttle
    if (_throttle_hover_learn == HOVER_LEARN_AND_SAVE) {
        _throttle_hover.save();
    }
}

// convert to PWM min and max in the motor lib
void AP_MotorsMulticopter::convert_pwm_min_max_param(int16_t radio_min, int16_t radio_max)
{
    if (_pwm_min.configured() || _pwm_max.configured()) {
        return;
    }
    _pwm_min.set_and_save(radio_min);
    _pwm_max.set_and_save(radio_max);
}
