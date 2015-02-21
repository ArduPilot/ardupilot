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
#include <AP_HAL.h>
extern const AP_HAL::HAL& hal;


// initialise motor map
#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
    const uint8_t AP_Motors::_motor_to_channel_map[AP_MOTORS_MAX_NUM_MOTORS] PROGMEM = {APM1_MOTOR_TO_CHANNEL_MAP};
#else
    const uint8_t AP_Motors::_motor_to_channel_map[AP_MOTORS_MAX_NUM_MOTORS] PROGMEM = {APM2_MOTOR_TO_CHANNEL_MAP};
#endif


// parameters for the motor class
const AP_Param::GroupInfo AP_Motors::var_info[] PROGMEM = {
    // 0 was used by TB_RATIO
    // 1,2,3 were used by throttle curve

    // @Param: SPIN_ARMED
    // @DisplayName: Motors always spin when armed
    // @Description: Controls whether motors always spin when armed (must be below THR_MIN)
    // @Values: 0:Do Not Spin,70:VerySlow,100:Slow,130:Medium,150:Fast
    // @User: Standard
    AP_GROUPINFO("SPIN_ARMED", 5, AP_Motors, _spin_when_armed, AP_MOTORS_SPIN_WHEN_ARMED),

    // @Param: YAW_HEADROOM
    // @DisplayName: Matrix Yaw Min
    // @Description: Yaw control is given at least this pwm range
    // @Range: 0 500
    // @Units: pwm
    // @User: Advanced
    AP_GROUPINFO("YAW_HEADROOM", 6, AP_Motors, _yaw_headroom, AP_MOTORS_YAW_HEADROOM_DEFAULT),

    // @Param: THR_LOW_CMP
    // @DisplayName: Motor low throttle compensation
    // @Description: Ratio controlling the max throttle output during competing requests of low throttle from the pilot (or autopilot) and higher throttle for attitude control
    // @Values: 0.2:Favour Throttle Control, 0.5:Equal Weighting, 1:Favour Attitude Control
    // @User: Advanced
    AP_GROUPINFO("THR_LOW_CMP", 7, AP_Motors, _throttle_low_comp, AP_MOTORS_THR_LOW_CMP_DEFAULT),

    // @Param: THST_EXPO
    // @DisplayName: Thrust Curve Expo
    // @Description: Motor thrust curve exponent (from 0 for linear to 1.0 for second order curve)
    // @Range: 0.25 0.8
    // @User: Advanced
    AP_GROUPINFO("THST_EXPO", 8, AP_Motors, _thrust_curve_expo, AP_MOTORS_THST_EXPO_DEFAULT),

    // @Param: THST_MAX
    // @DisplayName: Thrust Curve Max
    // @Description: Point at which the thrust saturates
    // @Values: 0.9:Low, 1.0:High
    // @User: Advanced
    AP_GROUPINFO("THST_MAX", 9, AP_Motors, _thrust_curve_max, AP_MOTORS_THST_MAX_DEFAULT),

    // @Param: THST_BAT_MAX
    // @DisplayName: Battery voltage compensation maximum voltage
    // @Description: Battery voltage compensation maximum voltage (voltage above this will have no additional scaling effect on thrust).  Recommend 4.4 * cell count, 0 = Disabled
    // @Range: 6 35
    // @Units: Volts
    // @User: Advanced
    AP_GROUPINFO("THST_BAT_MAX", 10, AP_Motors, _batt_voltage_max, AP_MOTORS_THST_BAT_MAX_DEFAULT),

    // @Param: THST_BAT_MIN
    // @DisplayName: Battery voltage compensation minimum voltage
    // @Description: Battery voltage compensation minimum voltage (voltage below this will have no additional scaling effect on thrust).  Recommend 3.5 * cell count, 0 = Disabled
    // @Range: 6 35
    // @Units: Volts
    // @User: Advanced
    AP_GROUPINFO("THST_BAT_MIN", 11, AP_Motors, _batt_voltage_min, AP_MOTORS_THST_BAT_MIN_DEFAULT),

    // @Param: CURR_MAX
    // @DisplayName: Motor Current Max
    // @Description: Maximum current over which maximum throttle is limited (0 = Disabled)
    // @Range: 0 200
    // @Units: Amps
    // @User: Advanced
    AP_GROUPINFO("CURR_MAX", 12, AP_Motors, _batt_current_max, AP_MOTORS_CURR_MAX_DEFAULT),

    AP_GROUPEND
};

// Constructor
AP_Motors::AP_Motors(RC_Channel& rc_roll, RC_Channel& rc_pitch, RC_Channel& rc_throttle, RC_Channel& rc_yaw, uint16_t loop_rate, uint16_t speed_hz) :
    _rc_roll(rc_roll),
    _rc_pitch(rc_pitch),
    _rc_throttle(rc_throttle),
    _rc_yaw(rc_yaw),
    _loop_rate(loop_rate),
    _speed_hz(speed_hz),
    _min_throttle(AP_MOTORS_DEFAULT_MIN_THROTTLE),
    _max_throttle(AP_MOTORS_DEFAULT_MAX_THROTTLE),
    _hover_out(AP_MOTORS_DEFAULT_MID_THROTTLE),
    _spin_when_armed_ramped(0),
    _batt_voltage(0.0f),
    _batt_voltage_resting(0.0f),
    _batt_voltage_filt(1.0f),
    _batt_current(0.0f),
    _batt_current_resting(0.0f),
    _batt_resistance(0.0f),
    _batt_timer(0),
    _lift_max(1.0f),
    _throttle_limit(1.0f)
{
    AP_Param::setup_object_defaults(this, var_info);

    // slow start motors from zero to min throttle
    _flags.slow_start_low_end = true;
};

void AP_Motors::armed(bool arm)
{
    _flags.armed = arm;
    if (!_flags.armed) {
        _flags.slow_start_low_end = true;
    }
    AP_Notify::flags.armed = arm;
};

// set_min_throttle - sets the minimum throttle that will be sent to the engines when they're not off (i.e. to prevents issues with some motors spinning and some not at very low throttle)
void AP_Motors::set_min_throttle(uint16_t min_throttle)
{
    _min_throttle = (float)min_throttle * (_rc_throttle.radio_max - _rc_throttle.radio_min) / 1000.0f;
}

// set_mid_throttle - sets the mid throttle which is close to the hover throttle of the copter
// this is used to limit the amount that the stability patch will increase the throttle to give more room for roll, pitch and yaw control
void AP_Motors::set_mid_throttle(uint16_t mid_throttle)
{
    _hover_out = _rc_throttle.radio_min + (float)(_rc_throttle.radio_max - _rc_throttle.radio_min) * mid_throttle / 1000.0f;
}

// throttle_pass_through - passes provided pwm directly to all motors - dangerous but used for initialising ESCs
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_Motors::throttle_pass_through(int16_t pwm)
{
    if (armed()) {
        // send the pilot's input directly to each enabled motor
        for (int16_t i=0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[i]), pwm);
            }
        }
    }
}

// output - sends commands to the motors
void AP_Motors::output()
{
    // update max throttle
    update_max_throttle();

    // calc filtered battery voltage and lift_max
    update_lift_max_from_batt_voltage();

    // output to motors
    if (_flags.armed ) {
        output_armed();
    }else{
        output_disarmed();
    }
};

// slow_start - set to true to slew motors from current speed to maximum
// Note: this must be set immediately before a step up in throttle
void AP_Motors::slow_start(bool true_false)
{
    // set slow_start flag
    _flags.slow_start = true;

    // initialise maximum throttle to current throttle
    _max_throttle = constrain_int16(_rc_throttle.servo_out, 0, AP_MOTORS_DEFAULT_MAX_THROTTLE);
}

// update_max_throttle - updates the limits on _max_throttle if necessary taking into account slow_start_throttle flag
void AP_Motors::update_max_throttle()
{
    // ramp up minimum spin speed if necessary
    if (_flags.slow_start_low_end) {
        _spin_when_armed_ramped += AP_MOTOR_SLOW_START_LOW_END_INCREMENT;
        if (_spin_when_armed_ramped >= _spin_when_armed) {
            _spin_when_armed_ramped = _spin_when_armed;
            _flags.slow_start_low_end = false;
        }
    }

    // return max throttle if we're not slow_starting
    if (!_flags.slow_start) {
        return;
    }

    // increase slow start throttle
    _max_throttle += AP_MOTOR_SLOW_START_INCREMENT;

    // turn off slow start if we've reached max throttle
    if (_max_throttle >= _rc_throttle.servo_out) {
        _max_throttle = AP_MOTORS_DEFAULT_MAX_THROTTLE;
        _flags.slow_start = false;
    }
}

// apply_thrust_curve_and_volt_scaling - returns throttle curve adjusted pwm value (i.e. 1000 ~ 2000)
int16_t AP_Motors::apply_thrust_curve_and_volt_scaling(int16_t pwm_out, int16_t pwm_min, int16_t pwm_max) const
{
    float temp_out = ((float)(pwm_out-pwm_min))/((float)(pwm_max-pwm_min));
    if (_thrust_curve_expo > 0.0f){
        temp_out = ((_thrust_curve_expo-1.0f) + safe_sqrt((1.0f-_thrust_curve_expo)*(1.0f-_thrust_curve_expo) + 4.0f*_thrust_curve_expo*_lift_max*temp_out))/(2.0f*_thrust_curve_expo*_batt_voltage_filt);
    }
    return (temp_out*(_thrust_curve_max*pwm_max-pwm_min)+pwm_min);
}

// update_lift_max from battery voltage - used for voltage compensation
void AP_Motors::update_lift_max_from_batt_voltage()
{
    // sanity check battery_voltage_min is not too small
    _batt_voltage_min = max(_batt_voltage_min, _batt_voltage_max * 0.6f);

    // if disabled or misconfigured exit immediately
    if(_batt_voltage_max <= 0 && _batt_voltage_min >= _batt_voltage_max) {
        _batt_voltage_filt = 1.0f;
        _lift_max = 1.0f;
        return;
    }

    // add current based voltage sag to battery voltage
    float batt_voltage = _batt_voltage + _batt_current * _batt_resistance;
    batt_voltage = constrain_float(batt_voltage, _batt_voltage_min, _batt_voltage_max);

    // filter at 0.5 Hz
    // todo: replace with filter object
    _batt_voltage_filt = _batt_voltage_filt  + 0.007792f*(batt_voltage/_batt_voltage_max-_batt_voltage_filt);         // ratio of current battery voltage to maximum battery voltage
    _lift_max = _batt_voltage_filt*(1-_thrust_curve_expo) + _thrust_curve_expo*_batt_voltage_filt*_batt_voltage_filt;
}
