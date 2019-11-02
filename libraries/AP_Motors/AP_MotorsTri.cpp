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
#include <GCS_MAVLink/GCS.h>
#include "AP_MotorsTri.h"

#define MEASURED_PIVOT_TOL radians(25)  // if this angle error is exceeded servo feedback will be disabled
#define CAL_TRAVERSE_TIME 2000          // time to allow the servo to move for static voltage readings
#define CAL_HOLD_TIME 2000              // time to average a static voltage reading over

extern const AP_HAL::HAL& hal;

// init
void AP_MotorsTri::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    add_motor_num(AP_MOTORS_MOT_1);
    add_motor_num(AP_MOTORS_MOT_2);
    add_motor_num(AP_MOTORS_MOT_4);

    // set update rate for the 3 motors (but not the servo on channel 7)
    set_update_rate(_speed_hz);

    // set the motor_enabled flag so that the ESCs can be calibrated like other frame types
    motor_enabled[AP_MOTORS_MOT_1] = true;
    motor_enabled[AP_MOTORS_MOT_2] = true;
    motor_enabled[AP_MOTORS_MOT_4] = true;

#if !APM_BUILD_TYPE(APM_BUILD_ArduPlane) // Tilt Rotors do not need a yaw servo
    // find the yaw servo
    if (!SRV_Channels::get_channel_for(SRV_Channel::k_motor7, AP_MOTORS_CH_TRI_YAW)) {
        gcs().send_text(MAV_SEVERITY_ERROR, "MotorsTri: unable to setup yaw channel");
        // don't set initialised_ok
        return;
    }
#endif

    // allow mapping of motor7
    add_motor_num(AP_MOTORS_CH_TRI_YAW);

    SRV_Channels::set_angle(SRV_Channels::get_motor_function(AP_MOTORS_CH_TRI_YAW), _yaw_servo_angle_max_deg*100);

    // check for reverse tricopter
    if (frame_type == MOTOR_FRAME_TYPE_PLUSREV) {
        _pitch_reversed = true;
    }

    _mav_type = MAV_TYPE_TRICOPTER;

    // init the analog yaw servo feedback pin
    _yaw_feedback = hal.analogin->channel(ANALOG_INPUT_NONE);
    if (_yaw_servo_speed == -1) {
        // dont init into a calibration
        _yaw_servo_speed.set_and_save(0);
    }

    // record successful initialisation if what we setup was the desired frame_class
    set_initialised_ok(frame_class == MOTOR_FRAME_TRI);
}

// set frame class (i.e. quad, hexa, heli) and type (i.e. x, plus)
void AP_MotorsTri::set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // check for reverse tricopter
    if (frame_type == MOTOR_FRAME_TYPE_PLUSREV) {
        _pitch_reversed = true;
    } else {
        _pitch_reversed = false;
    }

    set_initialised_ok((frame_class == MOTOR_FRAME_TRI) && SRV_Channels::function_assigned(SRV_Channel::k_motor7));
}

// set update rate to motors - a value in hertz
void AP_MotorsTri::set_update_rate(uint16_t speed_hz)
{
    // record requested speed
    _speed_hz = speed_hz;

    // set update rate for the 3 motors (but not the servo on channel 7)
    uint32_t mask = 
	    1U << AP_MOTORS_MOT_1 |
	    1U << AP_MOTORS_MOT_2 |
	    1U << AP_MOTORS_MOT_4;
    rc_set_freq(mask, _speed_hz);
}

void AP_MotorsTri::output_to_motors()
{
    switch (_spool_state) {
        case SpoolState::SHUT_DOWN: {
            // sends minimum values out to the motors
            rc_write(AP_MOTORS_MOT_1, output_to_pwm(0));
            rc_write(AP_MOTORS_MOT_2, output_to_pwm(0));
            rc_write(AP_MOTORS_MOT_4, output_to_pwm(0));
            float yaw_output = calibrate_yaw();
            rc_write_angle(AP_MOTORS_CH_TRI_YAW, yaw_output);
            break;
        }
        case SpoolState::GROUND_IDLE: {
            // sends output to motors when armed but not flying
            set_actuator_with_slew(_actuator[1], actuator_spin_up_to_ground_idle());
            set_actuator_with_slew(_actuator[2], actuator_spin_up_to_ground_idle());
            set_actuator_with_slew(_actuator[4], actuator_spin_up_to_ground_idle());
            rc_write(AP_MOTORS_MOT_1, output_to_pwm(_actuator[1]));
            rc_write(AP_MOTORS_MOT_2, output_to_pwm(_actuator[2]));
            rc_write(AP_MOTORS_MOT_4, output_to_pwm(_actuator[4]));
            rc_write_angle(AP_MOTORS_CH_TRI_YAW, 0);
            _cal_state = CAL_NONE;
            if (_yaw_servo_speed == -1) {
                _yaw_servo_speed.set_and_save(0);
            }
            break;
        }
        case SpoolState::SPOOLING_UP:
        case SpoolState::THROTTLE_UNLIMITED:
        case SpoolState::SPOOLING_DOWN: {
            // set motor output based on thrust requests
            set_actuator_with_slew(_actuator[1], thrust_to_actuator(_thrust_right));
            set_actuator_with_slew(_actuator[2], thrust_to_actuator(_thrust_left));
            set_actuator_with_slew(_actuator[4], thrust_to_actuator(_thrust_rear));
            rc_write(AP_MOTORS_MOT_1, output_to_pwm(_actuator[1]));
            rc_write(AP_MOTORS_MOT_2, output_to_pwm(_actuator[2]));
            rc_write(AP_MOTORS_MOT_4, output_to_pwm(_actuator[4]));
            rc_write_angle(AP_MOTORS_CH_TRI_YAW, degrees(_desired_pivot_angle)*100);
            _cal_state = CAL_NONE;
            if (_yaw_servo_speed == -1) {
                _yaw_servo_speed.set_and_save(0);
            }
            break;
        }
    }
}

// get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint16_t AP_MotorsTri::get_motor_mask()
{
    // tri copter uses channels 1,2,4 and 7
    uint16_t motor_mask = (1U << AP_MOTORS_MOT_1) |
                          (1U << AP_MOTORS_MOT_2) |
                          (1U << AP_MOTORS_MOT_4);
    uint16_t mask = motor_mask_to_srv_channel_mask(motor_mask);

    // add parent's mask
    mask |= AP_MotorsMulticopter::get_motor_mask();

    return mask;
}

// output_armed - sends commands to the motors
// includes new scaling stability patch
void AP_MotorsTri::output_armed_stabilizing()
{
    float   roll_thrust;                // roll thrust input value, +/- 1.0
    float   pitch_thrust;               // pitch thrust input value, +/- 1.0
    float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
    float   throttle_thrust;            // throttle thrust input value, 0.0 - 1.0
    float   throttle_avg_max;           // throttle thrust average maximum value, 0.0 - 1.0
    float   throttle_thrust_best_rpy;   // throttle providing maximum roll, pitch and yaw range without climbing
    float   rpy_scale = 1.0f;           // this is used to scale the roll, pitch and yaw to fit within the motor limits
    float   rpy_low = 0.0f;             // lowest motor value
    float   rpy_high = 0.0f;            // highest motor value
    float   thr_adj;                    // the difference between the pilot's desired throttle and throttle_thrust_best_rpy

    SRV_Channels::set_angle(SRV_Channels::get_motor_function(AP_MOTORS_CH_TRI_YAW), _yaw_servo_angle_max_deg*100);

    // sanity check YAW_SV_ANGLE parameter value to avoid divide by zero
    _yaw_servo_angle_max_deg = constrain_float(_yaw_servo_angle_max_deg, AP_MOTORS_TRI_SERVO_RANGE_DEG_MIN, AP_MOTORS_TRI_SERVO_RANGE_DEG_MAX);

    // apply voltage and air pressure compensation
    const float compensation_gain = get_compensation_gain();
    roll_thrust = (_roll_in + _roll_in_ff) * compensation_gain;
    pitch_thrust = (_pitch_in + _pitch_in_ff) * compensation_gain;
    yaw_thrust = (_yaw_in + _yaw_in_ff) * compensation_gain * sinf(radians(_yaw_servo_angle_max_deg)); // we scale this so a thrust request of 1.0f will ask for full servo deflection at full rear throttle
    throttle_thrust = get_throttle() * compensation_gain;
    throttle_avg_max = _throttle_avg_max * compensation_gain;

    // check for reversed pitch
    if (_pitch_reversed) {
        pitch_thrust *= -1.0f;
    }

    // calculate desired angle of yaw pivot
    _desired_pivot_angle = safe_asin(yaw_thrust);
    if (fabsf(_desired_pivot_angle) > radians(_yaw_servo_angle_max_deg)) {
        limit.yaw = true;
        _desired_pivot_angle = constrain_float(_desired_pivot_angle, -radians(_yaw_servo_angle_max_deg), radians(_yaw_servo_angle_max_deg));
    }

    /*
        Calculate the angle the pivot is actually at
        1. Calculate how far the pivot could have moved from the last desired angle based on the pivot speed paramiter (if set)
        2. Calculate the position of the servo from the analog feedback pin (if set)
        3. Compare the two to check for analog feedback failure
    */
    float pivot_angle;
    if (_yaw_servo_speed <= 0) {
        pivot_angle = _desired_pivot_angle;
    } else {
        // user must have speed set to use analog feedback
        // calculate the position of the servo
        // we only send the desired position to the servo when flying
        if (_spool_state > SpoolState::GROUND_IDLE) {
            pivot_angle = calculate_new_pivot(_desired_pivot_angle);
        } else {
            pivot_angle = calculate_new_pivot(0.0);
        }

        // if available measure the analog feedback position
        if (_yaw_feedback->set_pin(_yaw_servo_pin)) {
            float pivot_measured = measure_pivot_angle();

            // cheack if there is a large difference between the calculated
            if (fabsf(pivot_angle - pivot_measured) > MEASURED_PIVOT_TOL) {
                // disable analogue feedback
                _yaw_servo_pin.set(0);
                gcs().send_text(MAV_SEVERITY_WARNING, "Yaw Servo Feedback Error");
                gcs().send_text(MAV_SEVERITY_INFO, "MotorsTri: des: %.2f, calc: %.2f, mes %.2f",(double)_desired_pivot_angle,(double)pivot_angle,(double)pivot_measured);
            } else {
                pivot_angle = pivot_measured;
            }
        }
    }

    float pivot_thrust_max = cosf(pivot_angle);
    float thrust_max = 1.0f;

    // sanity check throttle is above zero and below current limited throttle
    if (throttle_thrust <= 0.0f) {
        throttle_thrust = 0.0f;
        limit.throttle_lower = true;
    }
    if (throttle_thrust >= _throttle_thrust_max) {
        throttle_thrust = _throttle_thrust_max;
        limit.throttle_upper = true;
    }

    throttle_avg_max = constrain_float(throttle_avg_max, throttle_thrust, _throttle_thrust_max);

    // The following mix may be offer less coupling between axis but needs testing
    //_thrust_right = roll_thrust * -0.5f + pitch_thrust * 1.0f;
    //_thrust_left = roll_thrust * 0.5f + pitch_thrust * 1.0f;
    //_thrust_rear = 0;

    _thrust_right = roll_thrust * -0.5f + pitch_thrust * 0.5f;
    _thrust_left = roll_thrust * 0.5f + pitch_thrust * 0.5f;
    _thrust_rear = pitch_thrust * -0.5f;

    // calculate roll and pitch for each motor
    // set rpy_low and rpy_high to the lowest and highest values of the motors

    // record lowest roll pitch command
    rpy_low = MIN(_thrust_right, _thrust_left);
    rpy_high = MAX(_thrust_right, _thrust_left);
    if (rpy_low > _thrust_rear) {
        rpy_low = _thrust_rear;
    }
    // check to see if the rear motor will reach maximum thrust before the front two motors
    if ((1.0f - rpy_high) > (pivot_thrust_max - _thrust_rear)) {
        thrust_max = pivot_thrust_max;
        rpy_high = _thrust_rear;
    }

    // calculate throttle that gives most possible room for yaw (range 1000 ~ 2000) which is the lower of:
    //      1. 0.5f - (rpy_low+rpy_high)/2.0 - this would give the maximum possible room margin above the highest motor and below the lowest
    //      2. the higher of:
    //            a) the pilot's throttle input
    //            b) the point _throttle_rpy_mix between the pilot's input throttle and hover-throttle
    //      Situation #2 ensure we never increase the throttle above hover throttle unless the pilot has commanded this.
    //      Situation #2b allows us to raise the throttle above what the pilot commanded but not so far that it would actually cause the copter to rise.
    //      We will choose #1 (the best throttle for yaw control) if that means reducing throttle to the motors (i.e. we favor reducing throttle *because* it provides better yaw control)
    //      We will choose #2 (a mix of pilot and hover throttle) only when the throttle is quite low.  We favor reducing throttle instead of better yaw control because the pilot has commanded it

    // check everything fits
    throttle_thrust_best_rpy = MIN(0.5f * thrust_max - (rpy_low + rpy_high) / 2.0, throttle_avg_max);
    if (is_zero(rpy_low)) {
        rpy_scale = 1.0f;
    } else {
        rpy_scale = constrain_float(-throttle_thrust_best_rpy / rpy_low, 0.0f, 1.0f);
    }

    // calculate how close the motors can come to the desired throttle
    thr_adj = throttle_thrust - throttle_thrust_best_rpy;
    if (rpy_scale < 1.0f) {
        // Full range is being used by roll, pitch, and yaw.
        limit.roll = true;
        limit.pitch = true;
        if (thr_adj > 0.0f) {
            limit.throttle_upper = true;
        }
        thr_adj = 0.0f;
    } else {
        if (thr_adj < -(throttle_thrust_best_rpy + rpy_low)) {
            // Throttle can't be reduced to desired value
            thr_adj = -(throttle_thrust_best_rpy + rpy_low);
        } else if (thr_adj > thrust_max - (throttle_thrust_best_rpy + rpy_high)) {
            // Throttle can't be increased to desired value
            thr_adj = thrust_max - (throttle_thrust_best_rpy + rpy_high);
            limit.throttle_upper = true;
        }
    }

    // determine throttle thrust for harmonic notch
    const float throttle_thrust_best_plus_adj = throttle_thrust_best_rpy + thr_adj;
    // compensation_gain can never be zero
    _throttle_out = throttle_thrust_best_plus_adj / compensation_gain;

    // add scaled roll, pitch, constrained yaw and throttle for each motor
    _thrust_right = throttle_thrust_best_plus_adj + rpy_scale * _thrust_right;
    _thrust_left = throttle_thrust_best_plus_adj + rpy_scale * _thrust_left;
    _thrust_rear = throttle_thrust_best_plus_adj + rpy_scale * _thrust_rear;

    // scale pivot thrust to account for pivot angle
    // we should not need to check for divide by zero as _pivot_angle is constrained to the 5deg ~ 80 deg range
    _thrust_rear = _thrust_rear / cosf(pivot_angle);

    // constrain all outputs to 0.0f to 1.0f
    // test code should be run with these lines commented out as they should not do anything
    _thrust_right = constrain_float(_thrust_right, 0.0f, 1.0f);
    _thrust_left = constrain_float(_thrust_left, 0.0f, 1.0f);
    _thrust_rear = constrain_float(_thrust_rear, 0.0f, 1.0f);
}

// output_test_seq - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsTri::output_test_seq(uint8_t motor_seq, int16_t pwm)
{
    // exit immediately if not armed
    if (!armed()) {
        return;
    }

    // output to motors and servos
    switch (motor_seq) {
        case 1:
            // front right motor
            rc_write(AP_MOTORS_MOT_1, pwm);
            break;
        case 2:
            // back motor
            rc_write(AP_MOTORS_MOT_4, pwm);
            break;
        case 3:
            // back servo
            rc_write(AP_MOTORS_CH_TRI_YAW, pwm);
            break;
        case 4:
            // front left motor
            rc_write(AP_MOTORS_MOT_2, pwm);
            break;
        default:
            // do nothing
            break;
    }
}

/*
  call vehicle supplied thrust compensation if set. This allows for
  vehicle specific thrust compensation for motor arrangements such as
  the forward motors tilting
*/
void AP_MotorsTri::thrust_compensation(void)
{
    if (_thrust_compensation_callback) {
        // convert 3 thrust values into an array indexed by motor number
        float thrust[4] { _thrust_right, _thrust_left, 0, _thrust_rear };

        // apply vehicle supplied compensation function
        _thrust_compensation_callback(thrust, 4);

        // extract compensated thrust values
        _thrust_right = thrust[0];
        _thrust_left = thrust[1];
        _thrust_rear = thrust[3];
    }
}

/*
  override tricopter tail servo output in output_motor_mask
 */
void AP_MotorsTri::output_motor_mask(float thrust, uint8_t mask, float rudder_dt)
{
    // normal multicopter output
    AP_MotorsMulticopter::output_motor_mask(thrust, mask, rudder_dt);

    // and override yaw servo
    rc_write_angle(AP_MOTORS_CH_TRI_YAW, 0);
}

float AP_MotorsTri::get_roll_factor(uint8_t i)
{
    float ret = 0.0f;

    switch (i) {
        // right motor
        case AP_MOTORS_MOT_1:
            ret = -1.0f;
            break;
            // left motor
        case AP_MOTORS_MOT_2:
            ret = 1.0f;
            break;
    }

    return ret;
}

// calculate the new pivot angle given its last position and speed
float AP_MotorsTri::calculate_new_pivot(float desired_pivot_angle)
{
    const float max_change = radians(_yaw_servo_speed) * (1.0 / _loop_rate);
    return constrain_float(desired_pivot_angle, _last_pivot_angle-max_change, _last_pivot_angle+max_change);
}

float AP_MotorsTri::measure_pivot_angle()
{
    float voltage = _yaw_feedback->voltage_latest();
    // linear interpolation between min, trim and max to angle

    // see if voltage is between min and trim
    if (MIN(_yaw_servo_min_voltage,_yaw_servo_mid_voltage) < voltage && voltage < MAX(_yaw_servo_min_voltage,_yaw_servo_mid_voltage)) {
        if (_yaw_servo_min_voltage < _yaw_servo_mid_voltage) {
            return linear_interpolate(-radians(_yaw_servo_angle_max_deg), 0.0, voltage, _yaw_servo_min_voltage, _yaw_servo_mid_voltage);
        }
        return linear_interpolate(0.0, -radians(_yaw_servo_angle_max_deg), voltage, _yaw_servo_mid_voltage, _yaw_servo_min_voltage);
    }
    if (_yaw_servo_mid_voltage < _yaw_servo_max_voltage) {
        return linear_interpolate(0.0, radians(_yaw_servo_angle_max_deg), voltage, _yaw_servo_mid_voltage, _yaw_servo_max_voltage);
    }
    return linear_interpolate(radians(_yaw_servo_angle_max_deg), 0.0, voltage, _yaw_servo_max_voltage, _yaw_servo_mid_voltage);
}

float AP_MotorsTri::calibrate_yaw()
{

    if (_yaw_servo_speed != -1) {
        _cal_state = CAL_NONE;
        return 0.0;
    }
    if (_cal_state == CAL_FAILED) {
        _cal_state = CAL_NONE;
        _yaw_servo_speed.set_and_save(0);
        return 0.0;
    }
    if (!_yaw_feedback->set_pin(_yaw_servo_pin)) {
        _cal_state = CAL_NONE;
        _yaw_servo_speed.set_and_save(0);
        gcs().send_text(MAV_SEVERITY_ERROR, "MotorsTri: cannot auto cal, no feedback pin not found");
        return 0.0;
    }

    // make sure angle and pin are set correctly
    SRV_Channels::set_angle(SRV_Channels::get_motor_function(AP_MOTORS_CH_TRI_YAW), _yaw_servo_angle_max_deg*100);

    uint32_t now = AP_HAL::millis();

    switch (_cal_state) {
        case CAL_NONE: {
            _cal_state = CAL_MIN;
            _cal_start_ms = now;
            FALLTHROUGH;
        }

        case CAL_MIN: {
            if (now - _cal_start_ms > CAL_TRAVERSE_TIME) {
                _count++;
                _reading_sum += _yaw_feedback->voltage_latest();
                // take average for another 5 sec
                if (now - _cal_start_ms > CAL_TRAVERSE_TIME + CAL_HOLD_TIME && _count > 0) {
                    _voltage_min_temp = _reading_sum / _count;
                    _reading_sum = 0;
                    _count = 0;
                    _cal_start_ms = now;
                    _cal_state = CAL_MID;
                    gcs().send_text(MAV_SEVERITY_INFO, "MotorsTri: volt min: %.2f",(double)_voltage_min_temp);
                }
            }
            return -_yaw_servo_angle_max_deg*100;
        }

        case CAL_MID: {
            // give 5 seconds for the servo to move
            if (now - _cal_start_ms > CAL_TRAVERSE_TIME) {
                _count++;
                _reading_sum += _yaw_feedback->voltage_latest();
                // take average for another 5 sec
                if (now - _cal_start_ms > CAL_TRAVERSE_TIME + CAL_HOLD_TIME && _count > 0) {
                    _voltage_mid_temp = _reading_sum / _count;
                    _reading_sum = 0;
                    _count = 0;
                    _cal_start_ms = now;
                    _cal_state = CAL_MAX;
                    gcs().send_text(MAV_SEVERITY_INFO, "MotorsTri: volt mid: %.2f",(double)_voltage_mid_temp);
                }
            }
            return 0.0;
        }

        case CAL_MAX: {
            // give 5 seconds for the servo to move
            if (now - _cal_start_ms > CAL_TRAVERSE_TIME) {
                _count++;
                _reading_sum += _yaw_feedback->voltage_latest();
                // take average for another 5 sec
                if (now - _cal_start_ms > CAL_TRAVERSE_TIME + CAL_HOLD_TIME && _count > 0) {
                    _voltage_max_temp = _reading_sum / _count;
                    _reading_sum = 0;
                    _count = 0;
                    _cal_start_ms = now;
                    gcs().send_text(MAV_SEVERITY_INFO, "MotorsTri: volt max: %.2f",(double)_voltage_max_temp);

                    // check if the found values seem OK
                    float voltage_diff_1 = _voltage_mid_temp - _voltage_min_temp;
                    float voltage_diff_2 = _voltage_max_temp - _voltage_mid_temp;
                    if (is_positive(voltage_diff_1) != is_positive(voltage_diff_1)) {
                        gcs().send_text(MAV_SEVERITY_INFO, "MotorsTri: cal failed - mid not between min and max");
                        _cal_state = CAL_FAILED;
                        return 0.0;
                    }
                    voltage_diff_1 = fabsf(voltage_diff_1);
                    voltage_diff_2 = fabsf(voltage_diff_2);
                    if (voltage_diff_1 < 0.2 || voltage_diff_2 <  0.2) {
                        gcs().send_text(MAV_SEVERITY_INFO, "MotorsTri: cal failed - too small a voltage change");
                        _cal_state = CAL_FAILED;
                        return 0.0;
                    }
                    float voltage_diff_average = (voltage_diff_1 + voltage_diff_2) * 0.5;
                    float voltage_1_error = voltage_diff_1 - voltage_diff_average;
                    float voltage_2_error = voltage_diff_2 - voltage_diff_average;
                    if (voltage_1_error > voltage_diff_average * 0.25 || voltage_2_error > voltage_diff_average * 0.25) {
                        gcs().send_text(MAV_SEVERITY_INFO, "MotorsTri: none linear result");
                        _cal_state = CAL_FAILED;
                        return 0.0f;
                    }

                    // we got this far so the results must be OK
                    _yaw_servo_min_voltage.set_and_save(_voltage_min_temp);
                    _yaw_servo_mid_voltage.set_and_save(_voltage_mid_temp);
                    _yaw_servo_max_voltage.set_and_save(_voltage_max_temp);

                    _cal_state = CAL_SPEED;

                }
            }
            return _yaw_servo_angle_max_deg*100;
        }

        case CAL_SPEED: {
            // move the yaw servo between min and max and measure the time to travel from -75% to + 75% of range
            if (_count == 0) {
                _count = 1;
                _cal_start_ms = now;
                _cal_last_angle = measure_pivot_angle();
                return _yaw_servo_angle_max_deg*100;
            }

            // take 10 readings
            if (_count >= 10) {
                float time = (_reading_sum / (_count - 1)) * 0.001;
                if (is_zero(time)) {
                    gcs().send_text(MAV_SEVERITY_INFO, "MotorsTri: speed cal error");
                    _cal_state = CAL_FAILED;
                    return 0.0;
                }
                float angle = 2*0.75*_yaw_servo_angle_max_deg;
                float speed = angle / time;
                gcs().send_text(MAV_SEVERITY_INFO, "MotorsTri: yaw speed: %.1f",(double)speed);
                _yaw_servo_speed.set_and_save(speed);
                _cal_state = CAL_NONE;
                return 0.0;
            }

            if (now - _cal_start_ms > 2000) {
                gcs().send_text(MAV_SEVERITY_INFO, "MotorsTri: speed cal timeout");
                _cal_state = CAL_FAILED;
                return 0.0;
            }

            // take readings for average
            float angle = measure_pivot_angle();
            if (_count % 2) {
                // going from max to min
                if (_cal_last_angle > 0.75*radians(_yaw_servo_angle_max_deg) && angle < 0.75*radians(_yaw_servo_angle_max_deg)) {
                    _cal_start_ms = now;
                }
                if (_cal_last_angle > -0.75*radians(_yaw_servo_angle_max_deg) && angle < -0.75*radians(_yaw_servo_angle_max_deg)) {
                    _reading_sum += now - _cal_start_ms;
                    _count++;
                }
                _cal_last_angle = angle;
                return -_yaw_servo_angle_max_deg*100;
            } else {
                // going from min to max
                if (_cal_last_angle < -0.75*radians(_yaw_servo_angle_max_deg) && angle > -0.75*radians(_yaw_servo_angle_max_deg)) {
                    _cal_start_ms = now;
                }
                if (_cal_last_angle < 0.75*radians(_yaw_servo_angle_max_deg) && angle > 0.75*radians(_yaw_servo_angle_max_deg)) {
                    _reading_sum += now - _cal_start_ms;
                    _count++;
                }
                _cal_last_angle = angle;
                return _yaw_servo_angle_max_deg*100;
            }
        }

        default: {
            return 0.0;
        }
    }
}
