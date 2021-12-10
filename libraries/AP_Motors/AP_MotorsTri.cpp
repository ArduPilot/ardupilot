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
#include <AP_Vehicle/AP_Vehicle_Type.h>

#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

#include "AP_MotorsTri.h"

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
        case SpoolState::SHUT_DOWN:
            // sends minimum values out to the motors
            rc_write(AP_MOTORS_MOT_1, output_to_pwm(0));
            rc_write(AP_MOTORS_MOT_2, output_to_pwm(0));
            rc_write(AP_MOTORS_MOT_4, output_to_pwm(0));
            rc_write_angle(AP_MOTORS_CH_TRI_YAW, 0);
            break;
        case SpoolState::GROUND_IDLE:
            // sends output to motors when armed but not flying
            set_actuator_with_slew(_actuator[1], actuator_spin_up_to_ground_idle());
            set_actuator_with_slew(_actuator[2], actuator_spin_up_to_ground_idle());
            set_actuator_with_slew(_actuator[4], actuator_spin_up_to_ground_idle());
            rc_write(AP_MOTORS_MOT_1, output_to_pwm(_actuator[1]));
            rc_write(AP_MOTORS_MOT_2, output_to_pwm(_actuator[2]));
            rc_write(AP_MOTORS_MOT_4, output_to_pwm(_actuator[4]));
            rc_write_angle(AP_MOTORS_CH_TRI_YAW, 0);
            break;
        case SpoolState::SPOOLING_UP:
        case SpoolState::THROTTLE_UNLIMITED:
        case SpoolState::SPOOLING_DOWN:
            // set motor output based on thrust requests
            set_actuator_with_slew(_actuator[1], thrust_to_actuator(_thrust_right));
            set_actuator_with_slew(_actuator[2], thrust_to_actuator(_thrust_left));
            set_actuator_with_slew(_actuator[4], thrust_to_actuator(_thrust_rear));
            rc_write(AP_MOTORS_MOT_1, output_to_pwm(_actuator[1]));
            rc_write(AP_MOTORS_MOT_2, output_to_pwm(_actuator[2]));
            rc_write(AP_MOTORS_MOT_4, output_to_pwm(_actuator[4]));
            rc_write_angle(AP_MOTORS_CH_TRI_YAW, degrees(_pivot_angle)*100);
            break;
    }
}

// get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint32_t AP_MotorsTri::get_motor_mask()
{
    // tri copter uses channels 1,2,4 and 7
    uint32_t motor_mask = (1U << AP_MOTORS_MOT_1) |
                          (1U << AP_MOTORS_MOT_2) |
                          (1U << AP_MOTORS_MOT_4);
    uint32_t mask = motor_mask_to_srv_channel_mask(motor_mask);

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

    // calculate angle of yaw pivot
    _pivot_angle = safe_asin(yaw_thrust);
    if (fabsf(_pivot_angle) > radians(_yaw_servo_angle_max_deg)) {
        limit.yaw = true;
        _pivot_angle = constrain_float(_pivot_angle, -radians(_yaw_servo_angle_max_deg), radians(_yaw_servo_angle_max_deg));
    }

    float pivot_thrust_max = cosf(_pivot_angle);
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
    _thrust_rear = _thrust_rear / cosf(_pivot_angle);

    // constrain all outputs to 0.0f to 1.0f
    // test code should be run with these lines commented out as they should not do anything
    _thrust_right = constrain_float(_thrust_right, 0.0f, 1.0f);
    _thrust_left = constrain_float(_thrust_left, 0.0f, 1.0f);
    _thrust_rear = constrain_float(_thrust_rear, 0.0f, 1.0f);
}

// output_test_seq - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsTri::_output_test_seq(uint8_t motor_seq, int16_t pwm)
{
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
