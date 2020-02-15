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
 *       AP_MotorsMatrixTS.cpp - tailsitters with multicopter motor configuration
 */

#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_MotorsMatrixTS.h"

extern const AP_HAL::HAL& hal;

#define SERVO_OUTPUT_RANGE  4500

// output_armed - sends commands to the motors
// includes new scaling stability patch
void AP_MotorsMatrixTS::output_armed_stabilizing()
{
    if (use_standard_matrix) {
        AP_MotorsMatrix::output_armed_stabilizing();
        return;
    }

    float   roll_thrust;                // roll thrust input value, +/- 1.0
    float   pitch_thrust;               // pitch thrust input value, +/- 1.0
    float   throttle_thrust;            // throttle thrust input value, 0.0 - 1.0
    float   thrust_max = 0.0f;          // highest motor value
    float   thr_adj = 0.0f;             // the difference between the pilot's desired throttle and throttle_thrust_best_rpy

    // apply voltage and air pressure compensation
    const float compensation_gain = get_compensation_gain(); // compensation for battery voltage and altitude
    roll_thrust = (_roll_in + _roll_in_ff) * compensation_gain;
    pitch_thrust = (_pitch_in + _pitch_in_ff) * compensation_gain;
    throttle_thrust = get_throttle() * compensation_gain;

    // sanity check throttle is above zero and below current limited throttle
    if (throttle_thrust <= 0.0f) {
        throttle_thrust = 0.0f;
        limit.throttle_lower = true;
    }
    if (throttle_thrust >= _throttle_thrust_max) {
        throttle_thrust = _throttle_thrust_max;
        limit.throttle_upper = true;
    }

    thrust_max = 0.0f;
    for (int i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            // calculate the thrust outputs for roll and pitch
            _thrust_rpyt_out[i] = throttle_thrust + roll_thrust * _roll_factor[i] + pitch_thrust * _pitch_factor[i];
            if (thrust_max < _thrust_rpyt_out[i]) {
                thrust_max = _thrust_rpyt_out[i];
            }
        }
    }

    // if max thrust is more than one reduce average throttle
    if (thrust_max > 1.0f) {
        thr_adj = 1.0f - thrust_max;
        limit.throttle_upper = true;
        limit.roll = true;
        limit.pitch = true;
        for (int i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                // calculate the thrust outputs for roll and pitch
                _thrust_rpyt_out[i] += thr_adj;
            }
        }
    }

    // compensation_gain can never be zero
    _throttle_out = (throttle_thrust + thr_adj) / compensation_gain;

}

void AP_MotorsMatrixTS::setup_motors(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // remove existing motors
    for (int8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        remove_motor(i);
    }

    bool success = false;
    use_standard_matrix = true;

    switch (frame_class) {

        case MOTOR_FRAME_TRI:
            // frame_type ignored since only one frame type is currently supported
            add_motor(AP_MOTORS_MOT_1,  90, 0, 2);
            add_motor(AP_MOTORS_MOT_2, -90, 0, 4);
            add_motor(AP_MOTORS_MOT_4, 180, 0, 3);
            success = true;
            break;
        case MOTOR_FRAME_QUAD:
            switch (frame_type) {
                case MOTOR_FRAME_TYPE_NYT_PLUS:
                    // motors 1,2 on wings, motors 3,4 on vertical tail/subfin
                    // motors 1,2 are counter-rotating, as are motors 3,4
                    // left wing motor is CW (looking from front)
                    // don't think it matters which of 3,4 is CW
                    add_motor(AP_MOTORS_MOT_1,  90, 0, 2);
                    add_motor(AP_MOTORS_MOT_2, -90, 0, 4);
                    add_motor(AP_MOTORS_MOT_3,   0, 0, 1);
                    add_motor(AP_MOTORS_MOT_4, 180, 0, 3);

                    use_standard_matrix = false;
                    success = true;
                    break;

                case MOTOR_FRAME_TYPE_NYT_X:
                    // PLUS_TS layout rotated 45 degrees about X axis
                    // no differential torque for yaw: wing and fin motors counter-rotating
                    add_motor(AP_MOTORS_MOT_1,   45, 0, 1);
                    add_motor(AP_MOTORS_MOT_2, -135, 0, 3);
                    add_motor(AP_MOTORS_MOT_3,  -45, 0, 4);
                    add_motor(AP_MOTORS_MOT_4,  135, 0, 2);

                    use_standard_matrix = false;
                    success = true;
                    break;
                default:
                    // matrixTS doesn't support the configured frame_type
                    // try to use normal Matrix
                    AP_MotorsMatrix::setup_motors(frame_class, frame_type);
                    return;
            }
            break;
        default:
            // matrixTS doesn't support the configured frame_class
            // try to use normal Matrix
            AP_MotorsMatrix::setup_motors(frame_class, frame_type);
            return;
        } // switch frame_class

    // normalise factors to magnitude 0.5
    normalise_rpy_factors();

    _flags.initialised_ok = success;
}
