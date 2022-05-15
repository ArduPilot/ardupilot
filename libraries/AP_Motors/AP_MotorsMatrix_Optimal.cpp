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

#include "AP_MotorsMatrix_Optimal.h"

#if HAL_HAVE_HARDWARE_DOUBLE

#include <AP_HAL/AP_HAL.h>
#include <AP_InternalError/AP_InternalError.h>

extern const AP_HAL::HAL& hal;

void AP_MotorsMatrix_Optimal::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    AP_MotorsMatrix::init(frame_class, frame_type);
    set_initialised_ok(false);

    if (frame_class == MOTOR_FRAME_SCRIPTING_MATRIX) {
        // Scripting frame class not supported
        // Scripting is setup to use the AP_MotorsMatrix singleton, which wont exist if were here.
        // So there is no way for scripting to setup the motors, once we fix that it should work well...
        return;
    }

    if (_heap != nullptr) {
        // copter can re-init motors when disarmed if FRAME_* params are changed
        // Don't allow as a change in heap size would be needed.
        return;
    }

    // count number of motors
    num_motors = 0;
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            num_motors++;
        }
    }

    // grab some memory for matrix operations, very approximate guestimate at usage
    // 4096 -> 36864
    _heap = hal.util->allocate_heap_memory(256 * num_motors * num_motors);
    if (_heap == nullptr) {
        return;
    }

    uint8_t num_constraints = (num_motors*2) + 1;

    // convert motor factors to matrix format
    motor_factors.init(num_motors,4);
    for (uint8_t i = 0; i < num_motors; i++) {
        motor_factors(i, 0, _roll_factor[i]);
        motor_factors(i, 1, _pitch_factor[i]);
        motor_factors(i, 2, _yaw_factor[i]);
        motor_factors(i, 3, _throttle_factor[i]);
    }

    // Weighting vector defines the relative weighting of roll, pitch, yaw and throttle
    // may want to change this on the fly in the future, but in that case we can no longer pre-compute the hessian
    w.init(1, 4);
    w(0, 0, 15.0); // roll
    w(0, 1, 15.0); // pitch
    w(0, 2, 15.0); // yaw
    w(0, 3, 1.0);  // throttle

    // setup hessian matrix
    H.init(num_motors, num_motors);
    H = motor_factors.per_element_mult_mv(w) * motor_factors.transposed();

    // setup constraints matrix
    A.init(num_motors, num_constraints);
    for (uint8_t i = 0; i < num_motors; i++) {
        A(i, 0, -_throttle_factor[i] / num_motors); // average throttle constraint
        A(i, 1+i, -1.0);                            // upper throttle limit
        A(i, 1+num_motors+i,  1.0);                 // lower throttle limit
    }

    // re-scale weights to save runtime calculation
    w(0, 3, w(0, 3) * num_motors);
    w *= -1;

    // size runtime matrices
    f.init(num_motors,1);
    b.init(num_constraints,1);
    input.init(4,1);
    out.init(num_motors,1);

    // check for matrix fail
    set_initialised_ok(!error_flag);
}


// output - sends commands to the motors, 
void AP_MotorsMatrix_Optimal::output_armed_stabilizing()
{
    if (!initialised_ok()) {
        AP_MotorsMatrix::output_armed_stabilizing();
        return;
    }

    // apply voltage and air pressure compensation
    const float compensation_gain = get_compensation_gain(); // compensation for battery voltage and altitude
    const float roll_thrust = (_roll_in + _roll_in_ff) * compensation_gain;
    const float pitch_thrust = (_pitch_in + _pitch_in_ff) * compensation_gain;
    const float yaw_thrust = (_yaw_in + _yaw_in_ff) * compensation_gain;
    float throttle_thrust = get_throttle() * compensation_gain;
    float throttle_avg_max = _throttle_avg_max * compensation_gain;

    // If thrust boost is active then do not limit maximum thrust
    const float throttle_thrust_max = _thrust_boost_ratio + (1.0 - _thrust_boost_ratio) * _throttle_thrust_max * compensation_gain;

    // sanity check throttle is above zero and below current limited throttle
    if (throttle_thrust <= 0.0) {
        throttle_thrust = 0.0;
        limit.throttle_lower = true;
    }
    if (throttle_thrust >= throttle_thrust_max) {
        throttle_thrust = throttle_thrust_max;
        limit.throttle_upper = true;
    }

    // ensure that throttle_avg_max is between the input throttle and the maximum throttle
    throttle_avg_max = constrain_float(throttle_avg_max, throttle_thrust, throttle_thrust_max);

    // set roll, pitch, yaw and throttle input value
    input(0, 0, roll_thrust);
    input(1, 0, pitch_thrust);
    input(2, 0, yaw_thrust);
    input(3, 0, throttle_thrust);

    // input matrix
    f = motor_factors.per_element_mult_mv(w) * input;

    // constraints, average throttle, 1 and 0
    b(0, 0, -throttle_avg_max);
    for (uint8_t i = 0; i < num_motors; i++) {
        b(1+i, 0, -1.0); // we could set this to 0 if we detect a failed motor
        b(1+num_motors+i, 0, 0.0);
    }

    // the clever bit
    out = interior_point_solve(H, f, A, b, _heap, error_flag);

    if (error_flag) {
        set_initialised_ok(false);
        // something bad happended, probably ran out of memory
        // call the base class ouput function
        AP_MotorsMatrix::output_armed_stabilizing();
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
        return;
    }

    // copy to motor outputs
    for (uint8_t i = 0; i < num_motors; i++) {
        _thrust_rpyt_out[i] = out(i,0);
    }

    // note that this does not do anything with the limit flags, we could set a threshold on desired vs achieved output.
    // hopefully limit flags will be triggered much less often in anycase...
}

#endif // HAL_HAVE_HARDWARE_DOUBLE
