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

// output a thrust to all motors that match a given motor mask. This
// is used to control motors enabled for forward flight. Thrust is in
// the range 0 to 1
void AP_MotorsMatrixTS::output_motor_mask(float thrust, uint8_t mask, float rudder_dt)
{
    for (uint8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            int16_t motor_out;
            if (mask & (1U<<i)) {
                /*
                    apply rudder mixing differential thrust
                    copter frame roll is plane frame yaw as this only
                    apples to either tilted motors or tailsitters
                */
                float diff_thrust = get_roll_factor(i) * rudder_dt * 0.5f;
                thrust = constrain_float(thrust + diff_thrust, 0.0f, 1.0f);
                motor_out = get_pwm_output_min() + (get_pwm_output_max()-get_pwm_output_min()) * thrust;
            } else {
                motor_out = get_pwm_output_min();
            }
            rc_write(i, motor_out);
        }
    }
}

void AP_MotorsMatrixTS::output_to_motors()
{
    // calls calc_thrust_to_pwm(_thrust_rpyt_out[i]) for each enabled motor
    AP_MotorsMatrix::output_to_motors();

    // also actuate control surfaces
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron,  -_yaw_in * SERVO_OUTPUT_RANGE);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, _pitch_in * SERVO_OUTPUT_RANGE);
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, _roll_in * SERVO_OUTPUT_RANGE);
}

void AP_MotorsMatrixTS::output_armed_stabilizing()
{
    // calculates thrust values _thrust_rpyt_out
    AP_MotorsMatrix::output_armed_stabilizing();
}

void AP_MotorsMatrixTS::setup_motors(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // remove existing motors
    for (int8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        remove_motor(i);
    }

    bool success = false;

    switch (frame_class) {

        case MOTOR_FRAME_TRI:
            // frame_type ignored since only one frame type is currently supported
            add_motor(AP_MOTORS_MOT_1,  90, 0, 2);
            add_motor(AP_MOTORS_MOT_2, -90, 0, 4);
            add_motor(AP_MOTORS_MOT_4, 180, 0,  3);
            success = true;
            break;
        case MOTOR_FRAME_QUAD:
            switch (frame_type) {
                case MOTOR_FRAME_TYPE_PLUS:
                    // motors 1,2 on wings, motors 3,4 on vertical tail/subfin
                    // motors 1,2 are counter-rotating, as are motors 3,4
                    // left wing motor is CW (looking from front)
                    // don't think it matters which of 3,4 is CW
                    add_motor(AP_MOTORS_MOT_1,  90, 0, 2);
                    add_motor(AP_MOTORS_MOT_2, -90, 0, 4);
                    add_motor(AP_MOTORS_MOT_3,   0, 0,  1);
                    add_motor(AP_MOTORS_MOT_4, 180, 0,  3);
                    success = true;
                    break;
                case MOTOR_FRAME_TYPE_X:
                    // PLUS_TS layout rotated 45 degrees about X axis
                    add_motor(AP_MOTORS_MOT_1,   45, 0, 1);
                    add_motor(AP_MOTORS_MOT_2, -135, 0, 3);
                    add_motor(AP_MOTORS_MOT_3,  -45, 0,  4);
                    add_motor(AP_MOTORS_MOT_4,  135, 0,  2);
                    success = true;
                    break;
                default:
                    // matrixTS doesn't support the configured frame_type
                    break;
            }

        default:
            // matrixTS doesn't support the configured frame_class
            break;
        } // switch frame_class

    // normalise factors to magnitude 0.5
    normalise_rpy_factors();

    _flags.initialised_ok = success;
}
