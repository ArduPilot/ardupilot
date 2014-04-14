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
 *       AP_MotorsQuad.cpp - ArduCopter motors library
 *       Code by RandyMackay. DIYDrones.com
 *
 */

#include "AP_MotorsQuad.h"

// setup_motors - configures the motors for a quad
void AP_MotorsQuad::setup_motors()
{
    // call parent
    AP_MotorsMatrix::setup_motors();

    // hard coded config for supported frames
    if( _flags.frame_orientation == AP_MOTORS_PLUS_FRAME ) {
        // plus frame set-up
        add_motor(AP_MOTORS_MOT_1,  90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
        add_motor(AP_MOTORS_MOT_2, -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
        add_motor(AP_MOTORS_MOT_3,   0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1);
        add_motor(AP_MOTORS_MOT_4, 180, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3);

    }else if( _flags.frame_orientation == AP_MOTORS_V_FRAME ) {
        // V frame set-up
        add_motor(AP_MOTORS_MOT_1,   45,  0.7981,  1);
        add_motor(AP_MOTORS_MOT_2, -135,  1.0000,  3);
        add_motor(AP_MOTORS_MOT_3,  -45, -0.7981,  4);
        add_motor(AP_MOTORS_MOT_4,  135, -1.0000,  2);

    }else if( _flags.frame_orientation == AP_MOTORS_H_FRAME ) {
        // H frame set-up - same as X but motors spin in opposite directiSons
        add_motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1);
        add_motor(AP_MOTORS_MOT_2, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3);
        add_motor(AP_MOTORS_MOT_3,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
        add_motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
    } else if ( _flags.frame_orientation == AP_MOTORS_V_TAIL_FRAME ) {

      /********************************
       Lynxmotion Hunter Vtail 400/500
      ********************************/

      /**
        use add_motor_raw because not all of the propellors are as involved 
        in pitch / roll as other props. 
        - Front props control only roll
        - Rear props control only yaw

        add_motor_raw(
          moter number,
          roll factor,
          pitch factor,
          yaw factor,
          testing order
        )

        roll factor is measured by the angle perpendicular to that of the prop arm to the roll axis (x)
        pitch factor is measured by the angle perpendicular to the prop arm to the pitch axis (y)
        testing order is a clockwise path around the vehicle


        assumptions:
                             20      20
         \      /          3_____________1
          \    /                  |
           \  /                   |
        40  \/  40            20  |  20
           Tail                  / \
                                2   4


        All angles measured from their closest axis

      */
      // these are for if we want the front props to help with yaw
      // int v_tail_angle = 40;
      // float v_tail_thrust = sin(v_tail_angle);

      add_motor_raw(AP_MOTORS_MOT_1,      // front right
        cosf(radians(150)),               // 70 degrees right of roll axis
        cosf(radians(-70)),                // 20 degrees up of pitch axis
        0,                                // no yaw
        1);
      add_motor_raw(AP_MOTORS_MOT_2,      // back right
        0,                                // no roll
        cosf(radians(-160)),               // 70 degrees down of pitch axis
        AP_MOTORS_MATRIX_YAW_FACTOR_CW,  // full yaw
        3);
      add_motor_raw(AP_MOTORS_MOT_3,      // front left
        cosf(radians(20)),                // 70 degrees left of roll axis
        cosf(radians(70)),               // 20 degrees up of pitch axis
        0,                                // no yaw
        4);
      add_motor_raw(AP_MOTORS_MOT_4,      // back left
        0,                                // no roll
        cosf(radians(160)),              // 70 degrees down of pitch axis
        AP_MOTORS_MATRIX_YAW_FACTOR_CCW,   // full yaw
        2);



    }else{
        // X frame set-up
        add_motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1);
        add_motor(AP_MOTORS_MOT_2, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3);
        add_motor(AP_MOTORS_MOT_3,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4);
        add_motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2);
    }
}
