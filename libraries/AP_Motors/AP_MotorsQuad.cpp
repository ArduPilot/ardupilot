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
        add_motor(AP_MOTORS_MOT_1,   45,  0.7981f,  1);
        add_motor(AP_MOTORS_MOT_2, -135,  1.0000f,  3);
        add_motor(AP_MOTORS_MOT_3,  -45, -0.7981f,  4);
        add_motor(AP_MOTORS_MOT_4,  135, -1.0000f,  2);

    }else if( _flags.frame_orientation == AP_MOTORS_H_FRAME ) {
        // H frame set-up - same as X but motors spin in opposite directiSons
        add_motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1);
        add_motor(AP_MOTORS_MOT_2, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3);
        add_motor(AP_MOTORS_MOT_3,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
        add_motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
    }else if(_flags.frame_orientation == AP_MOTORS_VTAIL_FRAME) {
        /*
            Tested with: Lynxmotion Hunter Vtail 400
            - inverted rear outward blowing motors (at a 40 degree angle)
            - should also work with non-inverted rear outward blowing motors
            - no roll in rear motors
            - no yaw in front motors
            - should fly like some mix between a tricopter and X Quadcopter

            Roll control comes only from the front motors, Yaw control only from the rear motors.
            Roll & Pitch factor is measured by the angle away from the top of the forward axis to each arm.

            Note: if we want the front motors to help with yaw,
                motors 1's yaw factor should be changed to sin(radians(40)).  Where "40" is the vtail angle
                motors 3's yaw factor should be changed to -sin(radians(40))
        */

        add_motor(AP_MOTORS_MOT_1, 60, 60, 0, 1);
        add_motor(AP_MOTORS_MOT_2, 0, -160, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 3);
        add_motor(AP_MOTORS_MOT_3, -60, -60, 0, 4);
        add_motor(AP_MOTORS_MOT_4, 0, 160, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
    } else if (_flags.frame_orientation == AP_MOTORS_ATAIL_FRAME) {
        /*
            The A-Shaped VTail is the exact same as a V-Shaped VTail, with one difference:
            - The Yaw factors are reversed, because the rear motors are facing different directions

            With V-Shaped VTails, the props make a V-Shape when spinning, but with
            A-Shaped VTails, the props make an A-Shape when spinning.
            - Rear thrust on a V-Shaped V-Tail Quad is outward
            - Rear thrust on an A-Shaped V-Tail Quad is inward

            Still functions the same as the V-Shaped VTail mixing below:
            - Yaw control is entirely in the rear motors
            - Roll is is entirely in the front motors
        */
        add_motor(AP_MOTORS_MOT_1, 60, 60, 0, 1);
        add_motor(AP_MOTORS_MOT_2, 0, -160, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3);
        add_motor(AP_MOTORS_MOT_3, -60, -60, 0, 4);
        add_motor(AP_MOTORS_MOT_4, 0, 160, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 2);
    } else if ( _flags.frame_orientation == AP_MOTORS_QUADPLANE ) {
        // quadplane frame set-up, X arrangement on motors 5 to 8
        add_motor(AP_MOTORS_MOT_5,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1);
        add_motor(AP_MOTORS_MOT_6, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3);
        add_motor(AP_MOTORS_MOT_7,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4);
        add_motor(AP_MOTORS_MOT_8,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2);
    }else{
        // X frame set-up
        add_motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1);
        add_motor(AP_MOTORS_MOT_2, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3);
        add_motor(AP_MOTORS_MOT_3,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4);
        add_motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2);
    }
}
