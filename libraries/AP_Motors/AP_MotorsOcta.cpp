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
 *       AP_MotorsOcta.cpp - ArduCopter motors library
 *       Code by RandyMackay. DIYDrones.com
 *
 */

#include "AP_MotorsOcta.h"

// setup_motors - configures the motors for a octa
void AP_MotorsOcta::setup_motors()
{
    // call parent
    AP_MotorsMatrix::setup_motors();

    // hard coded config for supported frames
    if( _flags.frame_orientation == AP_MOTORS_PLUS_FRAME ) {
        // plus frame set-up
        add_motor(AP_MOTORS_MOT_1,    0,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1);
        add_motor(AP_MOTORS_MOT_2,  180,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5);
        add_motor(AP_MOTORS_MOT_3,   45,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
        add_motor(AP_MOTORS_MOT_4,  135,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
        add_motor(AP_MOTORS_MOT_5,  -45,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8);
        add_motor(AP_MOTORS_MOT_6, -135,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6);
        add_motor(AP_MOTORS_MOT_7,  -90,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7);
        add_motor(AP_MOTORS_MOT_8,   90,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3);

    }else if( _flags.frame_orientation == AP_MOTORS_V_FRAME ) {
        // V frame set-up
        add_motor_raw(AP_MOTORS_MOT_1,  1.0,  0.34, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7);
        add_motor_raw(AP_MOTORS_MOT_2, -1.0, -0.32, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3);
        add_motor_raw(AP_MOTORS_MOT_3,  1.0, -0.32, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6);
        add_motor_raw(AP_MOTORS_MOT_4, -0.5,  -1.0, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
        add_motor_raw(AP_MOTORS_MOT_5,  1.0,   1.0, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8);
        add_motor_raw(AP_MOTORS_MOT_6, -1.0,  0.34, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
        add_motor_raw(AP_MOTORS_MOT_7, -1.0,   1.0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1);
        add_motor_raw(AP_MOTORS_MOT_8,  0.5,  -1.0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5);

    }else {
        // X frame set-up
        add_motor(AP_MOTORS_MOT_1,   22.5, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1);
        add_motor(AP_MOTORS_MOT_2, -157.5, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5);
        add_motor(AP_MOTORS_MOT_3,   67.5, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
        add_motor(AP_MOTORS_MOT_4,  157.5, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
        add_motor(AP_MOTORS_MOT_5,  -22.5, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8);
        add_motor(AP_MOTORS_MOT_6, -112.5, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6);
        add_motor(AP_MOTORS_MOT_7,  -67.5, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7);
        add_motor(AP_MOTORS_MOT_8,  112.5, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3);
    }
}
