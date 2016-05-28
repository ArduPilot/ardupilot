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
 *       AP_MotorsY6.cpp - ArduCopter motors library
 *       Code by RandyMackay. DIYDrones.com
 *
 */

#include "AP_MotorsY6.h"

// setup_motors - configures the motors for a hexa
void AP_MotorsY6::setup_motors()
{
    // call parent
    AP_MotorsMatrix::setup_motors();

    if (_flags.frame_orientation >= AP_MOTORS_NEW_PLUS_FRAME) {
        // Y6 motor definition with all top motors spinning clockwise, all bottom motors counter clockwise
        add_motor_raw(AP_MOTORS_MOT_1, -1.0,  0.500, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1);
        add_motor_raw(AP_MOTORS_MOT_2, -1.0,  0.500, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
        add_motor_raw(AP_MOTORS_MOT_3,  0.0, -1.000, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3);
        add_motor_raw(AP_MOTORS_MOT_4,  0.0, -1.000, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4);
        add_motor_raw(AP_MOTORS_MOT_5,  1.0,  0.500, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5);
        add_motor_raw(AP_MOTORS_MOT_6,  1.0,  0.500, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6);
    }else{
        // original Y6 motor definition
        add_motor_raw(AP_MOTORS_MOT_1, -1.0,  0.666, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
        add_motor_raw(AP_MOTORS_MOT_2,  1.0,  0.666, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5);
        add_motor_raw(AP_MOTORS_MOT_3,  1.0,  0.666, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6);
        add_motor_raw(AP_MOTORS_MOT_4,  0.0, -1.333, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4);
        add_motor_raw(AP_MOTORS_MOT_5, -1.0,  0.666, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1);
        add_motor_raw(AP_MOTORS_MOT_6,  0.0, -1.333, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3);
    }
}
