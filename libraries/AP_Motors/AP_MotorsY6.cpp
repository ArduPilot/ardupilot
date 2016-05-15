// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 *  Copyright (c) BirdsEyeView Aerobotics, LLC, 2016.
 *
 *  This program is free software: you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 3 as published
 *  by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 *  Public License version 3 for more details.
 *
 *  You should have received a copy of the GNU General Public License version
 *  3 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  All APM Project credits from the original work are kept intact below as a
 *  courtesy.
 *
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

    // call parent
    AP_MotorsMatrix::setup_motors();

    //BEV hardcoding in the FireFLY6 motor arrangement
    add_motor_raw(AP_MOTORS_MOT_3,  0.0, -1.000, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1); //aft top (ccw)
    add_motor_raw(AP_MOTORS_MOT_4, -1.0,  0.500, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2); //front right top (ccw)
    add_motor_raw(AP_MOTORS_MOT_5,  1.0,  0.500, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3); //front left top (ccw)
    add_motor_raw(AP_MOTORS_MOT_6,  0.0, -1.000, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4); //aft bottom (cw)
    add_motor_raw(AP_MOTORS_MOT_7, -1.0,  0.500, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5); //front right bottom (cw)
    add_motor_raw(AP_MOTORS_MOT_8,  1.0,  0.500, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6); //front left bottom (cw)
}
