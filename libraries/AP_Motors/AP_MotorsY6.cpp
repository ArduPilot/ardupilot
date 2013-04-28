/*
 *       AP_MotorsY6.cpp - ArduCopter motors library
 *       Code by RandyMackay. DIYDrones.com
 *
 *       This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 */

#include "AP_MotorsY6.h"

// setup_motors - configures the motors for a hexa
void AP_MotorsY6::setup_motors()
{
    // call parent
    AP_MotorsMatrix::setup_motors();

    // MultiWii set-up
    add_motor_raw(AP_MOTORS_MOT_1, -1.0,  0.666, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
    add_motor_raw(AP_MOTORS_MOT_2,  1.0,  0.666, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5);
    add_motor_raw(AP_MOTORS_MOT_3,  1.0,  0.666, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6);
    add_motor_raw(AP_MOTORS_MOT_4,  0.0, -1.333, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4);
    add_motor_raw(AP_MOTORS_MOT_5, -1.0,  0.666, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1);
    add_motor_raw(AP_MOTORS_MOT_6,  0.0, -1.333, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3);
}