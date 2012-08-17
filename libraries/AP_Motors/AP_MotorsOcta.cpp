/*
 *       AP_MotorsOcta.cpp - ArduCopter motors library
 *       Code by RandyMackay. DIYDrones.com
 *
 *       This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 */

#include "AP_MotorsOcta.h"

// setup_motors - configures the motors for a octa
void AP_MotorsOcta::setup_motors()
{
    // call parent
    AP_MotorsMatrix::setup_motors();

    // hard coded config for supported frames
    if( _frame_orientation == AP_MOTORS_PLUS_FRAME ) {
        // plus frame set-up
        add_motor(AP_MOTORS_MOT_1,    0,  AP_MOTORS_MATRIX_MOTOR_CW,  AP_MOTORS_MOT_2, 1);
        add_motor(AP_MOTORS_MOT_2,  180,  AP_MOTORS_MATRIX_MOTOR_CW,  AP_MOTORS_MOT_1, 5);
        add_motor(AP_MOTORS_MOT_3,   45,  AP_MOTORS_MATRIX_MOTOR_CCW, AP_MOTORS_MOT_6, 2);
        add_motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_MOTOR_CCW, AP_MOTORS_MOT_5, 4);
        add_motor(AP_MOTORS_MOT_5,  -45, AP_MOTORS_MATRIX_MOTOR_CCW, AP_MOTORS_MOT_4, 8);
        add_motor(AP_MOTORS_MOT_6, -135, AP_MOTORS_MATRIX_MOTOR_CCW, AP_MOTORS_MOT_3, 6);
        add_motor(AP_MOTORS_MOT_7,  -90, AP_MOTORS_MATRIX_MOTOR_CW,  AP_MOTORS_MOT_8, 7);
        add_motor(AP_MOTORS_MOT_8,   90, AP_MOTORS_MATRIX_MOTOR_CW,  AP_MOTORS_MOT_7, 3);

    }else if( _frame_orientation == AP_MOTORS_V_FRAME ) {
        // V frame set-up
        add_motor_raw(AP_MOTORS_MOT_1,  1.0,  0.34, AP_MOTORS_MATRIX_MOTOR_CW,  AP_MOTORS_MOT_2, 7);
        add_motor_raw(AP_MOTORS_MOT_2, -1.0, -0.32, AP_MOTORS_MATRIX_MOTOR_CW,  AP_MOTORS_MOT_1, 3);
        add_motor_raw(AP_MOTORS_MOT_3,  1.0, -0.32, AP_MOTORS_MATRIX_MOTOR_CCW, AP_MOTORS_MOT_6, 6);
        add_motor_raw(AP_MOTORS_MOT_4, -0.5,  -1.0, AP_MOTORS_MATRIX_MOTOR_CCW, AP_MOTORS_MOT_5, 4);
        add_motor_raw(AP_MOTORS_MOT_5,  1.0,   1.0, AP_MOTORS_MATRIX_MOTOR_CCW, AP_MOTORS_MOT_4, 8);
        add_motor_raw(AP_MOTORS_MOT_6, -1.0,  0.34, AP_MOTORS_MATRIX_MOTOR_CCW, AP_MOTORS_MOT_3, 2);
        add_motor_raw(AP_MOTORS_MOT_7, -1.0,   1.0, AP_MOTORS_MATRIX_MOTOR_CW,  AP_MOTORS_MOT_8, 1);
        add_motor_raw(AP_MOTORS_MOT_8,  0.5,  -1.0, AP_MOTORS_MATRIX_MOTOR_CW,  AP_MOTORS_MOT_7, 5);

    }else {
        // X frame set-up
        add_motor(AP_MOTORS_MOT_1,   22.5, AP_MOTORS_MATRIX_MOTOR_CW,  AP_MOTORS_MOT_2, 1);
        add_motor(AP_MOTORS_MOT_2, -157.5, AP_MOTORS_MATRIX_MOTOR_CW,  AP_MOTORS_MOT_1, 5);
        add_motor(AP_MOTORS_MOT_3,   67.5, AP_MOTORS_MATRIX_MOTOR_CCW, AP_MOTORS_MOT_6, 2);
        add_motor(AP_MOTORS_MOT_4,  157.5, AP_MOTORS_MATRIX_MOTOR_CCW, AP_MOTORS_MOT_5, 4);
        add_motor(AP_MOTORS_MOT_5,  -22.5, AP_MOTORS_MATRIX_MOTOR_CCW, AP_MOTORS_MOT_4, 8);
        add_motor(AP_MOTORS_MOT_6, -112.5, AP_MOTORS_MATRIX_MOTOR_CCW, AP_MOTORS_MOT_3, 6);
        add_motor(AP_MOTORS_MOT_7,  -67.5, AP_MOTORS_MATRIX_MOTOR_CW,  AP_MOTORS_MOT_8, 7);
        add_motor(AP_MOTORS_MOT_8,  112.5, AP_MOTORS_MATRIX_MOTOR_CW,  AP_MOTORS_MOT_7, 3);
    }
}