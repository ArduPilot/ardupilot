/*
 *       AP_MotorsOctaQuad.cpp - ArduCopter motors library
 *       Code by RandyMackay. DIYDrones.com
 *
 *       This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 */

#include "AP_MotorsOctaQuad.h"

// setup_motors - configures the motors for a octa
void AP_MotorsOctaQuad::setup_motors()
{
    // call parent
    AP_MotorsMatrix::setup_motors();

    // hard coded config for supported frames
    if( _frame_orientation == AP_MOTORS_PLUS_FRAME ) {
        // plus frame set-up
        add_motor(AP_MOTORS_MOT_1,    0, AP_MOTORS_MATRIX_MOTOR_CCW, AP_MOTORS_MOT_3, 1);
        add_motor(AP_MOTORS_MOT_2,  -90, AP_MOTORS_MATRIX_MOTOR_CW,  AP_MOTORS_MOT_4, 7);
        add_motor(AP_MOTORS_MOT_3,  180, AP_MOTORS_MATRIX_MOTOR_CCW, AP_MOTORS_MOT_1, 5);
        add_motor(AP_MOTORS_MOT_4,   90, AP_MOTORS_MATRIX_MOTOR_CW,  AP_MOTORS_MOT_2, 4);
        add_motor(AP_MOTORS_MOT_5,  -90, AP_MOTORS_MATRIX_MOTOR_CCW, AP_MOTORS_MOT_7, 8);
        add_motor(AP_MOTORS_MOT_6,    0, AP_MOTORS_MATRIX_MOTOR_CW,  AP_MOTORS_MOT_8, 2);
        add_motor(AP_MOTORS_MOT_7,   90, AP_MOTORS_MATRIX_MOTOR_CCW, AP_MOTORS_MOT_5, 4);
        add_motor(AP_MOTORS_MOT_8,  180, AP_MOTORS_MATRIX_MOTOR_CW,  AP_MOTORS_MOT_6, 6);
    }else{
        // X frame set-up
        add_motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_MOTOR_CCW, AP_MOTORS_MOT_3, 1);
        add_motor(AP_MOTORS_MOT_2,  -45, AP_MOTORS_MATRIX_MOTOR_CW,  AP_MOTORS_MOT_4, 7);
        add_motor(AP_MOTORS_MOT_3, -135, AP_MOTORS_MATRIX_MOTOR_CCW, AP_MOTORS_MOT_1, 5);
        add_motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_MOTOR_CW,  AP_MOTORS_MOT_2, 4);
        add_motor(AP_MOTORS_MOT_5,  -45, AP_MOTORS_MATRIX_MOTOR_CCW, AP_MOTORS_MOT_7, 8);
        add_motor(AP_MOTORS_MOT_6,   45, AP_MOTORS_MATRIX_MOTOR_CW,  AP_MOTORS_MOT_8, 2);
        add_motor(AP_MOTORS_MOT_7,  135, AP_MOTORS_MATRIX_MOTOR_CCW, AP_MOTORS_MOT_5, 4);
        add_motor(AP_MOTORS_MOT_8, -135, AP_MOTORS_MATRIX_MOTOR_CW,  AP_MOTORS_MOT_6, 6);
    }
}