// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Tracker.h"

/*
 * control_servo_test.pde - GCS controlled servo test mode
 */

/*
 * servo_test_set_servo - sets the yaw or pitch servo pwm directly
 *  servo_num are 1 for yaw, 2 for pitch
 */
bool Tracker::servo_test_set_servo(uint8_t servo_num, uint16_t pwm)
{
    // convert servo_num from 1~2 to 0~1 range
    servo_num--;

    // exit immediately if servo_num is invalid
    if (servo_num != CH_YAW && servo_num != CH_PITCH) {
        return false;
    }

    // ensure we are in servo test mode
    if (control_mode != SERVO_TEST) {
        set_mode(SERVO_TEST);
    }

    // set yaw servo pwm and send output to servo
    if (servo_num == CH_YAW) {
        channel_yaw.radio_out = constrain_int16(pwm, channel_yaw.radio_min, channel_yaw.radio_max);
        channel_yaw.output();
    }

    // set pitch servo pwm and send output to servo
    if (servo_num == CH_PITCH) {
        channel_pitch.radio_out = constrain_int16(pwm, channel_pitch.radio_min, channel_pitch.radio_max);
        channel_pitch.output();
    }

    // return success
    return true;
}
