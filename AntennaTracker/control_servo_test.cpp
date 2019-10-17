#include "Tracker.h"

/*
 * GCS controlled servo test mode
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
        set_mode(SERVO_TEST, ModeReason::SERVOTEST);
    }

    // set yaw servo pwm and send output to servo
    if (servo_num == CH_YAW) {
        SRV_Channels::set_output_pwm(SRV_Channel::k_tracker_yaw, pwm);
        SRV_Channels::constrain_pwm(SRV_Channel::k_tracker_yaw);
    }

    // set pitch servo pwm and send output to servo
    if (servo_num == CH_PITCH) {
        SRV_Channels::set_output_pwm(SRV_Channel::k_tracker_pitch, pwm);
        SRV_Channels::constrain_pwm(SRV_Channel::k_tracker_pitch);
    }

    SRV_Channels::calc_pwm();
    SRV_Channels::output_ch_all();
    
    // return success
    return true;
}
