#include "Plane.h"

/*
  mavlink motor test - implements the MAV_CMD_DO_MOTOR_TEST mavlink
                       command so that the quadplane pilot can test an
                       individual motor to ensure proper wiring, rotation.
 */

// motor test definitions
#define MOTOR_TEST_PWM_MIN              800     // min pwm value accepted by the test
#define MOTOR_TEST_PWM_MAX              2200    // max pwm value accepted by the test
#define MOTOR_TEST_TIMEOUT_MS_MAX       30000   // max timeout is 30 seconds

// motor_test_output - checks for timeout and sends updates to motors objects
void QuadPlane::motor_test_output()
{
    // exit immediately if the motor test is not running
    if (!motor_test.running) {
        return;
    }

    // check for test timeout
    uint32_t now = AP_HAL::millis();
    if ((now - motor_test.start_ms) >= motor_test.timeout_ms) {
        if (motor_test.motor_count > 1) {
            if (now - motor_test.start_ms < motor_test.timeout_ms*1.5) {
                // output zero for 0.5s
                motors->output_min();
            } else {
                // move onto next motor
                motor_test.seq++;
                motor_test.motor_count--;
                motor_test.start_ms = now;
            }
            return;
        }
        // stop motor test
        motor_test_stop();
        return;
    }
            
    int16_t pwm = 0;   // pwm that will be output to the motors

    // calculate pwm based on throttle type
    switch (motor_test.throttle_type) {
    case MOTOR_TEST_THROTTLE_PERCENT:
        // sanity check motor_test.throttle value
        if (motor_test.throttle_value <= 100) {
            pwm = thr_min_pwm + (thr_max_pwm - thr_min_pwm) * (float)motor_test.throttle_value*0.01f;
        }
        break;

    case MOTOR_TEST_THROTTLE_PWM:
        pwm = motor_test.throttle_value;
        break;

    case MOTOR_TEST_THROTTLE_PILOT:
        pwm = thr_min_pwm + (thr_max_pwm - thr_min_pwm) * (float)plane.channel_throttle->get_control_in()*0.01f;
        break;

    default:
        motor_test_stop();
        return;
    }

    // sanity check throttle values
    if (pwm >= MOTOR_TEST_PWM_MIN && pwm <= MOTOR_TEST_PWM_MAX ) {
        // turn on motor to specified pwm vlaue
        motors->output_test(motor_test.seq, pwm);
    } else {
        motor_test_stop();
    }
}

// mavlink_motor_test_start - start motor test - spin a single motor at a specified pwm
//  returns MAV_RESULT_ACCEPTED on success, MAV_RESULT_FAILED on failure
MAV_RESULT QuadPlane::mavlink_motor_test_start(mavlink_channel_t chan, uint8_t motor_seq, uint8_t throttle_type,
                                            uint16_t throttle_value, float timeout_sec, uint8_t motor_count)
{
    if (motors->armed()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Must be disarmed for motor test");
        return MAV_RESULT_FAILED;
    }
    // if test has not started try to start it
    if (!motor_test.running) {
        // start test
        motor_test.running = true;

        // enable and arm motors
        set_armed(true);
        
        // turn on notify leds
        AP_Notify::flags.esc_calibration = true;
    }

    // set timeout
    motor_test.start_ms = AP_HAL::millis();
    motor_test.timeout_ms = MIN(timeout_sec * 1000, MOTOR_TEST_TIMEOUT_MS_MAX);

    // store required output
    motor_test.seq = motor_seq;
    motor_test.throttle_type = throttle_type;
    motor_test.throttle_value = throttle_value;
    motor_test.motor_count = MIN(motor_count, 8);

    // return success
    return MAV_RESULT_ACCEPTED;
}

// motor_test_stop - stops the motor test
void QuadPlane::motor_test_stop()
{
    // exit immediately if the test is not running
    if (!motor_test.running) {
        return;
    }

    // flag test is complete
    motor_test.running = false;

    // disarm motors
    set_armed(false);

    // reset timeout
    motor_test.start_ms = 0;
    motor_test.timeout_ms = 0;

    // turn off notify leds
    AP_Notify::flags.esc_calibration = false;
}
