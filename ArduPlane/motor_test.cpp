#include "Plane.h"

/*
  mavlink motor test - implements the MAV_CMD_DO_MOTOR_TEST mavlink
                       command so that the quadplane pilot can test an
                       individual motor to ensure proper wiring, rotation.
 */

// motor test definitions
#define MOTOR_TEST_TIMEOUT_MS_MAX       30000   // max timeout is 30 seconds

// motor_test_output - checks for timeout and sends updates to motors objects
#if HAL_QUADPLANE_ENABLED
void QuadPlane::motor_test_output()
{
    // exit immediately if the motor test is not running
    if (!motor_test.running) {
        return;
    }

    // if the order test is ORDER_BOARD then servo must be non-null and configured to continue.
    // So, if it's ever non-null then we should use it.
    SRV_Channel *servo = nullptr;
    if (motor_test.order == MOTOR_TEST_ORDER_BOARD) {
        servo = SRV_Channels::srv_channel(motor_test.seq-1);
        if (servo == nullptr || !servo->valid_function()) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Motor Test: SERVO%d_FUNCTION not configured", (unsigned)motor_test.seq);
            motor_test_stop();
            return;
        }
    }

    // check for test timeout
    uint32_t now = AP_HAL::millis();
    if ((now - motor_test.start_ms) >= motor_test.timeout_ms) {
        if (motor_test.motor_count > 1) {
            if (now - motor_test.start_ms >= motor_test.timeout_ms*1.5) {
                // move onto next motor
                motor_test.seq++;
                motor_test.motor_count--;
                motor_test.start_ms = now;
            } else if (servo != nullptr) { // MOTOR_TEST_ORDER_BOARD
                servo->set_output_pwm(servo->get_output_min());
            } else { // MOTOR_TEST_ORDER_DEFAULT
                // output zero for 0.5s
                motors->output_min();
            }
            return;
        }
        // stop motor test
        motor_test_stop();
        return;
    }
            
    int16_t pwm = 0;   // pwm that will be output to the motors

    // calculate pwm based on throttle type
    const int16_t thr_min_pwm = (servo != nullptr) ? servo->get_output_min() : motors->get_pwm_output_min();
    const int16_t thr_max_pwm = (servo != nullptr) ? servo->get_output_max() : motors->get_pwm_output_max();

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
        pwm = thr_min_pwm + (thr_max_pwm - thr_min_pwm) * plane.get_throttle_input()*0.01f;
        break;

    default:
        motor_test_stop();
        return;
    }

    // sanity check throttle values and turn on motor to specified pwm value
    if (pwm < RC_Channel::RC_MIN_LIMIT_PWM || pwm > RC_Channel::RC_MAX_LIMIT_PWM) {
        motor_test_stop();
    } else if (servo != nullptr) {
        servo->set_output_pwm(pwm);
    } else {
        motors->output_test_seq(motor_test.seq, pwm);
    }
}

// mavlink_motor_test_start - start motor test - spin a single motor at a specified pwm
//  returns MAV_RESULT_ACCEPTED on success, MAV_RESULT_FAILED on failure
MAV_RESULT QuadPlane::mavlink_motor_test_start(mavlink_channel_t chan, uint8_t motor_seq, uint8_t throttle_type,
                                            uint16_t throttle_value, float timeout_sec, uint8_t motor_count, uint8_t motor_test_order)
{
    if (!available() || motors == nullptr) {
        return MAV_RESULT_FAILED;
    }

    if (motors->armed()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Must be disarmed for motor test");
        return MAV_RESULT_FAILED;
    }

    if (motor_test_order != MOTOR_TEST_ORDER_DEFAULT && motor_test_order != MOTOR_TEST_ORDER_BOARD) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Motor Test: Bad test type %u", (unsigned)motor_test_order);
        return MAV_RESULT_FAILED;
    }

    // Check Motor test is allowed
    char failure_msg[50] {};
    if (!motors->motor_test_checks(ARRAY_SIZE(failure_msg), failure_msg)) {
        gcs().send_text(MAV_SEVERITY_CRITICAL,"Motor Test: %s", failure_msg);
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
    motor_test.order = motor_test_order;

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

#endif  // HAL_QUADPLANE_ENABLED
