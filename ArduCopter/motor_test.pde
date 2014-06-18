// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
  mavlink motor test - implements the MAV_CMD_DO_MOTOR_TEST mavlink command so that the GCS/pilot can test an individual motor or flaps
                       to ensure proper wiring, rotation.
 */

// motor test definitions
#define MOTOR_TEST_PWM_MIN              800     // min pwm value accepted by the test
#define MOTOR_TEST_PWM_MAX              2200    // max pwm value accepted by the test
#define MOTOR_TEST_TIMEOUT_MS_MAX       30000   // max timeout is 30 seconds

static uint32_t motor_test_start_ms = 0;        // system time the motor test began
static uint32_t motor_test_timeout_ms = 0;      // test will timeout this many milliseconds after the motor_test_start_ms
static uint8_t motor_test_seq = 0;              // motor sequence number of motor being tested
static uint8_t motor_test_throttle_type = 0;    // motor throttle type (0=throttle percentage, 1=PWM, 2=pilot throttle channel pass-through)
static uint16_t motor_test_throttle_value = 0;  // throttle to be sent to motor, value depends upon it's type

// motor_test_output - checks for timeout and sends updates to motors objects
static void motor_test_output()
{
    // exit immediately if the motor test is not running
    if (!ap.motor_test) {
        return;
    }

    // check for test timeout
    if ((hal.scheduler->millis() - motor_test_start_ms) >= motor_test_timeout_ms) {
        // stop motor test
        motor_test_stop();
    } else {
        int16_t pwm = 0;   // pwm that will be output to the motors

        // calculate pwm based on throttle type
        switch (motor_test_throttle_type) {

            case MOTOR_TEST_THROTTLE_PERCENT:
                // sanity check motor_test_throttle value
                if (motor_test_throttle_value <= 100) {
                    pwm = g.rc_3.radio_min + (g.rc_3.radio_max - g.rc_3.radio_min) * (float)motor_test_throttle_value/100.0f;
                }
                break;

            case MOTOR_TEST_THROTTLE_PWM:
                pwm = motor_test_throttle_value;
                break;

            case MOTOR_TEST_THROTTLE_PILOT:
                pwm = g.rc_3.radio_in;
                break;

            default:
                motor_test_stop();
                return;
                break;
        }

        // sanity check throttle values
        if (pwm >= MOTOR_TEST_PWM_MIN && pwm <= MOTOR_TEST_PWM_MAX ) {
            // turn on motor to specified pwm vlaue
            motors.output_test(motor_test_seq, pwm);
        } else {
            motor_test_stop();
        }
    }
}

// mavlink_motor_test_check - perform checks before motor tests can begin
//  return true if tests can continue, false if not
static bool mavlink_motor_test_check(mavlink_channel_t chan)
{
    // check rc has been calibrated
    pre_arm_rc_checks();
    if(!ap.pre_arm_rc_check) {
        gcs[chan-MAVLINK_COMM_0].send_text_P(SEVERITY_HIGH,PSTR("Motor Test: RC not calibrated"));
        return false;
    }

    // ensure we are landed
    if (!ap.land_complete) {
        gcs[chan-MAVLINK_COMM_0].send_text_P(SEVERITY_HIGH,PSTR("Motor Test: vehicle not landed"));
        return false;
    }

    // check if safety switch has been pushed
    if (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
        gcs[chan-MAVLINK_COMM_0].send_text_P(SEVERITY_HIGH,PSTR("Motor Test: Safety Switch"));
        return false;
    }

    // if we got this far the check was successful and the motor test can continue
    return true;
}

// mavlink_motor_test_start - start motor test - spin a single motor at a specified pwm
//  returns MAV_RESULT_ACCEPTED on success, MAV_RESULT_FAILED on failure
static uint8_t mavlink_motor_test_start(mavlink_channel_t chan, uint8_t motor_seq, uint8_t throttle_type, uint16_t throttle_value, float timeout_sec)
{
    // debug
    cliSerial->printf_P(PSTR("\nMotTest Seq:%d TT:%d Thr:%d TimOut:%4.2f"),(int)motor_seq, (int)throttle_type, (int)throttle_value, (float)timeout_sec);

    // if test has not started try to start it
    if (!ap.motor_test) {
        // perform checks that it is ok to start test
        if (!mavlink_motor_test_check(chan)) {
            return MAV_RESULT_FAILED;
        } else {
            // start test
            ap.motor_test = true;

            // enable and arm motors
            if (!motors.armed()) {
                init_rc_out();
                output_min();
                motors.armed(true);
            }

            // disable throttle, battery and gps failsafe
            g.failsafe_throttle = FS_THR_DISABLED;
            g.failsafe_battery_enabled = FS_BATT_DISABLED;
            g.failsafe_gps_enabled = FS_GPS_DISABLED;
            g.failsafe_gcs = FS_GCS_DISABLED;

            // turn on notify leds
            AP_Notify::flags.esc_calibration = true;
        }
    }

    // set timeout
    motor_test_start_ms = hal.scheduler->millis();
    motor_test_timeout_ms = min(timeout_sec * 1000, MOTOR_TEST_TIMEOUT_MS_MAX);

    // store required output
    motor_test_seq = motor_seq;
    motor_test_throttle_type = throttle_type;
    motor_test_throttle_value = throttle_value;

    // return success
    return MAV_RESULT_ACCEPTED;
}

// motor_test_stop - stops the motor test
static void motor_test_stop()
{
    // exit immediately if the test is not running
    if (!ap.motor_test) {
        return;
    }

    // flag test is complete
    ap.motor_test = false;

    // disarm motors
    motors.armed(false);

    // reset timeout
    motor_test_start_ms = 0;
    motor_test_timeout_ms = 0;

    // re-enable failsafes
    g.failsafe_throttle.load();
    g.failsafe_battery_enabled.load();
    g.failsafe_gps_enabled.load();
    g.failsafe_gcs.load();

    // turn off notify leds
    AP_Notify::flags.esc_calibration = false;
}
