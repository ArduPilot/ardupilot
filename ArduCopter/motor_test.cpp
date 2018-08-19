#include "Copter.h"

/*
  mavlink motor test - implements the MAV_CMD_DO_MOTOR_TEST mavlink command so that the GCS/pilot can test an individual motor or flaps
                       to ensure proper wiring, rotation.
 */

// motor test definitions
#define MOTOR_TEST_PWM_MIN              800     // min pwm value accepted by the test
#define MOTOR_TEST_PWM_MAX              2200    // max pwm value accepted by the test
#define MOTOR_TEST_TIMEOUT_SEC          600     // max timeout is 10 minutes (600 seconds)

static uint32_t motor_test_start_ms;        // system time the motor test began
static uint32_t motor_test_timeout_ms;      // test will timeout this many milliseconds after the motor_test_start_ms
static uint8_t motor_test_seq;              // motor sequence number of motor being tested
static uint8_t motor_test_count;            // number of motors to test
static uint8_t motor_test_throttle_type;    // motor throttle type (0=throttle percentage, 1=PWM, 2=pilot throttle channel pass-through)
static uint16_t motor_test_throttle_value;  // throttle to be sent to motor, value depends upon it's type

// motor_test_output - checks for timeout and sends updates to motors objects
void Copter::motor_test_output()
{
    // exit immediately if the motor test is not running
    if (!ap.motor_test) {
        return;
    }

    // check for test timeout
    uint32_t now = AP_HAL::millis();
    if ((now - motor_test_start_ms) >= motor_test_timeout_ms) {
        if (motor_test_count > 1) {
            if (now - motor_test_start_ms < motor_test_timeout_ms*1.5) {
                // output zero for 50% of the test time
                motors->output_min();
            } else {
                // move onto next motor
                motor_test_seq++;
                motor_test_count--;
                motor_test_start_ms = now;
                if (!motors->armed()) {
                    motors->armed(true);
                }
            }
            return;
        }
        // stop motor test
        motor_test_stop();
    } else {
        int16_t pwm = 0;   // pwm that will be output to the motors

        // calculate pwm based on throttle type
        switch (motor_test_throttle_type) {

            case MOTOR_TEST_COMPASS_CAL:
                compass.set_voltage(battery.voltage());
                compass.per_motor_calibration_update();
                FALLTHROUGH;

            case MOTOR_TEST_THROTTLE_PERCENT:
                // sanity check motor_test_throttle value
#if FRAME_CONFIG != HELI_FRAME
                if (motor_test_throttle_value <= 100) {
                    int16_t pwm_min = motors->get_pwm_output_min();
                    int16_t pwm_max = motors->get_pwm_output_max();
                    pwm = pwm_min + (pwm_max - pwm_min) * (float)motor_test_throttle_value/100.0f;
                }
#endif
                break;

            case MOTOR_TEST_THROTTLE_PWM:
                pwm = motor_test_throttle_value;
                break;

            case MOTOR_TEST_THROTTLE_PILOT:
                pwm = channel_throttle->get_radio_in();
                break;

            default:
                motor_test_stop();
                return;
        }

        // sanity check throttle values
        if (pwm >= MOTOR_TEST_PWM_MIN && pwm <= MOTOR_TEST_PWM_MAX ) {
            // turn on motor to specified pwm vlaue
            motors->output_test_seq(motor_test_seq, pwm);
        } else {
            motor_test_stop();
        }
    }
}

// mavlink_motor_test_check - perform checks before motor tests can begin
//  return true if tests can continue, false if not
bool Copter::mavlink_motor_test_check(mavlink_channel_t chan, bool check_rc)
{
    GCS_MAVLINK_Copter &gcs_chan = gcs().chan(chan-MAVLINK_COMM_0);

    // check board has initialised
    if (!ap.initialised) {
        gcs_chan.send_text(MAV_SEVERITY_CRITICAL,"Motor Test: Board initialising");
        return false;
    }

    // check rc has been calibrated
    if (check_rc && !arming.rc_calibration_checks(true)) {
        gcs_chan.send_text(MAV_SEVERITY_CRITICAL,"Motor Test: RC not calibrated");
        return false;
    }

    // ensure we are landed
    if (!ap.land_complete) {
        gcs_chan.send_text(MAV_SEVERITY_CRITICAL,"Motor Test: vehicle not landed");
        return false;
    }

    // check if safety switch has been pushed
    if (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
        gcs_chan.send_text(MAV_SEVERITY_CRITICAL,"Motor Test: Safety switch");
        return false;
    }

    // if we got this far the check was successful and the motor test can continue
    return true;
}

// mavlink_motor_test_start - start motor test - spin a single motor at a specified pwm
//  returns MAV_RESULT_ACCEPTED on success, MAV_RESULT_FAILED on failure
MAV_RESULT Copter::mavlink_motor_test_start(mavlink_channel_t chan, uint8_t motor_seq, uint8_t throttle_type, uint16_t throttle_value,
                                         float timeout_sec, uint8_t motor_count)
{
    if (motor_count == 0) {
        motor_count = 1;
    }
    // if test has not started try to start it
    if (!ap.motor_test) {
        gcs().send_text(MAV_SEVERITY_INFO, "starting motor test");

        /* perform checks that it is ok to start test
           The RC calibrated check can be skipped if direct pwm is
           supplied
        */
        if (!mavlink_motor_test_check(chan, throttle_type != 1)) {
            return MAV_RESULT_FAILED;
        } else {
            // start test
            ap.motor_test = true;

            // enable and arm motors
            if (!motors->armed()) {
                init_rc_out();
                enable_motor_output();
                motors->armed(true);
            }

            // disable throttle and gps failsafe
            g.failsafe_throttle = FS_THR_DISABLED;
            g.failsafe_gcs = FS_GCS_DISABLED;
            g.fs_ekf_action = 0;

            // turn on notify leds
            AP_Notify::flags.esc_calibration = true;
        }
    }

    // set timeout
    motor_test_start_ms = AP_HAL::millis();
    motor_test_timeout_ms = MIN(timeout_sec, MOTOR_TEST_TIMEOUT_SEC) * 1000;

    // store required output
    motor_test_seq = motor_seq;
    motor_test_count = motor_count;
    motor_test_throttle_type = throttle_type;
    motor_test_throttle_value = throttle_value;

    if (motor_test_throttle_type == MOTOR_TEST_COMPASS_CAL) {
        compass.per_motor_calibration_start();
    }            

    // return success
    return MAV_RESULT_ACCEPTED;
}

// motor_test_stop - stops the motor test
void Copter::motor_test_stop()
{
    // exit immediately if the test is not running
    if (!ap.motor_test) {
        return;
    }

    gcs().send_text(MAV_SEVERITY_INFO, "finished motor test");    

    // flag test is complete
    ap.motor_test = false;

    // disarm motors
    motors->armed(false);

    // reset timeout
    motor_test_start_ms = 0;
    motor_test_timeout_ms = 0;

    // re-enable failsafes
    g.failsafe_throttle.load();
    g.failsafe_gcs.load();
    g.fs_ekf_action.load();

    if (motor_test_throttle_type == MOTOR_TEST_COMPASS_CAL) {
        compass.per_motor_calibration_end();
    }

    // turn off notify leds
    AP_Notify::flags.esc_calibration = false;
}
