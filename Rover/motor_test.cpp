#include "Rover.h"

/*
  mavlink motor test - implements the MAV_CMD_DO_MOTOR_TEST mavlink command so that the GCS/pilot can test an individual motor or flaps
                       to ensure proper wiring, rotation.
 */

// motor test definitions
static const int16_t MOTOR_TEST_PWM_MAX = 2200; // max pwm value accepted by the test
static const int16_t MOTOR_TEST_TIMEOUT_MS_MAX = 30000; // max timeout is 30 seconds

static uint32_t motor_test_start_ms = 0;        // system time the motor test began
static uint32_t motor_test_timeout_ms = 0;      // test will timeout this many milliseconds after the motor_test_start_ms
static AP_MotorsUGV::motor_test_order motor_test_instance;              // motor instance number of motor being tested (see AP_MotorsUGV::motor_test_order)
static uint8_t motor_test_throttle_type = 0;    // motor throttle type (0=throttle percentage, 1=PWM, 2=pilot throttle channel pass-through)
static int16_t motor_test_throttle_value = 0;   // throttle to be sent to motor, value depends upon it's type

// motor_test_output - checks for timeout and sends updates to motors objects
void Rover::motor_test_output()
{
    // exit immediately if the motor test is not running
    if (!motor_test) {
        return;
    }

    // check for test timeout
    if ((AP_HAL::millis() - motor_test_start_ms) >= motor_test_timeout_ms) {
        // stop motor test
        motor_test_stop();
    } else {
        bool test_result = false;
        // calculate  based on throttle type
        switch (motor_test_throttle_type) {
            case MOTOR_TEST_THROTTLE_PERCENT:
                test_result = g2.motors.output_test_pct(motor_test_instance, motor_test_throttle_value);
                break;

            case MOTOR_TEST_THROTTLE_PWM:
                test_result = g2.motors.output_test_pwm(motor_test_instance, motor_test_throttle_value);
                break;

            case MOTOR_TEST_THROTTLE_PILOT:
                if (motor_test_instance == AP_MotorsUGV::MOTOR_TEST_STEERING) {
                    test_result = g2.motors.output_test_pct(motor_test_instance, channel_steer->norm_input_dz() * 100.0f);
                } else {
                    test_result = g2.motors.output_test_pct(motor_test_instance, channel_throttle->get_control_in());
                }
                break;

            default:
                // do nothing
                return;
        }
        // stop motor test on failure
        if (!test_result) {
            motor_test_stop();
        }
    }
}

// mavlink_motor_test_check - perform checks before motor tests can begin
// return true if tests can continue, false if not
bool Rover::mavlink_motor_test_check(const GCS_MAVLINK &gcs_chan, bool check_rc, AP_MotorsUGV::motor_test_order motor_seq, uint8_t throttle_type, int16_t throttle_value)
{
    // check board has initialised
    if (!initialised) {
        gcs_chan.send_text(MAV_SEVERITY_CRITICAL, "Motor Test: Board initialising");
        return false;
    }

    // check rc has been calibrated
    if (check_rc && !arming.rc_calibration_checks(true)) {
        gcs_chan.send_text(MAV_SEVERITY_CRITICAL, "Motor Test: RC not calibrated");
        return false;
    }

    // check if safety switch has been pushed
    if (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
        gcs_chan.send_text(MAV_SEVERITY_CRITICAL, "Motor Test: Safety switch");
        return false;
    }

    // check motor_seq
    if (motor_seq > AP_MotorsUGV::MOTOR_TEST_THROTTLE_RIGHT) {
        gcs_chan.send_text(MAV_SEVERITY_CRITICAL, "Motor Test: invalid motor (%d)", (int)motor_seq);
        return false;
    }

    // check throttle type
    if (throttle_type > MOTOR_TEST_THROTTLE_PILOT) {
        gcs_chan.send_text(MAV_SEVERITY_CRITICAL, "Motor Test: invalid throttle type: %d", (int)throttle_type);
        return false;
    }

    // check throttle value
    if (throttle_type == MOTOR_TEST_THROTTLE_PWM && throttle_value > MOTOR_TEST_PWM_MAX) {
        gcs_chan.send_text(MAV_SEVERITY_CRITICAL, "Motor Test: pwm (%d) too high", (int)throttle_value);
        return false;
    }
    if (throttle_type == MOTOR_TEST_THROTTLE_PERCENT && throttle_value > 100) {
        gcs_chan.send_text(MAV_SEVERITY_CRITICAL, "Motor Test: percentage (%d) too high", (int)throttle_value);
        return false;
    }

    // if we got this far the check was successful and the motor test can continue
    return true;
}

// mavlink_motor_test_start - start motor test - spin a single motor at a specified pwm
// returns MAV_RESULT_ACCEPTED on success, MAV_RESULT_FAILED on failure
MAV_RESULT Rover::mavlink_motor_test_start(const GCS_MAVLINK &gcs_chan, AP_MotorsUGV::motor_test_order motor_instance, uint8_t throttle_type, int16_t throttle_value, float timeout_sec)
{
    // if test has not started try to start it
    if (!motor_test) {
        /* perform checks that it is ok to start test
           The RC calibrated check can be skipped if direct pwm is
           suppliedo
        */
        if (!mavlink_motor_test_check(gcs_chan, throttle_type != 1, motor_instance, throttle_type, throttle_value)) {
            return MAV_RESULT_FAILED;
        } else {
            // start test
            motor_test = true;

            // arm motors
            if (!arming.is_armed()) {
                if (!arming.arm(AP_Arming::Method::MOTORTEST)) {
                    return MAV_RESULT_FAILED;
                }
            }

            // disable failsafes
            g.fs_gcs_enabled = 0;
            g.fs_throttle_enabled = 0;
            g.fs_crash_check = 0;

            // turn on notify leds
            AP_Notify::flags.esc_calibration = true;
        }
    }

    // set timeout
    motor_test_start_ms = AP_HAL::millis();
    motor_test_timeout_ms = MIN(timeout_sec * 1000, MOTOR_TEST_TIMEOUT_MS_MAX);

    // store required output
    motor_test_instance = motor_instance;
    motor_test_throttle_type = throttle_type;
    motor_test_throttle_value = throttle_value;

    // return success
    return MAV_RESULT_ACCEPTED;
}

// motor_test_stop - stops the motor test
void Rover::motor_test_stop()
{
    // exit immediately if the test is not running
    if (!motor_test) {
        return;
    }

    // disarm motors
    AP::arming().disarm(AP_Arming::Method::MOTORTEST);

    // reset timeout
    motor_test_start_ms = 0;
    motor_test_timeout_ms = 0;

    // re-enable failsafes
    g.fs_gcs_enabled.load();
    g.fs_throttle_enabled.load();
    g.fs_crash_check.load();

    // turn off notify leds
    AP_Notify::flags.esc_calibration = false;

    // flag test is complete
    motor_test = false;
}
