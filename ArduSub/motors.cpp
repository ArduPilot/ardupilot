#include "Sub.h"

// enable_motor_output() - enable and output lowest possible value to motors
void Sub::enable_motor_output()
{
    motors.output_min();
}

// motors_output - send output to motors library which will adjust and send to ESCs and servos
void Sub::motors_output()
{
    // Motor detection mode controls the thrusters directly
    if (control_mode == Mode::Number::MOTOR_DETECT){
        return;
    }
    // check if we are performing the motor test
    if (ap.motor_test) {
        verify_motor_test();
    } else {
        motors.set_interlock(true);
        auto &srv = AP::srv();
        srv.cork();
        SRV_Channels::calc_pwm();
        SRV_Channels::output_ch_all();
        motors.output();
        srv.push();
    }
}

// Initialize new style motor test
// Perform checks to see if it is ok to begin the motor test
// Returns true if motor test has begun
bool Sub::init_motor_test()
{
    uint32_t tnow = AP_HAL::millis();

    // Ten second cooldown period required with no do_set_motor requests required
    // after failure.
    if (tnow < last_do_motor_test_fail_ms + 10000 && last_do_motor_test_fail_ms > 0) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "10 second cooldown required after motor test");
        return false;
    }

    // check if safety switch has been pushed
    if (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
        gcs().send_text(MAV_SEVERITY_CRITICAL,"Disarm hardware safety switch before testing motors.");
        return false;
    }

    // Make sure we are on the ground
    if (!motors.armed()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Arm motors before testing motors.");
        return false;
    }

    enable_motor_output(); // set all motor outputs to zero
    ap.motor_test = true;

    return true;
}

// Verify new style motor test
// The motor test will fail if the interval between received
// MAV_CMD_DO_SET_MOTOR requests exceeds a timeout period
// Returns true if it is ok to proceed with new style motor test
bool Sub::verify_motor_test()
{
    bool pass = true;

    // Require at least 2 Hz incoming do_set_motor requests
    if (AP_HAL::millis() > last_do_motor_test_ms + 500) {
        gcs().send_text(MAV_SEVERITY_INFO, "Motor test timed out!");
        pass = false;
    }

    if (!pass) {
        ap.motor_test = false;
        AP::arming().disarm(AP_Arming::Method::MOTORTEST);
        last_do_motor_test_fail_ms = AP_HAL::millis();
        return false;
    }

    return true;
}

bool Sub::handle_do_motor_test(mavlink_command_int_t command) {
    last_do_motor_test_ms = AP_HAL::millis();

    // If we are not already testing motors, initialize test
    static uint32_t tLastInitializationFailed = 0;
    if(!ap.motor_test) {
        // Do not allow initializations attempt under 2 seconds
        // If one fails, we need to give the user time to fix the issue
        // instead of spamming error messages
        if (AP_HAL::millis() > (tLastInitializationFailed + 2000)) {
            if (!init_motor_test()) {
                gcs().send_text(MAV_SEVERITY_WARNING, "motor test initialization failed!");
                tLastInitializationFailed = AP_HAL::millis();
                return false; // init fail
            }
        } else {
            return false;
        }
    }

    float motor_number = command.param1;
    float throttle_type = command.param2;
    float throttle = command.param3;
    // float timeout_s = command.param4; // not used
    // float motor_count = command.param5; // not used
    const uint32_t test_type = command.y;

    if (test_type != MOTOR_TEST_ORDER_BOARD) {
        gcs().send_text(MAV_SEVERITY_WARNING, "bad test type %0.2f", (double)test_type);
        return false; // test type not supported here
    }

    if (is_equal(throttle_type, (float)MOTOR_TEST_THROTTLE_PILOT)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "bad throttle type %0.2f", (double)throttle_type);

        return false; // throttle type not supported here
    }

    if (is_equal(throttle_type, (float)MOTOR_TEST_THROTTLE_PWM)) {
        return motors.output_test_num(motor_number, throttle); // true if motor output is set
    }

    if (is_equal(throttle_type, (float)MOTOR_TEST_THROTTLE_PERCENT)) {
        throttle = constrain_float(throttle, 0.0f, 100.0f);
        throttle = channel_throttle->get_radio_min() + throttle * 0.01f * (channel_throttle->get_radio_max() - channel_throttle->get_radio_min());
        return motors.output_test_num(motor_number, throttle); // true if motor output is set
    }

    return false;
}


// translate wpnav roll/pitch outputs to lateral/forward
void Sub::translate_wpnav_rp(float &lateral_out, float &forward_out)
{
    // get roll and pitch targets in centidegrees
    int32_t lateral = wp_nav.get_roll();
    int32_t forward = -wp_nav.get_pitch(); // output is reversed

    // constrain target forward/lateral values
    // The outputs of wp_nav.get_roll and get_pitch should already be constrained to these values
    lateral = constrain_int16(lateral, -aparm.angle_max, aparm.angle_max);
    forward = constrain_int16(forward, -aparm.angle_max, aparm.angle_max);

    // Normalize
    lateral_out = (float)lateral/(float)aparm.angle_max;
    forward_out = (float)forward/(float)aparm.angle_max;
}

// translate wpnav roll/pitch outputs to lateral/forward
void Sub::translate_circle_nav_rp(float &lateral_out, float &forward_out)
{
    // get roll and pitch targets in centidegrees
    int32_t lateral = circle_nav.get_roll_cd();
    int32_t forward = -circle_nav.get_pitch_cd(); // output is reversed

    // constrain target forward/lateral values
    lateral = constrain_int16(lateral, -aparm.angle_max, aparm.angle_max);
    forward = constrain_int16(forward, -aparm.angle_max, aparm.angle_max);

    // Normalize
    lateral_out = (float)lateral/(float)aparm.angle_max;
    forward_out = (float)forward/(float)aparm.angle_max;
}

// translate pos_control roll/pitch outputs to lateral/forward
void Sub::translate_pos_control_rp(float &lateral_out, float &forward_out)
{
    // get roll and pitch targets in centidegrees
    int32_t lateral = pos_control.get_roll_cd();
    int32_t forward = -pos_control.get_pitch_cd(); // output is reversed

    // constrain target forward/lateral values
    lateral = constrain_int16(lateral, -aparm.angle_max, aparm.angle_max);
    forward = constrain_int16(forward, -aparm.angle_max, aparm.angle_max);

    // Normalize
    lateral_out = (float)lateral/(float)aparm.angle_max;
    forward_out = (float)forward/(float)aparm.angle_max;
}

// State feedback position controller helper
// Returns true if state feedback was used, false otherwise
bool Sub::run_state_feedback_position_controller(const Vector3f& pos_target_ned,
                                                  const Vector3f& vel_target_ned)
{
    // Only use state feedback if enabled
    if (g2.sf_params.enable < 3) {
        return false;
    }

    // Check if we have valid position estimate
    if (!position_ok()) {
        return false;
    }

    // Get current position and velocity in NED frame
    Vector3p pos_actual_ned_d;
    if (!ahrs.get_relative_position_NED_origin(pos_actual_ned_d)) {
        return false;
    }
    Vector3f pos_actual_ned = pos_actual_ned_d.tofloat();  // Convert to float

    // Get current velocity in NED frame
    Vector3f vel_actual_ned;
    if (!ahrs.get_velocity_NED(vel_actual_ned)) {
        vel_actual_ned.zero();  // Use zero velocity if estimate not available
    }

    // Run state feedback position controller
    // Returns [vertical_thrust, roll_torque, pitch_torque, yaw_torque]
    Vector4f control = attitude_control.position_controller_run_state_feedback(
        g2.sf_params,
        pos_target_ned,
        pos_actual_ned,
        vel_target_ned,
        vel_actual_ned
    );

    // The position controller outputs:
    // control.x = vertical thrust (normalized -1 to 1, positive up)
    // control.y = roll torque (normalized)
    // control.z = pitch torque (normalized)
    // control.w = yaw torque (normalized)

    // Set vertical thrust (convert from -1..1 to 0..1 for motors)
    float throttle_out = constrain_float(control.x * 0.5f + 0.5f, 0.0f, 1.0f);
    attitude_control.set_throttle_out(throttle_out, true, g.throttle_filt);

    // Convert torques to motor outputs
    motors.set_roll(control.y);
    motors.set_pitch(control.z);
    motors.set_yaw(control.w);

    // For 6DOF control, we need to convert desired attitude to forward/lateral
    // This is a simplified approach - full 6DOF would directly control thrusters
    // For now, use zero forward/lateral (hovering in place)
    motors.set_forward(0.0f);
    motors.set_lateral(0.0f);

    return true;
}
