#include "Sub.h"
#include "stdio.h"

/*
 * control_motordetect.cpp - init and run calls for motordetect flightmode;
 *
 *  This mode pulses all thrusters to detect if they need to be reversed.
 *  This still requires that the user has the correct frame selected and the motors
 *  are connected to the correct ESCs.
 *
 *  For each motor:
 *      wait until vehicle is stopped for > 500ms
 *      apply throttle up for 500ms
 *      If results are good:
 *          save direction and try the next motor.
 *      else
 *          wait until vehicle is stopped for > 500ms
 *          apply throttle down for 500ms
 *          If results are good
 *              save direction and try the next motor.
 *          If results are bad
 *              Abort!
 */

namespace {

    // controller states
    enum test_state {
        STANDBY,
        SETTLING,
        THRUSTING,
        DETECTING,
        DONE
    };

    enum direction {
        UP = 1,
        DOWN = -1
    };

    static uint32_t settling_timer;
    static uint32_t thrusting_timer;
    static uint8_t md_state;
    static uint8_t current_motor;
    static int16_t current_direction;
}

bool ModeMotordetect::init(bool ignore_checks)
{
    current_motor = 0;
    md_state = STANDBY;
    current_direction = UP;
    return true;
}

void ModeMotordetect::run()
{
    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // Force all motors to stop
        for(uint8_t i=0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motors.motor_is_enabled(i)) {
                motors.output_test_num(i, 1500);
            }
        }
        md_state = STANDBY;
        return;
    }

    switch(md_state) {
    // Motor detection is not running, set it up to start.
    case STANDBY:
        current_direction = UP;
        current_motor = 0;
        settling_timer = AP_HAL::millis();
        md_state = SETTLING;
        break;

    // Wait until sub stays for 500ms not spinning and leveled.
    case SETTLING:
        // Force all motors to stop
        for (uint8_t i=0; i <AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motors.motor_is_enabled(i)) {
                motors.output_test_num(i, 1500);
            }
        }
        // wait until gyro product is under a certain(experimental) threshold
        if ((ahrs.get_gyro()*ahrs.get_gyro()) > 0.01) {
            settling_timer = AP_HAL::millis();
        }
        // then wait 500ms more
        if (AP_HAL::millis() > (settling_timer + 500)) {
            md_state = THRUSTING;
            thrusting_timer = AP_HAL::millis();
        }

        break;

    // Thrusts motor for 500ms
    case THRUSTING:
        if (AP_HAL::millis() < (thrusting_timer + 500)) {
            if (!motors.output_test_num(current_motor, 1500 + 300*current_direction)) {
                md_state = DONE;
            };

        } else {
            md_state = DETECTING;
        }
        break;

    // Checks the result of thrusting the motor.
    // Starts again at the other direction if unable to get a good reading.
    // Fails if it is the second reading and it is still not good.
    // Set things up to test the next motor if the reading is good.
    case DETECTING:
    {
        // This logic results in a vector such as (1, -1, 0)
        // TODO: make these thresholds parameters
        Vector3f gyro = ahrs.get_gyro();
        bool roll_up = gyro.x > 0.4;
        bool roll_down = gyro.x < -0.4;
        int roll = (int(roll_up) - int(roll_down))*current_direction;

        bool pitch_up = gyro.y > 0.4;
        bool pitch_down = gyro.y < -0.4;
        int pitch = (int(pitch_up) - int(pitch_down))*current_direction;

        bool yaw_up = gyro.z > 0.5;
        bool yaw_down = gyro.z < -0.5;
        int yaw = (+int(yaw_up) - int(yaw_down))*current_direction;

        Vector3f directions(roll, pitch, yaw);
        // Good read, not inverted
        if (directions == motors.get_motor_angular_factors(current_motor)) {
            gcs().send_text(MAV_SEVERITY_INFO, "Thruster %d is ok!", current_motor + 1);
        }
        // Good read, inverted
        else if (-directions == motors.get_motor_angular_factors(current_motor)) {
            gcs().send_text(MAV_SEVERITY_INFO, "Thruster %d is reversed! Saving it!", current_motor + 1);
            motors.set_reversed(current_motor, true);
        }
        // Bad read!
        else {
            gcs().send_text(MAV_SEVERITY_INFO, "Bad thrust read, trying to push the other way...");
            // If we got here, we couldn't identify anything that made sense.
            // Let's try pushing the thruster the other way, maybe we are in too shallow waters or hit something
            if (current_direction == DOWN) {
                // The reading for the second direction was also bad, we failed.
                gcs().send_text(MAV_SEVERITY_WARNING, "Failed! Please check Thruster %d and frame setup!", current_motor + 1);
                md_state = DONE;
                break;
            }
            current_direction = DOWN;
            md_state = SETTLING;
            break;
        }
        // If we got here, we have a decent motor reading
        md_state = SETTLING;
        // Test the next motor, if it exists
        current_motor++;
        current_direction = UP;
        if (!motors.motor_is_enabled(current_motor)) {
            md_state = DONE;
            gcs().send_text(MAV_SEVERITY_WARNING, "Motor direction detection is complete.");
        }
        break;
    }
    case DONE:
        set_mode(sub.prev_control_mode, ModeReason::MISSION_END);
        sub.arming.disarm(AP_Arming::Method::MOTORDETECTDONE);
        break;
    }
}
