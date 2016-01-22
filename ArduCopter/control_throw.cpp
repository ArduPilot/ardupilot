/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"


// throw_init - initialise throw controller
bool Copter::throw_init(bool ignore_checks)
{
#if FRAME_CONFIG == HELI_FRAME
    // do not allow helis to use throw to start
    return false;
#endif

    // do not enter the mode when already armed or when flying
    if (motors.armed()) {
        return false;
    }

    // this mode needs a position reference
    if (position_ok()) {
        throw_state = Throw_Disarmed;
        return true;
    } else {
        return false;
    }
}

bool Copter::throw_motor_stop()
{
    return control_mode == THROW && (throw_state == Throw_Disarmed || throw_state == Throw_Detecting) && g.throw_motor_start != 1;
}

// runs the throw to start controller
// should be called at 100hz or more
void Copter::throw_run()
{
    /* Throw State Machine
    Throw_Disarmed - motors are off
    Throw_Detecting -  motors are on and we are waiting for the throw
    Throw_Uprighting - the throw has been detected and the copter is being uprighted
    Throw_HgtStabilise - the copter is kept level and  height is stabilised about the target height
    Throw_PosHold - the copter is kept at a constant position and height
    */

    bool motors_unlocked = (!ap.using_interlock || ap.motor_interlock_switch) && !ap.motor_emergency_stop;

    if (!motors.armed() || (throw_state == Throw_Detecting && !motors_unlocked)) {
        // switch to Throw_Disarmed if the vehicle is disarmed, or if the motors become locked during detection
        throw_state = Throw_Disarmed;
    } else if (throw_state == Throw_Disarmed && motors.armed() && motors_unlocked) {
        // switch to Throw_Detecting once the motors are armed and unlocked
        gcs_send_text(MAV_SEVERITY_INFO,"waiting for throw");
        throw_state = Throw_Detecting;
    } else if (throw_state == Throw_Detecting && throw_detected()){
        gcs_send_text(MAV_SEVERITY_INFO,"throw detected - uprighting");
        throw_state = Throw_Uprighting;

        // Cancel the waiting for throw tone sequence
        AP_Notify::flags.waiting_for_throw = false;

        // reset the interlock
        motors.set_interlock(true);
    } else if (throw_state == Throw_Uprighting && throw_attitude_good()) {
        gcs_send_text(MAV_SEVERITY_INFO,"uprighted - controlling height");
        throw_state = Throw_HgtStabilise;

        // initialize vertical speed and acceleration limits
        // use brake mode values for rapid response
        pos_control.set_speed_z(BRAKE_MODE_SPEED_Z, BRAKE_MODE_SPEED_Z);
        pos_control.set_accel_z(BRAKE_MODE_DECEL_RATE);

        // initialise the demanded height to 3m above the throw height
        // we want to rapidly clear surrounding obstacles
        pos_control.set_alt_target(inertial_nav.get_altitude() + 300);

        // set the initial velocity of the height controller demand to the measured velocity if it is going up
        // if it is going down, set it to zero to enforce a very hard stop
        pos_control.set_desired_velocity_z(fmaxf(inertial_nav.get_velocity_z(),0.0f));

        // Set the auto_arm status to true to avoid a possible automatic disarm caused by selection of an auto mode with throttle at minimum
        set_auto_armed(true);

    } else if (throw_state == Throw_HgtStabilise && throw_height_good()) {
        gcs_send_text(MAV_SEVERITY_INFO,"height achieved - controlling position");
        throw_state = Throw_PosHold;

        // initialise the loiter target to the curent position and velocity
        wp_nav.init_loiter_target();

        // Set the auto_arm status to true to avoid a possible automatic disarm caused by selection of an auto mode with throttle at minimum
        set_auto_armed(true);
    }

    // Throw State Processing
    switch (throw_state) {

    case Throw_Disarmed:

        // demand zero throttle (motors will be stopped anyway) and continually reset the attitude controller
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        motors.slow_start(true);

        break;

    case Throw_Detecting:

        // Hold throttle at zero during the throw and continually reset the attitude controller
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);

        // Play the waiting for throw tone sequence to alert the user
        AP_Notify::flags.waiting_for_throw = true;

        break;

    case Throw_Uprighting:

        // demand a level roll/pitch attitude with zero yaw rate
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f, 0.0f);

        // output 50% throttle and turn off angle boost to maximise righting moment
        attitude_control.set_throttle_out(500, false, g.throttle_filt);

        break;

    case Throw_HgtStabilise:

        // call attitude controller
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f, 0.0f);

        // call height controller
        pos_control.set_alt_target_from_climb_rate_ff(0.0f, G_Dt, false);
        pos_control.update_z_controller();

        break;

    case Throw_PosHold:

        // run loiter controller
        wp_nav.update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);

        // call attitude controller
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), 0.0f);

        // call height controller
        pos_control.set_alt_target_from_climb_rate_ff(0.0f, G_Dt, false);
        pos_control.update_z_controller();

        break;
    }
}

bool Copter::throw_detected()
{
    // Check that we have a valid navigation solution
    nav_filter_status filt_status = inertial_nav.get_filter_status();
    if (!filt_status.flags.attitude || !filt_status.flags.horiz_pos_abs || !filt_status.flags.vert_pos) {
        return false;
    }

    // Check for high speed (note get_inertial_nav methods use a cm length scale)
    bool high_speed = inertial_nav.get_velocity().length() > 500.0f;

    // check for upwards trajectory
    bool gaining_height = inertial_nav.get_velocity().z > 50.0f;

    // Check the vertical acceleraton is greater than 0.25g
    bool free_falling = ahrs.get_accel_ef().z > -0.25 * GRAVITY_MSS;

    // Check if the accel length is < 1.0g indicating that any throw action is complete and the copter has been released
    bool no_throw_action = ins.get_accel().length() < 1.0f * GRAVITY_MSS;

    // High velocity or free-fall combined with incresing height indicate a possible throw release
    bool possible_throw_detected = (free_falling || high_speed) && gaining_height && no_throw_action;

    // Record time and vertical velocity when we detect the possible throw
    if (possible_throw_detected && ((AP_HAL::millis() - throw_free_fall_start_ms) > 500)) {
        throw_free_fall_start_ms = AP_HAL::millis();
        throw_free_fall_start_velz = inertial_nav.get_velocity().z;
    }

    // Once a possible throw condition has been detected, we check for 2.5 m/s of downwards velocity change in less than 0.5 seconds to confirm
    bool throw_condition_confirmed = ((AP_HAL::millis() - throw_free_fall_start_ms < 500) && ((inertial_nav.get_velocity().z - throw_free_fall_start_velz) < -250.0f));

    // start motors and enter the control mode if we are in continuous freefall
    if (throw_condition_confirmed) {
        return true;
    } else {
        return false;
    }
}

bool Copter::throw_attitude_good()
{
    // Check that we have uprighted the copter
    const Matrix3f &rotMat = ahrs.get_rotation_body_to_ned();
    bool is_upright = (rotMat.c.z > 0.866f);
    return is_upright;
}

bool Copter::throw_height_good()
{
    // Check that we are no more than 0.5m below the demanded height
    return (pos_control.get_alt_error() < 50.0f);
}

