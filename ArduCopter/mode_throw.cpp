#include "Copter.h"


// throw_init - initialise throw controller
bool Copter::ModeThrow::init(bool ignore_checks)
{
#if FRAME_CONFIG == HELI_FRAME
    // do not allow helis to use throw to start
    return false;
#endif

    // do not enter the mode when already armed or when flying
    if (motors->armed()) {
        return false;
    }

    // init state
    stage = Throw_Disarmed;
    nextmode_attempted = false;

    return true;
}

// runs the throw to start controller
// should be called at 100hz or more
void Copter::ModeThrow::run()
{
    /* Throw State Machine
    Throw_Disarmed - motors are off
    Throw_Detecting -  motors are on and we are waiting for the throw
    Throw_Uprighting - the throw has been detected and the copter is being uprighted
    Throw_HgtStabilise - the copter is kept level and  height is stabilised about the target height
    Throw_PosHold - the copter is kept at a constant position and height
    */

    // initialize smoothing gain
    attitude_control->set_smoothing_gain(get_smoothing_gain());

    // Don't enter THROW mode if interlock will prevent motors running
    if (!motors->armed() && motors->get_interlock()) {
        // state machine entry is always from a disarmed state
        stage = Throw_Disarmed;

    } else if (stage == Throw_Disarmed && motors->armed()) {
        gcs().send_text(MAV_SEVERITY_INFO,"waiting for throw");
        stage = Throw_Detecting;

    } else if (stage == Throw_Detecting && throw_detected()){
        gcs().send_text(MAV_SEVERITY_INFO,"throw detected - uprighting");
        stage = Throw_Uprighting;

        // Cancel the waiting for throw tone sequence
        AP_Notify::flags.waiting_for_throw = false;

    } else if (stage == Throw_Uprighting && throw_attitude_good()) {
        gcs().send_text(MAV_SEVERITY_INFO,"uprighted - controlling height");
        stage = Throw_HgtStabilise;

        // initialize vertical speed and acceleration limits
        // use brake mode values for rapid response
        pos_control->set_speed_z(BRAKE_MODE_SPEED_Z, BRAKE_MODE_SPEED_Z);
        pos_control->set_accel_z(BRAKE_MODE_DECEL_RATE);

        // initialise the demanded height to 3m above the throw height
        // we want to rapidly clear surrounding obstacles
        if (g2.throw_type == ThrowType_Drop) {
            pos_control->set_alt_target(inertial_nav.get_altitude() - 100);
        } else {
            pos_control->set_alt_target(inertial_nav.get_altitude() + 300);
        }

        // set the initial velocity of the height controller demand to the measured velocity if it is going up
        // if it is going down, set it to zero to enforce a very hard stop
        pos_control->set_desired_velocity_z(fmaxf(inertial_nav.get_velocity_z(),0.0f));

        // Set the auto_arm status to true to avoid a possible automatic disarm caused by selection of an auto mode with throttle at minimum
        _copter.set_auto_armed(true);

    } else if (stage == Throw_HgtStabilise && throw_height_good()) {
        gcs().send_text(MAV_SEVERITY_INFO,"height achieved - controlling position");
        stage = Throw_PosHold;

        // initialise the loiter target to the curent position and velocity
        wp_nav->init_loiter_target();

        // Set the auto_arm status to true to avoid a possible automatic disarm caused by selection of an auto mode with throttle at minimum
        _copter.set_auto_armed(true);
    } else if (stage == Throw_PosHold && throw_position_good()) {
        if (!nextmode_attempted) {
            switch (g2.throw_nextmode) {
                case AUTO:
                case GUIDED:
                case RTL:
                case LAND:
                case BRAKE:
                case LOITER:
                    set_mode((control_mode_t)g2.throw_nextmode.get(), MODE_REASON_THROW_COMPLETE);
                    break;
                default:
                    // do nothing
                    break;
            }
            nextmode_attempted = true;
        }
    }

    // Throw State Processing
    switch (stage) {

    case Throw_Disarmed:

        // prevent motors from rotating before the throw is detected unless enabled by the user
        if (g.throw_motor_start == 1) {
            motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        } else {
            motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
        }

        // demand zero throttle (motors will be stopped anyway) and continually reset the attitude controller
        attitude_control->set_throttle_out_unstabilized(0,true,g.throttle_filt);
        break;

    case Throw_Detecting:

        // prevent motors from rotating before the throw is detected unless enabled by the user
        if (g.throw_motor_start == 1) {
            motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        } else {
            motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
        }

        // Hold throttle at zero during the throw and continually reset the attitude controller
        attitude_control->set_throttle_out_unstabilized(0,true,g.throttle_filt);

        // Play the waiting for throw tone sequence to alert the user
        AP_Notify::flags.waiting_for_throw = true;

        break;

    case Throw_Uprighting:

        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        // demand a level roll/pitch attitude with zero yaw rate
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f, 0.0f);

        // output 50% throttle and turn off angle boost to maximise righting moment
        attitude_control->set_throttle_out(0.5f, false, g.throttle_filt);

        break;

    case Throw_HgtStabilise:

        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f, 0.0f);

        // call height controller
        pos_control->set_alt_target_from_climb_rate_ff(0.0f, G_Dt, false);
        pos_control->update_z_controller();

        break;

    case Throw_PosHold:

        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        // run loiter controller
        wp_nav->update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), 0.0f);

        // call height controller
        pos_control->set_alt_target_from_climb_rate_ff(0.0f, G_Dt, false);
        pos_control->update_z_controller();

        break;
    }

    // log at 10hz or if stage changes
    uint32_t now = AP_HAL::millis();
    if ((stage != prev_stage) || (now - last_log_ms) > 100) {
        prev_stage = stage;
        last_log_ms = now;
        float velocity = inertial_nav.get_velocity().length();
        float velocity_z = inertial_nav.get_velocity().z;
        float accel = _copter.ins.get_accel().length();
        float ef_accel_z = ahrs.get_accel_ef().z;
        bool throw_detect = (stage > Throw_Detecting) || throw_detected();
        bool attitude_ok = (stage > Throw_Uprighting) || throw_attitude_good();
        bool height_ok = (stage > Throw_HgtStabilise) || throw_height_good();
        bool pos_ok = (stage > Throw_PosHold) || throw_position_good();
        _copter.Log_Write_Throw(stage,
                                velocity,
                                velocity_z,
                                accel,
                                ef_accel_z,
                                throw_detect,
                                attitude_ok,
                                height_ok,
                                pos_ok);
    }
}

bool Copter::ModeThrow::throw_detected()
{
    // Check that we have a valid navigation solution
    nav_filter_status filt_status = inertial_nav.get_filter_status();
    if (!filt_status.flags.attitude || !filt_status.flags.horiz_pos_abs || !filt_status.flags.vert_pos) {
        return false;
    }

    // Check for high speed (>500 cm/s)
    bool high_speed = inertial_nav.get_velocity().length() > THROW_HIGH_SPEED;

    // check for upwards or downwards trajectory (airdrop) of 50cm/s
    bool changing_height;
    if (g2.throw_type == ThrowType_Drop) {
        changing_height = inertial_nav.get_velocity().z < -THROW_VERTICAL_SPEED;
    } else {
        changing_height = inertial_nav.get_velocity().z > THROW_VERTICAL_SPEED;
    }

    // Check the vertical acceleraton is greater than 0.25g
    bool free_falling = ahrs.get_accel_ef().z > -0.25 * GRAVITY_MSS;

    // Check if the accel length is < 1.0g indicating that any throw action is complete and the copter has been released
    bool no_throw_action = _copter.ins.get_accel().length() < 1.0f * GRAVITY_MSS;

    // High velocity or free-fall combined with increasing height indicate a possible air-drop or throw release
    bool possible_throw_detected = (free_falling || high_speed) && changing_height && no_throw_action;

    // Record time and vertical velocity when we detect the possible throw
    if (possible_throw_detected && ((AP_HAL::millis() - free_fall_start_ms) > 500)) {
        free_fall_start_ms = AP_HAL::millis();
        free_fall_start_velz = inertial_nav.get_velocity().z;
    }

    // Once a possible throw condition has been detected, we check for 2.5 m/s of downwards velocity change in less than 0.5 seconds to confirm
    bool throw_condition_confirmed = ((AP_HAL::millis() - free_fall_start_ms < 500) && ((inertial_nav.get_velocity().z - free_fall_start_velz) < -250.0f));

    // start motors and enter the control mode if we are in continuous freefall
    if (throw_condition_confirmed) {
        return true;
    } else {
        return false;
    }
}

bool Copter::ModeThrow::throw_attitude_good()
{
    // Check that we have uprighted the copter
    const Matrix3f &rotMat = ahrs.get_rotation_body_to_ned();
    return (rotMat.c.z > 0.866f); // is_upright
}

bool Copter::ModeThrow::throw_height_good()
{
    // Check that we are within 0.5m of the demanded height
    return (pos_control->get_alt_error() < 50.0f);
}

bool Copter::ModeThrow::throw_position_good()
{
    // check that our horizontal position error is within 50cm
    return (pos_control->get_horizontal_error() < 50.0f);
}
