#include "Copter.h"

#if MODE_THROW_ENABLED

// throw_init - initialise throw controller
bool ModeThrow::init(bool ignore_checks)
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
    xy_controller_active = false;
    drop_confirm_start_ms = 0;
    drop_release_alt_m = 0;
    last_stage_msg_ms = 0;

    // initialise pos controller speed and acceleration
    pos_control->NE_set_max_speed_accel_m(wp_nav->get_default_speed_NE_ms(), BRAKE_MODE_DECEL_RATE_MSS);
    pos_control->NE_set_correction_speed_accel_m(wp_nav->get_default_speed_NE_ms(), BRAKE_MODE_DECEL_RATE_MSS);

    // set vertical speed and acceleration limits
    if (g2.throw_type == ThrowType::Drop) {
        const float ag = MAX(g2.throw_drop_ag, 1.0f);
        pos_control->D_set_max_speed_accel_m(THROW_DROP_SPEED_Z_MS * ag, THROW_DROP_SPEED_Z_MS * ag, THROW_DROP_DECEL_RATE_MSS * ag);
        pos_control->D_set_correction_speed_accel_m(THROW_DROP_SPEED_Z_MS * ag, THROW_DROP_SPEED_Z_MS * ag, THROW_DROP_DECEL_RATE_MSS * ag);
    } else {
        pos_control->D_set_max_speed_accel_m(BRAKE_MODE_SPEED_Z_MS, BRAKE_MODE_SPEED_Z_MS, BRAKE_MODE_DECEL_RATE_MSS);
        pos_control->D_set_correction_speed_accel_m(BRAKE_MODE_SPEED_Z_MS, BRAKE_MODE_SPEED_Z_MS, BRAKE_MODE_DECEL_RATE_MSS);
    }

    return true;
}

// runs the throw to start controller
// should be called at 100hz or more
void ModeThrow::run()
{
    /* Throw State Machine
    Throw_Disarmed - motors are off
    Throw_Detecting -  motors are on and we are waiting for the throw
    Throw_Uprighting - the throw has been detected and the copter is being uprighted
    Throw_HgtStabilise - the copter is kept level and  height is stabilised about the target height
    Throw_PosHold - the copter is kept at a constant position and height
    */

    if (!motors->armed()) {
        // state machine entry is always from a disarmed state
        stage = Throw_Disarmed;

    } else if (stage == Throw_Disarmed && motors->armed()) {
        stage = Throw_Detecting;

    } else if (stage == Throw_Detecting && throw_detected()){
        copter.set_land_complete(false);
        stage = Throw_Wait_Throttle_Unlimited;

        // Cancel the waiting for throw tone sequence
        AP_Notify::flags.waiting_for_throw = false;

    } else if (stage == Throw_Wait_Throttle_Unlimited &&
               motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
        stage = Throw_Uprighting;
        uprighting_start_ms = AP_HAL::millis();
    } else if (stage == Throw_Uprighting && throw_uprighting_complete()) {
        stage = Throw_HgtStabilise;
        hgt_stabilise_start_ms = AP_HAL::millis();

        // initialise the z controller
        pos_control->D_init_controller_no_descent();

        // initialise the demanded height below/above the throw height from user parameters
        // this allows for rapidly clearing surrounding obstacles
        if (g2.throw_type == ThrowType::Drop) {
            // Target altitude is THROW_ALT_DCSND below the release point.
            // The freefall confirmation period already ensures the vehicle
            // has fallen approximately this distance before motors start,
            // so the controller mostly just needs to arrest the descent.
            pos_control->set_pos_desired_U_m(drop_release_alt_m - g.throw_altitude_descend);
        } else {
            pos_control->set_pos_desired_U_m(pos_control->get_pos_estimate_U_m() + g.throw_altitude_ascend);
        }

        // Set the auto_arm status to true to avoid a possible automatic disarm caused by selection of an auto mode with throttle at minimum
        copter.set_auto_armed(true);

    } else if (stage == Throw_HgtStabilise &&
               ((g2.throw_type == ThrowType::Drop)
                   ? (throw_velocity_good() || (AP_HAL::millis() - hgt_stabilise_start_ms > 3000))
                   : (throw_height_good() && (throw_velocity_good() || (AP_HAL::millis() - hgt_stabilise_start_ms > 2000))))) {
        // check if we have horizontal position for PosHold
        const bool have_horiz_pos = ahrs.has_status(AP_AHRS::Status::HORIZ_POS_ABS);
        // determine if the next mode needs horizontal position
        const Mode::Number nextmode = (Mode::Number)g2.throw_nextmode.get();
        const bool nextmode_needs_pos = (nextmode != Mode::Number::STABILIZE &&
                                         nextmode != Mode::Number::ALT_HOLD);
        if (have_horiz_pos) {
            gcs().send_text(MAV_SEVERITY_INFO,"Throw height achieved, good position");
            stage = Throw_PosHold;

            // initialise position controller
            pos_control->NE_init_controller();
            xy_controller_active = true;
        } else if (nextmode_needs_pos) {
            gcs().send_text(MAV_SEVERITY_WARNING,"Throw height achieved, lost position");
            stage = Throw_PosHold;
        } else {
            gcs().send_text(MAV_SEVERITY_INFO,"Throw height achieved");
            stage = Throw_PosHold;
        }

        // Set the auto_arm status to true to avoid a possible automatic disarm caused by selection of an auto mode with throttle at minimum
        copter.set_auto_armed(true);
    } else if (stage == Throw_PosHold && (!xy_controller_active || throw_position_good())) {
        if (!nextmode_attempted) {
            // Warn if throttle is low — in ALT_HOLD, below mid-stick commands descent
            if (channel_throttle->get_control_in() < copter.get_throttle_mid() - copter.g.throttle_deadzone) {
                gcs().send_text(MAV_SEVERITY_WARNING, "Throttle low - losing altitude");
            }
            switch ((Mode::Number)g2.throw_nextmode.get()) {
                case Mode::Number::AUTO:
                case Mode::Number::GUIDED:
                case Mode::Number::RTL:
                case Mode::Number::LAND:
                case Mode::Number::BRAKE:
                case Mode::Number::LOITER:
                case Mode::Number::STABILIZE:
                case Mode::Number::ALT_HOLD:
                    set_mode((Mode::Number)g2.throw_nextmode.get(), ModeReason::THROW_COMPLETE);
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
        if (g.throw_motor_start == PreThrowMotorState::RUNNING) {
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        } else {
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        }

        // demand zero throttle (motors will be stopped anyway) and continually reset the attitude controller
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_throttle_out(0,true,g.throttle_filt);
        break;

    case Throw_Detecting:

        // prevent motors from rotating before the throw is detected unless enabled by the user
        if (g.throw_motor_start == PreThrowMotorState::RUNNING) {
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        } else {
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        }

        // Hold throttle at zero during the throw and continually reset the attitude controller
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_throttle_out(0,true,g.throttle_filt);

        // Play the waiting for throw tone sequence to alert the user
        AP_Notify::flags.waiting_for_throw = true;

        break;

    case Throw_Wait_Throttle_Unlimited:

        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // Keep the attitude controller's internal target tracking the
        // vehicle's actual attitude through the spool-up.  Without this,
        // _attitude_target is stale from Detecting and input_quaternion
        // in Uprighting computes the wrong error, producing a slow mushy
        // recovery instead of a crisp shortest-path rotation.
        attitude_control->relax_attitude_controllers();
        attitude_control->set_throttle_out(0, true, g.throttle_filt);

        break;

    case Throw_Uprighting: {

        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // Use quaternion attitude controller to find the shortest rotation
        // path to level from any starting orientation — including inverted.
        // The Euler method has singularities near ±90° pitch that produce
        // suboptimal paths.  Target a level attitude preserving current yaw.
        Quaternion level_quat;
        level_quat.from_euler(0.0f, 0.0f, ahrs.get_yaw());
        Vector3f zero_ang_vel;
        attitude_control->input_quaternion(level_quat, zero_ang_vel);

        // For drops, scale throttle by cos_tilt so thrust is zero when
        // inverted and ramps to arrest level as the vehicle rights itself.
        // Angle boost is disabled since we manage the throttle directly.
        // For upward throws use 50% without boost to maximise righting moment.
        if (g2.throw_type == ThrowType::Drop) {
            const float cos_tilt = MAX(ahrs.get_rotation_body_to_ned().c.z, 0.0f);
            const float hover_thr = motors->get_throttle_hover();
            // Scale arrest aggressiveness by descent rate.  When going
            // up or barely descending, use hover throttle only.
            // Ramp linearly to full THROW_DROP_AG at THROW_DROP_SPEED_Z_MS.
            const float ag = MAX(g2.throw_drop_ag, 1.0f);
            const float vel_z_up = pos_control->get_vel_estimate_U_ms();
            float thr_scale;
            if (vel_z_up >= 0.0f) {
                thr_scale = 1.0f;
            } else {
                thr_scale = constrain_float(1.0f + (ag - 1.0f) * (-vel_z_up) * (1.0f / THROW_DROP_SPEED_Z_MS), 1.0f, ag);
            }
            const float throttle = constrain_float(hover_thr * thr_scale, 0.0f, 1.0f) * cos_tilt;
            attitude_control->set_throttle_out(throttle, false, g.throttle_filt);
        } else {
            attitude_control->set_throttle_out(0.5f, false, g.throttle_filt);
        }

        break;
    }

    case Throw_HgtStabilise:

        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(0.0f, 0.0f, 0.0f);

        // call height controller
        pos_control->D_set_pos_target_from_climb_rate_ms(0.0f);
        pos_control->D_update_controller();

        break;

    case Throw_PosHold:

        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        if (xy_controller_active) {
            // use position controller to stop
            Vector2f vel_zero;
            Vector2f accel_zero;
            pos_control->input_vel_accel_NE_m(vel_zero, accel_zero);
            pos_control->NE_update_controller();

            // call attitude controller
            attitude_control->input_thrust_vector_rate_heading_rads(pos_control->get_thrust_vector(), 0.0f);
        } else {
            // no horizontal position available, hold level attitude only
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(0.0f, 0.0f, 0.0f);
        }

        // call height controller
        pos_control->D_set_pos_target_from_climb_rate_ms(0.0f);
        pos_control->D_update_controller();

        break;
    }

    // update OSD mode string and send periodic GCS stage messages at 2Hz
    {
        const uint32_t now_ms = AP_HAL::millis();
        const char *mode_str = "THRW";
        const char *stage_msg = nullptr;
        switch (stage) {
        case Throw_Disarmed:
            break;
        case Throw_Detecting:
            // flash mode string while armed and waiting for throw
            mode_str = ((now_ms / 500) % 2 == 0) ? "THRW" : "    ";
            stage_msg = "Waiting for throw";
            break;
        case Throw_Wait_Throttle_Unlimited:
            mode_str = ((now_ms / 250) % 2 == 0) ? "THR!" : "    ";
            stage_msg = "Throw detected";
            break;
        case Throw_Uprighting:
            mode_str = ((now_ms / 250) % 2 == 0) ? "THR!" : "    ";
            stage_msg = "Throw detected";
            break;
        case Throw_HgtStabilise:
            mode_str = "THHT";
            stage_msg = "Stabilizing throw height";
            break;
        case Throw_PosHold:
            mode_str = "THPH";
            stage_msg = "Throw holding position";
            break;
        }
        AP::notify().set_flight_mode_str(mode_str);

        if (stage_msg != nullptr && (now_ms - last_stage_msg_ms >= 500)) {
            last_stage_msg_ms = now_ms;
            gcs().send_text(MAV_SEVERITY_INFO, "%s", stage_msg);
        }
    }

#if HAL_LOGGING_ENABLED
    // log at 10hz or if stage changes
    uint32_t now = AP_HAL::millis();
    if ((stage != prev_stage) || (now - last_log_ms) > 100) {
        prev_stage = stage;
        last_log_ms = now;
        const float velocity_ms = pos_control->get_vel_estimate_NED_ms().length();
        const float velocity_z_ms = pos_control->get_vel_estimate_U_ms();
        const float accel_mss = copter.ins.get_accel().length();
        const float ef_accel_z_mss = ahrs.get_accel_ef().z;
        const bool throw_detect = (stage > Throw_Detecting) || throw_detected();
        const bool attitude_ok = (stage > Throw_Uprighting) || throw_attitude_good();
        const bool height_ok = (stage > Throw_HgtStabilise) || throw_height_good();
        const bool pos_ok = (stage > Throw_PosHold) || throw_position_good();

// @LoggerMessage: THRO
// @Description: Throw Mode messages
// @URL: https://ardupilot.org/copter/docs/throw-mode.html
// @Field: TimeUS: Time since system startup
// @Field: Stage: Current stage of the Throw Mode
// @Field: Vel: Magnitude of the velocity vector
// @Field: VelZ: Vertical Velocity
// @Field: Acc: Magnitude of the vector of the current acceleration
// @Field: AccEfZ: Vertical earth frame accelerometer value
// @Field: Throw: True if a throw has been detected since entering this mode
// @Field: AttOk: True if the vehicle is upright 
// @Field: HgtOk: True if the vehicle is within 0.5 m of the demanded height
// @Field: PosOk: True if the vehicle is within 0.5 m of the demanded horizontal position

        AP::logger().WriteStreaming(
            "THRO",
            "TimeUS,Stage,Vel,VelZ,Acc,AccEfZ,Throw,AttOk,HgtOk,PosOk",
            "s-nnoo----",
            "F-0000----",
            "QBffffbbbb",
            AP_HAL::micros64(),
            (uint8_t)stage,
            (double)velocity_ms,
            (double)velocity_z_ms,
            (double)accel_mss,
            (double)ef_accel_z_mss,
            throw_detect,
            attitude_ok,
            height_ok,
            pos_ok);
    }
#endif  // HAL_LOGGING_ENABLED
}

bool ModeThrow::throw_detected()
{
    // Check that we have a valid navigation solution
    if (!ahrs.has_status(AP_AHRS::Status::ATTITUDE_VALID)) {
        return false;
    }
    if (!ahrs.has_status(AP_AHRS::Status::VERT_POS)) {
        return false;
    }

    // Check for high speed ( >5 m/s)
    bool high_speed = pos_control->get_vel_estimate_NED_ms().length_squared() > (THROW_HIGH_SPEED_MS * THROW_HIGH_SPEED_MS);

    // check for upwards or downwards trajectory (airdrop) of 0.50 m/s
    bool changing_height;
    if (g2.throw_type == ThrowType::Drop) {
        changing_height = pos_control->get_vel_estimate_U_ms() < -THROW_VERTICAL_SPEED_MS;
    } else {
        changing_height = pos_control->get_vel_estimate_U_ms() > THROW_VERTICAL_SPEED_MS;
    }

    // Check for freefall.  For drops use body-frame accelerometer as the
    // primary check — it reads near zero in freefall regardless of EKF
    // state, and ~1g while attached to a carrier.  As a secondary path,
    // check earth-frame Z acceleration when the vehicle is spinning fast
    // (>10 rad/s).  Centripetal acceleration from yaw spin inflates
    // body-frame magnitude but is entirely in the horizontal plane, so
    // earth-frame Z remains ~0 in freefall.  The gyro rate gate prevents
    // false triggers on a carrier with bad EKF attitude (low gyro rate).
    // For upward throws keep the existing earth-frame check.
    bool free_falling;
    if (g2.throw_type == ThrowType::Drop) {
        const bool body_freefall = copter.ins.get_accel().length() < 0.5f * GRAVITY_MSS;
        const bool spin_freefall = fabsf(ahrs.get_accel_ef().z) < 0.5f * GRAVITY_MSS
                                && copter.ins.get_gyro().length() > 10.0f;
        free_falling = body_freefall || spin_freefall;
    } else {
        free_falling = ahrs.get_accel_ef().z > -0.25f * GRAVITY_MSS;
    }

    // Check if the accel length is < 1.0g indicating that any throw action is complete and the copter has been released
    bool no_throw_action = copter.ins.get_accel().length() < 1.0f * GRAVITY_MSS;

    // fetch the altitude above home
    float altitude_above_home_m;  // Use altitude above home if it is set, otherwise relative to EKF origin
    if (ahrs.home_is_set()) {
        ahrs.get_relative_position_D_home(altitude_above_home_m);
        altitude_above_home_m = -altitude_above_home_m; // altitude above home is returned as negative
    } else {
        altitude_above_home_m = pos_control->get_pos_estimate_U_m();
    }

    // Check that the altitude is within user defined limits
    const bool height_within_params = (g.throw_altitude_min == 0 || altitude_above_home_m > g.throw_altitude_min) && (g.throw_altitude_max == 0 || (altitude_above_home_m < g.throw_altitude_max));

    // High velocity or free-fall combined with increasing height indicate a possible air-drop or throw release
    bool possible_throw_detected;
    if (g2.throw_type == ThrowType::Drop) {
        // For drops, freefall detection is sufficient.  The no_throw_action
        // check (body accel < 1g) is redundant for body_freefall and would
        // falsely block spin_freefall where centripetal acceleration
        // inflates body-frame magnitude above 1g.
        possible_throw_detected = free_falling && height_within_params;
    } else {
        possible_throw_detected = (free_falling || high_speed) && changing_height && no_throw_action && height_within_params;
    }

    // For drops, require freefall conditions to persist long enough to
    // reject transient low-g events (e.g. carrier aircraft turbulence).
    // When THROW_ALT_DCSND > 0, require freefall for the time it takes
    // to fall that distance from rest: t = sqrt(2*d/g).  This ensures
    // the vehicle has truly separated from the carrier.
    if (g2.throw_type == ThrowType::Drop) {
        if (possible_throw_detected) {
            if (drop_confirm_start_ms == 0) {
                drop_confirm_start_ms = AP_HAL::millis();
                drop_release_alt_m = pos_control->get_pos_estimate_U_m();
            }
            const float dcsnd_m = g.throw_altitude_descend;
            const uint32_t confirm_ms = is_positive(dcsnd_m)
                ? MAX((uint32_t)THROW_DROP_CONFIRM_MS, (uint32_t)(sqrtf(2.0f * dcsnd_m / GRAVITY_MSS) * 1000.0f))
                : THROW_DROP_CONFIRM_MS;
            return (AP_HAL::millis() - drop_confirm_start_ms >= confirm_ms);
        }
        drop_confirm_start_ms = 0;
        return false;
    }

    // Record time and vertical velocity when we detect the possible throw
    if (possible_throw_detected && ((AP_HAL::millis() - free_fall_start_ms) > 500)) {
        free_fall_start_ms = AP_HAL::millis();
        free_fall_start_vel_u_ms = pos_control->get_vel_estimate_U_ms();
    }

    // Once a possible throw condition has been detected, we check for 2.5 m/s of downwards velocity change in less than 0.5 seconds to confirm
    bool throw_condition_confirmed = ((AP_HAL::millis() - free_fall_start_ms < 500) && ((pos_control->get_vel_estimate_U_ms() - free_fall_start_vel_u_ms) < -2.5));

    // start motors and enter the control mode if we are in continuous freefall
    return throw_condition_confirmed;
}

bool ModeThrow::throw_uprighting_complete() const
{
    // Three-tier exit from the uprighting stage:
    // 1. Within ~5° of level — attitude is excellent, proceed immediately
    // 2. Within 30° of level and 2s elapsed — gave it time, good enough
    // 3. 3s elapsed — safety timeout, proceed regardless
    const float cos_tilt = ahrs.get_rotation_body_to_ned().c.z;
    const uint32_t elapsed_ms = AP_HAL::millis() - uprighting_start_ms;
    if (cos_tilt > 0.996f) {            // ~5°
        return true;
    }
    if (cos_tilt > 0.866f && elapsed_ms > 2000) {  // ~30°
        return true;
    }
    return (elapsed_ms > 3000);
}

bool ModeThrow::throw_attitude_good() const
{
    // Check that we have uprighted the copter
    const Matrix3f &rotMat = ahrs.get_rotation_body_to_ned();
    return (rotMat.c.z > 0.866f); // is_upright
}

bool ModeThrow::throw_height_good() const
{
    // Check that we are within 0.5m of the demanded height
    return (fabsf(pos_control->get_pos_error_D_m()) < 0.5);
}

bool ModeThrow::throw_velocity_good() const
{
    // Check that vertical velocity is below 0.5 m/s
    return (fabsf(pos_control->get_vel_estimate_U_ms()) < 0.5f);
}

bool ModeThrow::throw_position_good() const
{
    // check that our horizontal position error is within 0.5 m
    return (pos_control->get_pos_error_NE_m() < 0.50);
}

#endif
