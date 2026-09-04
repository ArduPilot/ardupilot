#include "Copter.h"

#if MODE_THROW_ENABLED

// Body-frame freefall ceiling for drops.  Centripetal acceleration at the IMU
// mount offset inflates body |a| while the vehicle tumbles - enough to hold it
// near 1g through an entire fall - so a flat 0.5g gate rejects genuine
// spinning freefall.  Bounding the offset at 6 cm gives cap = 0.5g + r*w^2,
// which reduces to the plain gate at w=0.
//
// The ceiling cannot separate falling from handled on its own: above about 9
// rad/s it admits a 1g reading.  The confirmation in throw_detected() carries
// that, by requiring a sustained downward velocity change.
static bool drop_body_in_freefall(float accel_mss, float gyro_rate)
{
    constexpr float IMU_OFFSET_MAX_M = 0.06f;
    const float cap_mss = 0.5f * GRAVITY_MSS + IMU_OFFSET_MAX_M * sq(gyro_rate);
    return accel_mss < cap_mss;
}

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
    drop_confirm_start_ms = 0;
    drop_release_alt_m = 0;
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
        gcs().send_text(MAV_SEVERITY_INFO,"waiting for throw");
        stage = Throw_Detecting;

    } else if (stage == Throw_Detecting && throw_detected()){
        gcs().send_text(MAV_SEVERITY_INFO,"throw detected - spooling motors");
        copter.set_land_complete(false);
        stage = Throw_Wait_Throttle_Unlimited;
        spoolup_start_ms = AP_HAL::millis();
        spoolup_start_vel_u_ms = pos_control->get_vel_estimate_U_ms();

        // Cancel the waiting for throw tone sequence
        AP_Notify::flags.waiting_for_throw = false;

    } else if (stage == Throw_Wait_Throttle_Unlimited &&
               !throw_in_freefall() && !throw_still_falling()) {
        // False trigger during spool-up: the vehicle is neither in freefall
        // nor gaining downward speed, so it is still supported.  Drag alone
        // lifts body |a| past the freefall ceiling part way down a long drop,
        // which is why the velocity test has to agree before we abort.
        gcs().send_text(MAV_SEVERITY_WARNING, "Throw: freefall lost, resetting");
        stage = Throw_Detecting;
        drop_confirm_start_ms = 0;
        free_fall_start_ms = 0;
        AP_Notify::flags.waiting_for_throw = true;

    } else if (stage == Throw_Wait_Throttle_Unlimited &&
               motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
        gcs().send_text(MAV_SEVERITY_INFO,"throttle is unlimited - uprighting");
        stage = Throw_Uprighting;
    } else if (stage == Throw_Uprighting && throw_attitude_good() && throw_drop_distance_reached()) {
        gcs().send_text(MAV_SEVERITY_INFO,"uprighted - controlling height");
        stage = Throw_HgtStabilise;
        hgt_stabilise_start_ms = AP_HAL::millis();
        // initialise the z controller
        pos_control->D_init_controller_no_descent();

        // initialise the demanded height below/above the throw height from user parameters
        // this allows for rapidly clearing surrounding obstacles
        if (g2.throw_type == ThrowType::Drop) {
            // Target altitude is THROW_ALT_DCSND below the release point.  The
            // position controller climbs back to it if uprighting overshot,
            // which is trajectory-limited and cannot exceed the release point.
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
        gcs().send_text(MAV_SEVERITY_INFO,"height achieved - controlling position");
        stage = Throw_PosHold;

        // initialise position controller
        pos_control->NE_init_controller();

        // Set the auto_arm status to true to avoid a possible automatic disarm caused by selection of an auto mode with throttle at minimum
        copter.set_auto_armed(true);
    } else if (stage == Throw_PosHold && throw_position_good()) {
        // PosHold has settled and yaw alignment is either complete or
        // has timed out.  Hand off to THROW_NEXTMODE.  Yaw alignment
        // runs concurrently with HgtStabilise and PosHold (driven by
        // throw_apply_yaw_align) so we don't pay extra wall-clock time
        // for it on top of the height/position settle.
        if (!nextmode_attempted) {
            throw_do_nextmode_handoff();
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
        // ignore motor checks
        motors->set_spoolup_block(false);

        break;

    case Throw_Uprighting:

        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // demand a level roll/pitch attitude with zero yaw rate
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(0.0f, 0.0f, 0.0f);

        // output 50% throttle and turn off angle boost to maximise righting moment
        attitude_control->set_throttle_out(0.5f, false, g.throttle_filt);

        break;

    case Throw_HgtStabilise: {

        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // hold level with zero yaw rate
        const Vector3f hs_thrust_vec_up{0.0f, 0.0f, -1.0f};
        attitude_control->input_thrust_vector_rate_heading_rads(hs_thrust_vec_up, 0.0f);

        // call height controller
        pos_control->D_set_pos_target_from_climb_rate_ms(0.0f);
        pos_control->D_update_controller();

        break;
    }

    case Throw_PosHold:

        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // use position controller to stop
        Vector2f vel_zero;
        Vector2f accel_zero;
        pos_control->input_vel_accel_NE_m(vel_zero, accel_zero);
        pos_control->NE_update_controller();

        // call attitude controller
        attitude_control->input_thrust_vector_rate_heading_rads(pos_control->get_thrust_vector(), 0.0f);

        // call height controller
        pos_control->D_set_pos_target_from_climb_rate_ms(0.0f);
        pos_control->D_update_controller();

        break;
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
        const bool throw_detect = (stage > Throw_Detecting) || drop_confirm_start_ms != 0;
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

    // For drops the body-frame magnitude is the primary check: it reads near
    // zero in freefall whatever the EKF is doing.  The earth-frame Z fallback
    // catches a vertical spin axis, where centripetal stays in the body XY
    // plane.  Upward throws keep the earth-frame check.
    bool free_falling;
    if (g2.throw_type == ThrowType::Drop) {
        const float accel_mss = copter.ins.get_accel().length();
        const float gyro_rate = copter.ins.get_gyro().length();
        const bool body_freefall = drop_body_in_freefall(accel_mss, gyro_rate);
        const bool spin_freefall = fabsf(ahrs.get_accel_ef().z) < 0.5f * GRAVITY_MSS
                                && gyro_rate > 10.0f;
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

    // For drops, require both a sustained low-g window (THROW_DROP_CNF) and
    // evidence that the vehicle is genuinely accelerating downward.  The
    // freefall ceiling scales with spin rate and so cannot by itself tell a
    // falling vehicle from one being handled or carried; the velocity change
    // can, and is the same discriminator the upward path uses.
    if (g2.throw_type == ThrowType::Drop) {
        if (possible_throw_detected) {
            if (drop_confirm_start_ms == 0) {
                drop_confirm_start_ms = AP_HAL::millis();
                drop_release_alt_m = pos_control->get_pos_estimate_U_m();
                drop_confirm_start_vel_u_ms = pos_control->get_vel_estimate_U_ms();
            }
            const float confirm_time_s = MAX(g2.throw_drop_confirm_time.get(), 0.0f);
            const uint32_t confirm_ms = MAX((uint32_t)THROW_DROP_CONFIRM_MS,
                                            (uint32_t)(confirm_time_s * 1000.0f));
            const uint32_t elapsed_ms = AP_HAL::millis() - drop_confirm_start_ms;
            const bool time_confirmed = (elapsed_ms >= confirm_ms);
            // A falling vehicle sheds upward velocity at close to g; one being
            // held or carried does not, whatever it is doing about its axes.
            // The estimate only has to be differentiable, not accurate - a
            // dead-reckoned velocity still integrates the real acceleration -
            // but with no velocity aiding at all there is nothing to compare.
            if (ahrs.has_status(AP_AHRS::Status::CONST_POS_MODE)) {
                return time_confirmed;
            }
            const float elapsed_s = elapsed_ms * 0.001f;
            const float vel_change_ms = pos_control->get_vel_estimate_U_ms() - drop_confirm_start_vel_u_ms;
            const bool fall_confirmed = vel_change_ms <= -THROW_DROP_CONFIRM_ACCEL_FRAC * GRAVITY_MSS * elapsed_s;
            return time_confirmed && fall_confirmed;
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

bool ModeThrow::throw_in_freefall() const
{
    // Spool-up freefall verification.  Body-frame accel is the primary
    // check: it is EKF/AHRS-independent and reads near zero in genuine
    // freefall regardless of filter health.  Under spin, centripetal
    // force inflates body |a| -- drop_body_in_freefall() uses a
    // physics-based ceiling that scales with w^2 to admit the spinning-
    // freefall envelope (mirrors throw_detected()).  Upward throws keep
    // the standard 0.5g check.
    const float accel_mss = copter.ins.get_accel().length();
    const float gyro_rate = copter.ins.get_gyro().length();
    const bool body_in_freefall = (g2.throw_type == ThrowType::Drop)
                                ? drop_body_in_freefall(accel_mss, gyro_rate)
                                : (accel_mss < 0.5f * GRAVITY_MSS);
    if (g2.throw_type != ThrowType::Drop || body_in_freefall) {
        return body_in_freefall;
    }
    // Drop earth-frame fallback: the body ceiling is conservative on r;
    // an unusually large IMU offset or spin beyond the design envelope
    // can still push body |a| above cap.  Earth-Z accel reads ~0 in
    // freefall regardless of spin but depends on AHRS attitude -- if
    // unhealthy we fall back to "not in freefall" (conservative).
    if (!ahrs.has_status(AP_AHRS::Status::ATTITUDE_VALID)) {
        return false;
    }
    return fabsf(ahrs.get_accel_ef().z) < 0.5f * GRAVITY_MSS
        && gyro_rate > 10.0f;
}

// True while the vehicle is still gaining downward speed at a good fraction of
// g, measured from the start of spool-up.  Drag flattens this out as the fall
// develops, so it is only consulted where a false trigger has to be separated
// from a genuine drop, not as a freefall test in its own right.
bool ModeThrow::throw_still_falling() const
{
    const float elapsed_s = (AP_HAL::millis() - spoolup_start_ms) * 0.001f;
    if (!is_positive(elapsed_s)) {
        return true;
    }
    const float vel_change_ms = pos_control->get_vel_estimate_U_ms() - spoolup_start_vel_u_ms;
    return vel_change_ms <= -THROW_SPOOLUP_ABORT_ACCEL_FRAC * GRAVITY_MSS * elapsed_s;
}

bool ModeThrow::throw_drop_distance_reached() const
{
    // For drops, check if vehicle has fallen THROW_ALT_DCSND below the
    // release point.  When DCSND is zero, no distance gate applies.
    if (g2.throw_type != ThrowType::Drop) {
        return true;
    }
    const float dcsnd_m = g.throw_altitude_descend;
    if (!is_positive(dcsnd_m)) {
        return true;
    }
    // Only trust the EKF altitude when the filter has velocity aiding: in
    // const-position mode the integrated velocity drifts far enough to
    // overshoot the release point and wedge this check.
    if (ahrs.has_status(AP_AHRS::Status::VERT_POS) && !ahrs.has_status(AP_AHRS::Status::CONST_POS_MODE)) {
        return (drop_release_alt_m - pos_control->get_pos_estimate_U_m() >= dcsnd_m);
    }
    // Fallback: estimate distance from freefall physics.  This
    // overestimates after motors start (thrust slows the fall) so the
    // gate opens conservatively early.
    const float t = (AP_HAL::millis() - drop_confirm_start_ms) * 0.001f;
    return (0.5f * GRAVITY_MSS * t * t >= dcsnd_m);
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

void ModeThrow::throw_do_nextmode_handoff()
{
    // Final transition to THROW_NEXTMODE shared by the PosHold (no-yaw)
    // and YawAlign exit paths.  Issues throttle warning, applies the
    // optional EKF source-set switch, and attempts the mode change.
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
            set_mode((Mode::Number)g2.throw_nextmode.get(), ModeReason::THROW_COMPLETE);
            break;
        default:
            // do nothing
            break;
    }
    nextmode_attempted = true;
}

#endif
