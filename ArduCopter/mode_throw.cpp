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
    xy_controller_active = false;
    drop_confirm_start_ms = 0;
    drop_release_alt_m = 0;
    yaw_align_start_ms = 0;
    yaw_align_timeout_ms = THROW_YAW_ALIGN_TIMEOUT_MS;
    yaw_align_locked = false;

    // Capture the throw-direction fallbacks.  A moving carrier's inherited
    // velocity is invisible to the IMU integrator, and a stationary release
    // has no velocity at all, so both entry velocity and entry yaw are kept.
    Vector3f vel_ned_ms;
    if (ahrs.get_velocity_NED(vel_ned_ms)) {
        throw_entry_vel_ne_ms = vel_ned_ms.xy();
        throw_entry_vel_valid = true;
    } else {
        throw_entry_vel_ne_ms.zero();
        throw_entry_vel_valid = false;
    }
    if (ahrs.has_status(AP_AHRS::Status::ATTITUDE_VALID)) {
        throw_entry_yaw_rad = ahrs.get_yaw_rad();
        throw_entry_yaw_valid = true;
    } else {
        throw_entry_yaw_rad = 0.0f;
        throw_entry_yaw_valid = false;
    }

    // Reset the IMU throw-direction integrator state.
    throw_dir_reset();
    throw_target_yaw_valid = false;
    throw_target_yaw_rad = 0.0f;
    throw_yaw_source = ThrowYawSource::None;

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

// Drops are designed to operate without horizontal aiding -- detection is
// body-frame / baro based -- so a drop can arm GPS-free.  Upward throws keep
// the upstream requirement of a position estimate (GPS or flow-relative);
// only a fully aiding-less upward throw is refused at arming.
bool ModeThrow::requires_position() const
{
    return g2.throw_type != ThrowType::Drop;
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

    // Track throw direction whenever the state machine is in Detecting
    // (motors off, body accel uncontaminated by thrust).  No-op
    // otherwise.  Cheap; just integration.
    throw_dir_update();

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

        // Lock in the Uprighting yaw target now, while the IMU
        // integrator state still reflects the throw motion.  Stashed
        // for use when Uprighting begins.
        (void)throw_dir_finalise_target_yaw();

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
        uprighting_start_ms = AP_HAL::millis();
    } else if (stage == Throw_Uprighting && throw_uprighting_complete() && throw_drop_distance_reached()) {
        gcs().send_text(MAV_SEVERITY_INFO,"uprighted - controlling height");
        stage = Throw_HgtStabilise;
        hgt_stabilise_start_ms = AP_HAL::millis();
        yaw_align_start_ms = hgt_stabilise_start_ms;
        yaw_align_locked = false;

        // Size the timeout to the rotation actually required: a fixed budget
        // cuts a half-turn short, and a heading-holding next mode then freezes
        // the partial heading.  Capped so a slow yaw tune cannot stall the
        // handoff for tens of seconds.
        yaw_align_timeout_ms = THROW_YAW_ALIGN_TIMEOUT_MS;
        if (throw_target_yaw_valid) {
            const float slew_rads = attitude_control->get_slew_yaw_max_rads();
            if (is_positive(slew_rads)) {
                const float err_rad = fabsf(wrap_PI(throw_target_yaw_rad - ahrs.get_yaw_rad()));
                const float needed_ms = MIN(err_rad / slew_rads * 1000.0f + THROW_YAW_ALIGN_MARGIN_MS,
                                            (float)THROW_YAW_ALIGN_TIMEOUT_MAX_MS);
                yaw_align_timeout_ms = MAX((uint32_t)THROW_YAW_ALIGN_TIMEOUT_MS, (uint32_t)needed_ms);
            }
        }
        yaw_align_timeout_ms = MIN(yaw_align_timeout_ms, (uint32_t)THROW_YAW_ALIGN_TIMEOUT_MAX_MS);

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
        // check if we have horizontal position for PosHold.  position_ok()
        // accepts a flow-relative estimate and honours the EKF failsafe, so it
        // matches what the next mode will be held to.
        const bool have_horiz_pos = copter.position_ok();
        // determine if the next mode needs horizontal position
        const Mode::Number nextmode = (Mode::Number)g2.throw_nextmode.get();
        const bool nextmode_needs_pos = (nextmode != Mode::Number::STABILIZE &&
                                         nextmode != Mode::Number::ALT_HOLD &&
                                         nextmode != Mode::Number::ACRO);
        if (have_horiz_pos) {
            gcs().send_text(MAV_SEVERITY_INFO,"Throw height achieved, good position");
            pos_control->NE_init_controller();
            xy_controller_active = true;
        } else if (nextmode_needs_pos) {
            gcs().send_text(MAV_SEVERITY_WARNING,"Throw height achieved, lost position");
        } else {
            gcs().send_text(MAV_SEVERITY_INFO,"Throw height achieved");
        }
        stage = Throw_PosHold;

        // Set the auto_arm status to true to avoid a possible automatic disarm caused by selection of an auto mode with throttle at minimum
        copter.set_auto_armed(true);
    } else if (stage == Throw_PosHold &&
               (!xy_controller_active || throw_position_good()) &&
               throw_yaw_align_done()) {
        // PosHold has settled and yaw alignment is either complete or
        // has timed out.  Hand off to THROW_NEXTMODE.  Yaw alignment
        // runs concurrently with HgtStabilise and PosHold (driven by
        // throw_apply_yaw_align) so we don't pay extra wall-clock time
        // for it on top of the height/position settle.
        if (!nextmode_attempted) {
            const bool yaw_target_active = throw_target_yaw_valid &&
                                           (ThrowYawType)g2.throw_yaw_type.get() != ThrowYawType::None;
            if (yaw_target_active && !throw_yaw_converged()) {
                gcs().send_text(MAV_SEVERITY_WARNING, "Throw yaw align timeout");
            }
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

        // Track the vehicle's actual attitude through the spool-up; a target
        // left stale from Detecting gives Uprighting the wrong error.
        attitude_control->relax_attitude_controllers();
        attitude_control->set_throttle_out(0, true, g.throttle_filt);

        break;

    case Throw_Uprighting: {

        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // Thrust-vector levelling takes the shortest path from any starting
        // orientation, inverted included.  Yaw is left as a rate target here
        // so the rate controller can damp the spin; THROW_YAW_TYPE is applied
        // later, by throw_apply_yaw_align().
        const Vector3f thrust_vec_up{0.0f, 0.0f, -1.0f};
        attitude_control->input_thrust_vector_rate_heading_rads(thrust_vec_up, 0.0f);

        // Drops get zero throttle, leaving the attitude controller's
        // differential thrust to do the righting; the arrest is HgtStabilise's
        // job.  Upward throws use 50% without boost for righting moment.
        if (g2.throw_type == ThrowType::Drop) {
            attitude_control->set_throttle_out(0, false, g.throttle_filt);
        } else {
            attitude_control->set_throttle_out(0.5f, false, g.throttle_filt);
        }

        break;
    }

    case Throw_HgtStabilise: {

        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // Yaw alignment runs alongside the height arrest.
        const Vector3f hs_thrust_vec_up{0.0f, 0.0f, -1.0f};
        throw_apply_yaw_align(hs_thrust_vec_up);

        // call height controller
        pos_control->D_set_pos_target_from_climb_rate_ms(0.0f);
        pos_control->D_update_controller();

        break;
    }

    case Throw_PosHold:

        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        if (xy_controller_active) {
            // use position controller to stop
            Vector2f vel_zero;
            Vector2f accel_zero;
            pos_control->input_vel_accel_NE_m(vel_zero, accel_zero);
            pos_control->NE_update_controller();

            // Yaw alignment continues during PosHold.  Pass the
            // position controller's thrust vector so position-keeping
            // tilt is preserved when we lock to absolute yaw.
            throw_apply_yaw_align(pos_control->get_thrust_vector());
        } else {
            // no horizontal position available, hold level attitude only
            const Vector3f ph_thrust_vec_up{0.0f, 0.0f, -1.0f};
            throw_apply_yaw_align(ph_thrust_vec_up);
        }

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
// @Field: AeZ: Vertical earth frame accelerometer value
// @Field: Throw: True if a throw has been detected since entering this mode
// @Field: AttOk: True if the vehicle is upright
// @Field: HgtOk: True if the vehicle is within 0.5 m of the demanded height
// @Field: PosOk: True if the vehicle is within 0.5 m of the demanded horizontal position
// @Field: TYaw: Recovery yaw target heading (THROW_YAW_TYPE), 0 until resolved at the freefall transition
// @Field: YSrc: Source that supplied the yaw target (0=none,1=IMU direction,2=entry velocity,3=entry yaw,4=absolute)

        AP::logger().WriteStreaming(
            "THRO",
            "TimeUS,Stage,Vel,VelZ,Acc,AeZ,Throw,AttOk,HgtOk,PosOk,TYaw,YSrc",
            "s-nnoo----d-",
            "F-0000----0-",
            "QBffffbbbbfB",
            AP_HAL::micros64(),
            (uint8_t)stage,
            (double)velocity_ms,
            (double)velocity_z_ms,
            (double)accel_mss,
            (double)ef_accel_z_mss,
            throw_detect,
            attitude_ok,
            height_ok,
            pos_ok,
            (double)degrees(throw_target_yaw_rad),
            (uint8_t)throw_yaw_source);
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

void ModeThrow::throw_dir_reset()
{
    // The integration that follows captures motion since this reset, so the
    // most recent reset is what anchors the velocity vector at release.
    throw_dir_q.initialise();
    throw_dir_q_valid = false;
    throw_dir_vel_ne_ms.zero();
    throw_dir_last_us = 0;
    throw_dir_anchor_yaw_valid = false;
    throw_dir_anchor_yaw_rad = 0.0f;
}

void ModeThrow::throw_dir_update()
{
    // Gyro-only body-to-pseudo-earth quaternion, anchored on the gravity
    // vector at the most recent stationary sample.  Pseudo-earth is Z-down
    // with arbitrary yaw, which is enough for a horizontal velocity vector;
    // its heading is tied to NED by one EKF yaw read at release.  Detecting
    // only, so motor thrust cannot contaminate the accelerometer.
    if (stage != Throw_Detecting) {
        return;
    }

    const Vector3f accel_body_mss = copter.ins.get_accel();
    const Vector3f gyro_rads = copter.ins.get_gyro();
    const uint32_t now_us = AP_HAL::micros();

    // Held-still detection: body |accel| close to gravity AND gyro
    // small.  Both required -- a free-falling vehicle has |accel|~0,
    // and a centripetal-loaded spinning vehicle has |accel|~g but
    // large gyro, neither of which is "stationary".
    const float accel_err_mss = fabsf(accel_body_mss.length() - GRAVITY_MSS);
    const bool held_still = (accel_err_mss < 0.2f * GRAVITY_MSS) &&
                            (gyro_rads.length() < 1.0f);

    if (held_still) {
        // Body yaw at this moment maps to pseudo-yaw 0, so the EKF yaw
        // captured here is what converts the pseudo-earth heading back to NED
        // at release.  Specific force at rest is -gravity_body, giving the
        // body-frame gravity direction that roll and pitch come from.
        Vector3f gravity_dir_body = -accel_body_mss;
        if (!gravity_dir_body.is_zero()) {
            gravity_dir_body.normalize();
            const float pitch_rad = asinf(constrain_float(-gravity_dir_body.x, -1.0f, 1.0f));
            const float roll_rad = atan2f(gravity_dir_body.y, gravity_dir_body.z);
            throw_dir_q.from_euler(roll_rad, pitch_rad, 0.0f);
            throw_dir_q_valid = true;
        }
        if (ahrs.has_status(AP_AHRS::Status::ATTITUDE_VALID)) {
            throw_dir_anchor_yaw_rad = ahrs.get_yaw_rad();
            throw_dir_anchor_yaw_valid = true;
        } else {
            throw_dir_anchor_yaw_valid = false;
        }
        throw_dir_vel_ne_ms.zero();
        throw_dir_last_us = now_us;
        return;
    }

    if (!throw_dir_q_valid || throw_dir_last_us == 0) {
        // Haven't yet seen a stationary sample to anchor from -- nothing
        // meaningful to integrate.  Wait for the operator to hold the
        // vehicle still briefly before throwing.
        throw_dir_last_us = now_us;
        return;
    }

    // Propagate attitude from gyro and integrate horizontal velocity.
    const float dt_s = constrain_float((now_us - throw_dir_last_us) * 1.0e-6f, 0.0f, 0.05f);
    throw_dir_last_us = now_us;
    if (dt_s <= 0.0f) {
        return;
    }

    // Integrate the body-to-pseudo-earth quaternion.
    throw_dir_q.rotate(gyro_rads * dt_s);
    throw_dir_q.normalize();

    // Rotate specific force into pseudo-earth and add gravity to recover the
    // vehicle's own acceleration: at rest the two cancel, in freefall the
    // result is g downward.
    Matrix3f rot_b_to_pe;
    throw_dir_q.rotation_matrix(rot_b_to_pe);
    Vector3f accel_pe_mss = rot_b_to_pe * accel_body_mss;
    accel_pe_mss.z += GRAVITY_MSS;
    throw_dir_vel_ne_ms.x += accel_pe_mss.x * dt_s;
    throw_dir_vel_ne_ms.y += accel_pe_mss.y * dt_s;
}

bool ModeThrow::throw_dir_finalise_target_yaw()
{
    // Called once at the Detecting -> Wait_Throttle_Unlimited transition
    // to lock in the heading the Uprighting stage will target.  Returns
    // true and sets throw_target_yaw_rad on success; returns false (no
    // target, hold current yaw) when the operator's THROW_YAW_TYPE is
    // disabled or no source has a confident horizontal vector.
    throw_target_yaw_valid = false;
    throw_target_yaw_rad = 0.0f;
    throw_yaw_source = ThrowYawSource::None;

    const ThrowYawType yaw_type = (ThrowYawType)g2.throw_yaw_type.get();
    if (yaw_type == ThrowYawType::None) {
        return false;
    }

    if (yaw_type == ThrowYawType::Absolute) {
        throw_target_yaw_rad = wrap_PI(radians(g2.throw_yaw_deg.get()));
        throw_target_yaw_valid = true;
        throw_yaw_source = ThrowYawSource::Absolute;
        return true;
    }

    // Confidence threshold: below ~1.5 m/s horizontal the heading is
    // dominated by noise and a yaw target would be a coin flip.
    const float min_speed_ms = 1.5f;

    float heading_rad = 0.0f;
    bool have_heading = false;

    // Source 1: IMU-integrated pseudo-earth velocity.  Pseudo-earth
    // yaw=0 was anchored to the body's NED yaw at the most recent
    // held-still sample, captured then as throw_dir_anchor_yaw_rad.
    // Heading in NED = anchor yaw + pseudo-frame heading.
    if (throw_dir_q_valid && throw_dir_anchor_yaw_valid &&
        throw_dir_vel_ne_ms.length() >= min_speed_ms) {
        const float pseudo_heading_rad = atan2f(throw_dir_vel_ne_ms.y,
                                                throw_dir_vel_ne_ms.x);
        heading_rad = wrap_PI(throw_dir_anchor_yaw_rad + pseudo_heading_rad);
        have_heading = true;
        throw_yaw_source = ThrowYawSource::ImuDirection;
    }

    // Source 2: EKF NED velocity captured at mode entry.  Useful for
    // moving-carrier drops where the IMU integrator doesn't see the
    // inherited carrier velocity.
    if (!have_heading && throw_entry_vel_valid &&
        throw_entry_vel_ne_ms.length() >= min_speed_ms) {
        heading_rad = atan2f(throw_entry_vel_ne_ms.y, throw_entry_vel_ne_ms.x);
        have_heading = true;
        throw_yaw_source = ThrowYawSource::EntryVelocity;
    }

    // Source 3: EKF yaw captured at mode entry.  Useful for stationary
    // cases (operator pointing a held vehicle, hovering carrier with
    // mount-aligned forward direction) where neither motion source
    // produced a confident vector.  Always preferable to "current yaw"
    // because the vehicle has been thrown/tumbled since entry.
    if (!have_heading && throw_entry_yaw_valid) {
        heading_rad = throw_entry_yaw_rad;
        have_heading = true;
        throw_yaw_source = ThrowYawSource::EntryYaw;
    }

    if (!have_heading) {
        // No source available at all (e.g., AHRS unhealthy at entry
        // and no motion since).  Silently hold current yaw.
        return false;
    }

    if (yaw_type == ThrowYawType::ReverseThrowDirection) {
        heading_rad = wrap_PI(heading_rad + M_PI);
    }

    throw_target_yaw_rad = heading_rad;
    throw_target_yaw_valid = true;
    return true;
}

bool ModeThrow::throw_uprighting_complete() const
{
    // Three-tier exit from the uprighting stage:
    // 1. Within ~5deg of level -- attitude is excellent, proceed immediately
    // 2. Within 30deg of level and 2s elapsed -- gave it time, good enough
    // 3. 3s elapsed -- safety timeout, proceed regardless
    const float cos_tilt = ahrs.get_rotation_body_to_ned().c.z;
    const uint32_t elapsed_ms = AP_HAL::millis() - uprighting_start_ms;
    if (cos_tilt > 0.996f) {            // ~5deg
        return true;
    }
    if (cos_tilt > 0.866f && elapsed_ms > 2000) {  // ~30deg
        return true;
    }
    return (elapsed_ms > 3000);
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

void ModeThrow::throw_apply_yaw_align(const Vector3f& thrust_vector)
{
    // Runs during HgtStabilise and PosHold, alongside the height and position
    // settle rather than after it.  Above the ride threshold the commanded
    // rate follows the measured spin so the vehicle is left to decay; below
    // it, a rate-limited slew closes on the target; once inside the catch
    // window the absolute heading is held.
    if (!throw_target_yaw_valid ||
        (ThrowYawType)g2.throw_yaw_type.get() == ThrowYawType::None) {
        attitude_control->input_thrust_vector_rate_heading_rads(thrust_vector, 0.0f);
        return;
    }

    const float yaw_err_rad = wrap_PI(throw_target_yaw_rad - ahrs.get_yaw_rad());
    const float gyro_z_rads = ahrs.get_gyro().z;
    const float catch_window_rad = radians(THROW_YAW_CATCH_WINDOW_DEG);
    // ATC_RATE_WPY_MAX governs both the rate commanded here and the
    // controller's own limiter, so raising it speeds up the whole approach.
    const float slew_cap_rads = attitude_control->get_slew_yaw_max_rads();
    const float ride_threshold_rads = radians(THROW_YAW_RIDE_THRESH_DEG);

    // Lock as soon as yaw enters the catch window, so the controller brakes
    // the spin onto the target rather than coasting it in the ride branch.  A
    // fast spin can sweep through the window and latch for one loop, which is
    // not a genuine alignment, so only the message is gated on the rate;
    // throw_yaw_converged() gates the handoff on it separately.
    if (!yaw_align_locked && fabsf(yaw_err_rad) <= catch_window_rad) {
        yaw_align_locked = true;
        if (fabsf(gyro_z_rads) <= ride_threshold_rads) {
            gcs().send_text(MAV_SEVERITY_INFO, "Throw yaw aligned");
        }
    }

    if (yaw_align_locked) {
        attitude_control->input_thrust_vector_heading_rad(thrust_vector, throw_target_yaw_rad);
        return;
    }

    // The gain holds the slew cap saturated until the error falls below
    // slew_cap/gain, so the approach runs at full rate and eases off just
    // short of the lock window.
    float ya_yaw_rate_rads;
    if (fabsf(gyro_z_rads) > ride_threshold_rads) {
        ya_yaw_rate_rads = gyro_z_rads;
    } else {
        ya_yaw_rate_rads = constrain_float(yaw_err_rad * THROW_YAW_SLEW_GAIN, -slew_cap_rads, slew_cap_rads);
    }
    attitude_control->input_thrust_vector_rate_heading_rads(thrust_vector, ya_yaw_rate_rads);
}

// True once yaw has settled on the absolute target: locked, within the done
// tolerance, AND no longer spinning faster than the ride threshold.  The spin
// gate matters because a fast throw spin (hundreds of deg/s) sweeps the yaw
// through the target every revolution; without it a transient sweep through
// the done window would be mistaken for alignment and hand off mid-spin.
bool ModeThrow::throw_yaw_converged() const
{
    return yaw_align_locked &&
           fabsf(wrap_PI(throw_target_yaw_rad - ahrs.get_yaw_rad())) <= radians(THROW_YAW_ALIGN_DONE_DEG) &&
           fabsf(ahrs.get_gyro().z) <= radians(THROW_YAW_RIDE_THRESH_DEG);
}

bool ModeThrow::throw_yaw_align_done() const
{
    // Permit the PosHold->NEXTMODE handoff once yaw has actually converged
    // on the absolute target (see throw_yaw_converged).  We cannot hand off
    // at the 30 deg catch-window lock: a heading-holding next-mode (LOITER)
    // freezes whatever heading we hand off at, so the remaining error becomes
    // permanent.  The adaptive timeout (sized to the rotation at HgtStabilise
    // entry) is the safety net for the case where the heading is never reached
    // or the spin never decays.
    if (!throw_target_yaw_valid ||
        (ThrowYawType)g2.throw_yaw_type.get() == ThrowYawType::None) {
        return true;
    }
    if (throw_yaw_converged()) {
        return true;
    }
    if (yaw_align_start_ms != 0 &&
        (AP_HAL::millis() - yaw_align_start_ms) > yaw_align_timeout_ms) {
        return true;
    }
    return false;
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
        case Mode::Number::STABILIZE:
        case Mode::Number::ALT_HOLD:
        case Mode::Number::ACRO:
            set_mode((Mode::Number)g2.throw_nextmode.get(), ModeReason::THROW_COMPLETE);
            break;
        default:
            // do nothing
            break;
    }
    nextmode_attempted = true;
}

#endif
