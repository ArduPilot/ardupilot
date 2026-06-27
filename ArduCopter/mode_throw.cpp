#include "Copter.h"

#if MODE_THROW_ENABLED

// Physics-based body-frame freefall ceiling for drops.  Under spin at
// the IMU mount offset r, centripetal acceleration w^2*r projects into
// the body-frame |a| reading during genuine freefall.  Bounding r <= 6
// cm (covers typical FC stacks) gives cap = 0.5g freefall margin +
// r*w^2, which reduces to the standard 0.5g gate at w=0.  A stationary
// carrier (|a|~1g, w=0) is always rejected.
//
// Replaces a fixed 1.5g ceiling that failed under multi-axis tumble:
// SFD1 log55 (2026-05-27) had combined gyro 25-30 rad/s for ~600 ms
// post-release, putting body |a| at 15-30 m/s^2 (1.5-3g) throughout
// genuine freefall.  The 1.5g cap rejected the entire window; detection
// stalled ~900 ms until spin decayed enough for the 0.5g base gate to
// fire, costing ~3.6 m of altitude budget.  The physics cap admits the
// same window: at w=30 rad/s cap=6g, at w=50 cap=15.8g.
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
    source_set_switched = false;
    xy_controller_active = false;
    drop_confirm_start_ms = 0;
    drop_release_alt_m = 0;
    last_stage_msg_ms = 0;
    yaw_align_start_ms = 0;
    yaw_align_timeout_ms = THROW_YAW_ALIGN_TIMEOUT_MS;
    yaw_align_locked = false;

    // Capture EKF state for the throw-direction fallback chain before
    // SRC_INI switches the source set.  On a moving carrier the IMU
    // integrator can't see the inherited velocity (built up over
    // seconds of steady flight that the held-still reset zeroes out),
    // so the entry velocity is the only horizontal-direction signal
    // available for that case.  When the vehicle is stationary at
    // entry (hand-held, hovering carrier) entry velocity is also
    // unhelpful, so we additionally capture entry yaw -- that's the
    // operator/mount-aligned "forward" direction and is always a
    // reasonable target if no motion signal arrives.  Captured
    // regardless of THROW_YAW_TYPE; cheap, only consulted later.
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

    // Switch EKF source set for the throw phase if configured.
    // During throw/drop the vehicle is tumbling -- position and velocity
    // sources (optical flow, GPS) produce garbage.  Switching to a
    // source set with no horizontal aiding prevents EKF variance growth
    // and nuisance EKF failsafes.  A completed throw restores THROW_SRC_SET
    // at THROW_COMPLETE; exit() restores the pre-throw set on any other exit
    // so an aborted/never-thrown throw cannot leave the source set stuck.
    const int8_t src_init = g2.throw_src_init.get();
    if (src_init >= 1 && src_init <= 3) {
        saved_source_set = AP::ahrs().get_posvelyaw_source_set();
        source_set_switched = true;
        AP::ahrs().set_posvelyaw_source_set(AP_NavEKF_Source::SourceSetSelection(src_init - 1));
        gcs().send_text(MAV_SEVERITY_INFO, "Throw: EKF Source Set %d", src_init);
    }

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

// Restore the EKF source set that was active before init() switched it for
// the throw phase.  A completed throw restores THROW_SRC_SET in
// throw_do_nextmode_handoff() and clears source_set_switched first, so this
// only fires when throw is left without completing (pilot or failsafe
// switches mode while still waiting for / recovering from the throw), which
// would otherwise leave the vehicle stuck on the throw source set for the
// rest of the power cycle.
void ModeThrow::exit()
{
    if (source_set_switched) {
        AP::ahrs().set_posvelyaw_source_set(AP_NavEKF_Source::SourceSetSelection(saved_source_set));
        gcs().send_text(MAV_SEVERITY_INFO, "Throw: restored EKF Source Set %d", saved_source_set + 1);
        source_set_switched = false;
    }
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
        stage = Throw_Detecting;

    } else if (stage == Throw_Detecting && throw_detected()){
        copter.set_land_complete(false);
        stage = Throw_Wait_Throttle_Unlimited;

        // Cancel the waiting for throw tone sequence
        AP_Notify::flags.waiting_for_throw = false;

        // Lock in the Uprighting yaw target now, while the IMU
        // integrator state still reflects the throw motion.  Stashed
        // for use when Uprighting begins.
        (void)throw_dir_finalise_target_yaw();

    } else if (stage == Throw_Wait_Throttle_Unlimited &&
               !throw_in_freefall()) {
        // Freefall lost during spool-up -- the vehicle is no longer in
        // freefall (e.g. carrier bounce settled, or false trigger from
        // turbulence).  Throttle is zero during this stage so accel is
        // a clean indicator.  Abort back to Detecting and spool down.
        gcs().send_text(MAV_SEVERITY_WARNING, "Throw: freefall lost, resetting");
        stage = Throw_Detecting;
        drop_confirm_start_ms = 0;
        free_fall_start_ms = 0;
        AP_Notify::flags.waiting_for_throw = true;

    } else if (stage == Throw_Wait_Throttle_Unlimited &&
               motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
        stage = Throw_Uprighting;
        uprighting_start_ms = AP_HAL::millis();
    } else if (stage == Throw_Uprighting && throw_uprighting_complete() && throw_drop_distance_reached()) {
        stage = Throw_HgtStabilise;
        hgt_stabilise_start_ms = AP_HAL::millis();
        yaw_align_start_ms = hgt_stabilise_start_ms;
        yaw_align_locked = false;

        // Size the yaw-alignment timeout to the rotation actually required
        // at the configured slew rate (ATC_SLEW_YAW).  A large heading
        // change -- up to a 180 deg half-turn -- needs more than the base
        // budget; with a fixed timeout the slew is cut off short and a
        // heading-holding next-mode (LOITER) freezes the partial heading.
        // Floored at the base timeout and capped so a very slow yaw tune (or
        // a spin that never decays) cannot stall the handoff for tens of
        // seconds -- at the floor of ATC_RATE_WPY_MAX a 180 deg slew alone
        // would otherwise size the timeout at ~37 s.
        yaw_align_timeout_ms = THROW_YAW_ALIGN_TIMEOUT_MS;
        if (throw_target_yaw_valid) {
            const float slew_rads = attitude_control->get_slew_yaw_max_rads();
            if (is_positive(slew_rads)) {
                const float err_rad = fabsf(wrap_PI(throw_target_yaw_rad - ahrs.get_yaw_rad()));
                const uint32_t needed_ms = (uint32_t)(err_rad / slew_rads * 1000.0f) + THROW_YAW_ALIGN_MARGIN_MS;
                yaw_align_timeout_ms = MAX((uint32_t)THROW_YAW_ALIGN_TIMEOUT_MS, needed_ms);
            }
        }
        yaw_align_timeout_ms = MIN(yaw_align_timeout_ms, (uint32_t)THROW_YAW_ALIGN_TIMEOUT_MAX_MS);

        // initialise the z controller
        pos_control->D_init_controller_no_descent();

        // initialise the demanded height below/above the throw height from user parameters
        // this allows for rapidly clearing surrounding obstacles
        if (g2.throw_type == ThrowType::Drop) {
            // Target altitude is THROW_ALT_DCSND below the release point.
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
                                         nextmode != Mode::Number::ALT_HOLD &&
                                         nextmode != Mode::Number::ACRO);
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

        // Level roll/pitch via thrust vector + zero yaw rate target.
        // Thrust-vector control gives the same shortest-path levelling
        // from any starting orientation (including inverted) as a
        // quaternion call.  Holding only yaw rate=0 lets the rate
        // controller damp residual spin without an absolute yaw target
        // piling on attitude-error torque -- that fight was aggressive
        // on heavy-spin drops.  THROW_YAW_TYPE is applied during
        // HgtStabilise and PosHold by throw_apply_yaw_align(), where it
        // runs concurrently with the height/position settle.
        const Vector3f thrust_vec_up{0.0f, 0.0f, -1.0f};
        attitude_control->input_thrust_vector_rate_heading_rads(thrust_vec_up, 0.0f);

        // For drops, command zero throttle and let ATC_THR_MIX_MAX
        // (auto-enabled at >30deg attitude error) provide differential
        // thrust for attitude control only.  This prevents the vehicle
        // from climbing back toward the carrier after arrest -- the
        // aggressive descent arrest is handled later by the position
        // controller in HgtStabilise using THROW_DROP_AG.
        // For upward throws use 50% without boost to maximise righting moment.
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

        // Run yaw alignment concurrently with the height arrest.  Level
        // is provided via thrust vector; yaw is ridden, slewed, or
        // locked depending on residual rate and error to target.  When
        // THROW_YAW_TYPE is None the helper falls back to yaw rate=0.
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

    // Check for freefall.  For drops use body-frame accelerometer as the
    // primary check -- it reads near zero in freefall regardless of EKF
    // state.  Under spin, centripetal force w^2*r at the IMU offset
    // inflates body |a|; the drop_body_in_freefall() ceiling scales
    // with w^2 to admit the spinning-freefall envelope (see helper at
    // top of file for cases driving the design).  As a final fallback
    // check earth-frame Z: centripetal is horizontal in body XY so
    // earth-Z reads ~0 if the spin axis is vertical, but oscillates
    // under multi-axis tumble.  For upward throws keep the existing
    // earth-frame check.
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

    // For drops, require BOTH time and freefall distance to confirm.
    // The time check (THROW_DROP_CNF) rejects transient low-g events.
    // The distance check (0.5*g*t^2 for the confirmation time) cross-
    // validates that the vehicle actually fell the expected distance,
    // not just sustained low-g on a smooth-flying carrier.
    // THROW_ALT_DCSND is checked separately at the Uprighting to
    // HgtStabilise transition, allowing the vehicle to spool up and
    // hold level attitude in a controlled descent before arresting.
    // Additional false-trigger protection is provided by the spool-up
    // freefall verification in Wait_Throttle_Unlimited.
    if (g2.throw_type == ThrowType::Drop) {
        if (possible_throw_detected) {
            if (drop_confirm_start_ms == 0) {
                drop_confirm_start_ms = AP_HAL::millis();
                drop_release_alt_m = pos_control->get_pos_estimate_U_m();
            }
            const uint32_t confirm_ms = MAX((uint32_t)THROW_DROP_CONFIRM_MS,
                                            (uint32_t)(g2.throw_drop_confirm_time * 1000.0f));
            const bool time_confirmed = (AP_HAL::millis() - drop_confirm_start_ms >= confirm_ms);
            // Cross-check: vehicle should have fallen the expected
            // freefall distance for the confirmation time.
            const float confirm_s = confirm_ms * 0.001f;
            const float confirm_dist = 0.5f * GRAVITY_MSS * confirm_s * confirm_s;
            const float t = (AP_HAL::millis() - drop_confirm_start_ms) * 0.001f;
            const bool fall_confirmed = (0.5f * GRAVITY_MSS * t * t >= confirm_dist);
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

void ModeThrow::throw_dir_reset()
{
    // Reset the IMU throw-direction integrator.  Called from init()
    // and whenever the vehicle is "held still" during Detecting (body
    // accel close to gravity, gyro low) -- the integration that follows
    // captures motion since this reset, so the most recent reset is
    // the one that anchors the velocity vector at release.
    throw_dir_q.initialise();
    throw_dir_q_valid = false;
    throw_dir_vel_ne_ms.zero();
    throw_dir_last_us = 0;
    throw_dir_anchor_yaw_valid = false;
    throw_dir_anchor_yaw_rad = 0.0f;
}

void ModeThrow::throw_dir_update()
{
    // EKF-independent throw-direction integration.  Maintains a body-
    // to-pseudo-earth quaternion using gyro integration only, starting
    // from the gravity vector at the most recent stationary sample.
    // Pseudo-earth is NED-aligned (Z-down) but with arbitrary yaw --
    // good enough to extract a horizontal velocity vector whose heading
    // can be anchored to true NED via a single EKF yaw read at release.
    //
    // Only runs in Throw_Detecting, so motors are off and body accel
    // is uncontaminated by motor thrust.
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
        // Reset and re-anchor attitude from current gravity reading.
        // Pseudo-earth Z is the gravity direction (down); X/Y are
        // arbitrary -- body-yaw at this moment maps to pseudo-yaw=0,
        // so we capture the EKF yaw here and use it as the anchor when
        // converting the pseudo-earth heading back to NED at the
        // freefall transition.  Body specific force at rest is
        // approximately -gravity_body, so the gravity unit vector in
        // body frame is -accel/|accel|; roll and pitch are extracted
        // from it (yaw left at zero).
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

    // Rotate body specific force into pseudo-earth, then add gravity
    // (NED Z-down: gravity vector is +g in Z) to recover the vehicle's
    // actual acceleration in pseudo-earth.  Specific force at rest is
    // (0,0,-g), so a + g = 0 -> vehicle isn't accelerating.  In freefall
    // a = (0,0,+g) -> vehicle accelerates downward at g.
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
    // Use EKF altitude if the filter has a vertical position estimate
    // AND velocity aiding.  Without velocity aiding (const_pos_mode)
    // the EKF velocity drifts from accelerometer integration, making
    // the altitude estimate unreliable during dynamic maneuvers -- on a
    // carrier the velocity can drift to 70+ m/s while stationary,
    // causing the altitude to overshoot the release point and the
    // distance check to fail permanently.
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
    // Yaw alignment runs in HgtStabilise and PosHold concurrently with
    // the height/position settle, so it adds no wall-clock time on the
    // common path.  Three regimes:
    //  - No target configured: command yaw rate=0 (legacy damping).
    //  - Outside the catch window:
    //      * heavy residual spin (>slew cap): track current gyro_z so
    //        no torque is applied -- natural drag and the rate
    //        controller's small damping carry yaw toward target.
    //      * low rate: rate-limited proportional slew toward target.
    //  - Inside the catch window AND rate manageable: lock to absolute
    //    target via input_thrust_vector_heading_rad so position-keeping
    //    tilt (when caller supplies it) is preserved.
    if (!throw_target_yaw_valid ||
        (ThrowYawType)g2.throw_yaw_type.get() == ThrowYawType::None) {
        attitude_control->input_thrust_vector_rate_heading_rads(thrust_vector, 0.0f);
        return;
    }

    const float yaw_err_rad = wrap_PI(throw_target_yaw_rad - ahrs.get_yaw_rad());
    const float gyro_z_rads = ahrs.get_gyro().z;
    const float catch_window_rad = radians(THROW_YAW_CATCH_WINDOW_DEG);
    // Slew cap comes from the attitude controller's own slew yaw max
    // (ATC_RATE_WPY_MAX, optionally further limited by ATC_RATE_Y_MAX),
    // so a single parameter governs both our commanded rate and the
    // controller's internal rate limiter.  Bumping ATC_RATE_WPY_MAX
    // makes the entire approach + lock phase faster.
    const float slew_cap_rads = attitude_control->get_slew_yaw_max_rads();
    const float ride_threshold_rads = radians(THROW_YAW_RIDE_THRESH_DEG);

    // Lock the moment yaw enters the catch window.  The lock command
    // (input_thrust_vector_heading_rad) is rate-limited by the same
    // get_slew_yaw_max_rads() so the post-lock convergence runs at the
    // same cap the slew was using.  Only announce alignment when the spin
    // is below the ride threshold: a fast spin can sweep yaw through the
    // catch window and latch the lock for one loop, which is not a genuine
    // alignment -- suppress the message there to avoid false "aligned"
    // telemetry (locking behaviour itself is unchanged).
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

    // Outside the catch window -- keep approaching target.  Ride mode is
    // reserved for genuinely heavy spin where drag decay is meaningful;
    // a moderate steady spin (below the ride threshold) gets active
    // decel via the slew path so we don't sit idle waiting for natural
    // rotation.  The gain keeps the slew cap saturated until the error
    // drops below slew_cap/gain, so the approach maintains max rate up
    // to the lock window and decelerates just before lock.
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
    const int8_t srcset = g2.throw_srcset.get();
    if (srcset >= 1 && srcset <= 3) {
        AP::ahrs().set_posvelyaw_source_set(AP_NavEKF_Source::SourceSetSelection(srcset - 1));
        gcs().send_text(MAV_SEVERITY_INFO, "EKF Source Set %d", srcset);
    }
    // The completion path owns the post-throw source set; stop exit() from
    // restoring the pre-throw set over it.  set_mode() below calls exit()
    // synchronously, so this must be cleared before the mode change.
    source_set_switched = false;
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
#if MODE_VALT_ENABLED
        case Mode::Number::VALT:
#endif
            set_mode((Mode::Number)g2.throw_nextmode.get(), ModeReason::THROW_COMPLETE);
            break;
        default:
            // do nothing
            break;
    }
    nextmode_attempted = true;
}

#endif
