#include "Copter.h"
#include <utility>

#if MODE_FLOWHOLD_ENABLED

/*
  implement FLOWHOLD mode, for position hold using optical flow
  without rangefinder
 */

const AP_Param::GroupInfo ModeFlowHold::var_info[] = {
    // @Param: _XY_P
    // @DisplayName: FlowHold P gain
    // @Description: FlowHold (horizontal) P gain.
    // @Range: 0.1 6.0
    // @Increment: 0.1
    // @User: Advanced

    // @Param: _XY_I
    // @DisplayName: FlowHold I gain
    // @Description: FlowHold (horizontal) I gain
    // @Range: 0.02 1.00
    // @Increment: 0.01
    // @User: Advanced

    // @Param: _XY_IMAX
    // @DisplayName: FlowHold Integrator Max
    // @Description: FlowHold (horizontal) integrator maximum
    // @Range: 0 4500
    // @Increment: 10
    // @Units: cdeg
    // @User: Advanced

    // @Param: _XY_FILT_HZ
    // @DisplayName: FlowHold filter on input to control
    // @Description: FlowHold (horizontal) filter on input to control
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced
    AP_SUBGROUPINFO(flow_pi_xy, "_XY_",  1, ModeFlowHold, AC_PI_2D),

    // @Param: _FLOW_MAX
    // @DisplayName: FlowHold Flow Rate Max
    // @Description: Controls maximum apparent flow rate in flowhold
    // @Range: 0.1 2.5
    // @User: Standard
    AP_GROUPINFO("_FLOW_MAX", 2, ModeFlowHold, flow_max, 0.6),

    // @Param: _FILT_HZ
    // @DisplayName: FlowHold Filter Frequency
    // @Description: Filter frequency for flow data
    // @Range: 1 100
    // @Units: Hz
    // @User: Standard
    AP_GROUPINFO("_FILT_HZ", 3, ModeFlowHold, flow_filter_hz, 5),

    // @Param: _QUAL_MIN
    // @DisplayName: FlowHold Flow quality minimum
    // @Description: Minimum flow quality to use flow position hold
    // @Range: 0 255
    // @User: Standard
    AP_GROUPINFO("_QUAL_MIN", 4, ModeFlowHold, flow_min_quality, 10),

    // 5 was FLOW_SPEED

    // @Param: _BRAKE_RATE
    // @DisplayName: FlowHold Braking rate
    // @Description: Controls deceleration rate on stick release
    // @Range: 1 30
    // @User: Standard
    // @Units: deg/s
    AP_GROUPINFO("_BRAKE_RATE", 6, ModeFlowHold, brake_rate_dps, 8),

    AP_GROUPEND
};

ModeFlowHold::ModeFlowHold(void) : Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

#define CONTROL_FLOWHOLD_EARTH_FRAME 0

// flowhold_init - initialise flowhold controller
bool ModeFlowHold::init(bool ignore_checks)
{
    if (!copter.optflow.enabled() || !copter.optflow.healthy()) {
        return false;
    }

    // set vertical speed and acceleration limits
    pos_control->D_set_max_speed_accel_m(get_pilot_speed_dn_ms(), get_pilot_speed_up_ms(), get_pilot_accel_D_mss());
    pos_control->D_set_correction_speed_accel_m(get_pilot_speed_dn_ms(), get_pilot_speed_up_ms(), get_pilot_accel_D_mss());

    // initialise the vertical position controller
    if (!copter.pos_control->D_is_active()) {
        pos_control->D_init_controller();
    }

    flow_filter.set_cutoff_frequency(copter.scheduler.get_loop_rate_hz(), flow_filter_hz.get());

    quality_filtered = 0;
    flow_pi_xy.reset_I();
    limited = false;

    flow_pi_xy.set_dt(1.0/copter.scheduler.get_loop_rate_hz());

    // start with INS height
    last_ins_height_m = pos_control->get_pos_estimate_U_m();
    height_offset_m = 0;

    return true;
}

/*
  calculate desired attitude from flow sensor. Called when flow sensor is healthy
 */
void ModeFlowHold::flowhold_flow_to_angle(Vector2f &bf_angles_rad, bool stick_input)
{
    uint32_t now = AP_HAL::millis();
    const float angle_max_rad = cd_to_rad(copter.aparm.angle_max);

    // get corrected raw flow rate
    Vector2f raw_flow_rads = copter.optflow.flowRate() - copter.optflow.bodyRate();

    // limit sensor flow, this prevents oscillation at low altitudes
    raw_flow_rads.x = constrain_float(raw_flow_rads.x, -flow_max, flow_max);
    raw_flow_rads.y = constrain_float(raw_flow_rads.y, -flow_max, flow_max);

    // filter the flow rate
    Vector2f sensor_flow_rads = flow_filter.apply(raw_flow_rads);

    // scale by height estimate, limiting it to height_min_m to height_max
    float ins_height_m = pos_control->get_pos_estimate_U_m();
    float height_estimate_m = ins_height_m + height_offset_m;

    // compensate for height, this converts to (approx) m/s
    Vector2f sensor_flow_ms = sensor_flow_rads * constrain_float(height_estimate_m, height_min_m, height_max);

    // rotate controller input to earth frame
    Vector2f input_ne_ms = copter.ahrs.body_to_earth2D(sensor_flow_ms);

    // run PI controller
    flow_pi_xy.set_input(input_ne_ms);

    // get the normalised earth frame controller attitude
    Vector2f ef_output;

    // get P term
    ef_output = flow_pi_xy.get_p();

    if (stick_input) {
        last_stick_input_ms = now;
        braking = true;
    }
    if (!stick_input && braking) {
        // stop braking if either 3s has passed, or we have slowed below 0.3m/s
        if (now - last_stick_input_ms > 3000 || sensor_flow_ms.length() < 0.3) {
            braking = false;
#if 0
            printf("braking done at %u vel=%f\n", now - last_stick_input_ms,
                   (double)sensor_flow_ms.length());
#endif
        }
    }

    if (!stick_input && !braking) {
        // get I term
        if (limited) {
            // only allow I term to shrink in length
            xy_I = flow_pi_xy.get_i_shrink();
        } else {
            // normal I term operation
            xy_I = flow_pi_xy.get_pi();
        }
    }

    if (!stick_input && braking) {
        // calculate brake angle for each axis separately
        for (uint8_t i=0; i<2; i++) {
            float &velocity = sensor_flow_ms[i];
            float abs_vel_ms = fabsf(velocity);
            const float brake_gain = (15.0f * brake_rate_dps.get() + 95.0f) * 0.01f;
            float lean_angle_rad = radians(brake_gain * abs_vel_ms * (1.0f + 5.0f/(abs_vel_ms + 0.60f)));
            if (velocity < 0) {
                lean_angle_rad = -lean_angle_rad;
            }
            bf_angles_rad[i] = lean_angle_rad;
        }
        ef_output.zero();
    }

    ef_output += xy_I;
    Vector2f ef_output_rad = ef_output * angle_max_rad;

    // convert to body frame
    bf_angles_rad += copter.ahrs.earth_to_body2D(ef_output_rad);

    // set limited flag to prevent integrator windup
    limited = fabsf(bf_angles_rad.x) > angle_max_rad || fabsf(bf_angles_rad.y) > angle_max_rad;

    // constrain to angle limit
    bf_angles_rad.x = constrain_float(bf_angles_rad.x, -angle_max_rad, angle_max_rad);
    bf_angles_rad.y = constrain_float(bf_angles_rad.y, -angle_max_rad, angle_max_rad);

#if HAL_LOGGING_ENABLED
// @LoggerMessage: FHLD
// @Description: FlowHold mode messages
// @URL: https://ardupilot.org/copter/docs/flowhold-mode.html
// @Field: TimeUS: Time since system startup
// @Field: SFx: Filtered flow rate, X-Axis
// @Field: SFy: Filtered flow rate, Y-Axis
// @Field: Ax: Target lean angle, X-Axis
// @Field: Ay: Target lean angle, Y-Axis
// @Field: Qual: Flow sensor quality. If this value falls below FHLD_QUAL_MIN parameter, FlowHold will act just like AltHold.
// @Field: Ix: Integral part of PI controller, X-Axis
// @Field: Iy: Integral part of PI controller, Y-Axis

    if (log_counter++ % 20 == 0) {
        AP::logger().WriteStreaming("FHLD", "TimeUS,SFx,SFy,Ax,Ay,Qual,Ix,Iy", "Qfffffff",
                                               AP_HAL::micros64(),
                                               (double)sensor_flow_ms.x, (double)sensor_flow_ms.y,
                                               (double)rad_to_cd(bf_angles_rad.x), (double)rad_to_cd(bf_angles_rad.y),
                                               (double)quality_filtered,
                                               (double)xy_I.x, (double)xy_I.y);
    }
#endif  // HAL_LOGGING_ENABLED
}

// flowhold_run - runs the flowhold controller
// should be called at 100hz or more
void ModeFlowHold::run()
{
    update_height_estimate();

    // set vertical speed and acceleration limits
    pos_control->D_set_max_speed_accel_m(get_pilot_speed_dn_ms(), get_pilot_speed_up_ms(), get_pilot_accel_D_mss());

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // check for filter change
    if (!is_equal(flow_filter.get_cutoff_freq(), flow_filter_hz.get())) {
        flow_filter.set_cutoff_frequency(copter.scheduler.get_loop_rate_hz(), flow_filter_hz.get());
    }

    // get pilot desired climb rate
    float target_climb_rate_ms = copter.get_pilot_desired_climb_rate_ms();
    target_climb_rate_ms = constrain_float(target_climb_rate_ms, -get_pilot_speed_dn_ms(), get_pilot_speed_up_ms());

    // get pilot's desired yaw rate
    float target_yaw_rate_rads = get_pilot_desired_yaw_rate_rads();

    // Flow Hold State Machine Determination
    AltHoldModeState flowhold_state = get_alt_hold_state_D_ms(target_climb_rate_ms);

    if (copter.optflow.healthy()) {
        const float filter_constant = 0.95;
        quality_filtered = filter_constant * quality_filtered + (1-filter_constant) * copter.optflow.quality();
    } else {
        quality_filtered = 0;
    }

    // Flow Hold State Machine
    switch (flowhold_state) {

    case AltHoldModeState::MotorStopped:
        copter.motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        copter.attitude_control->reset_rate_controller_I_terms();
        copter.attitude_control->reset_yaw_target_and_rate();
        copter.pos_control->D_relax_controller(0.0f);   // forces throttle output to decay to zero
        flow_pi_xy.reset_I();
        break;

    case AltHoldModeState::Takeoff:
        // set motors to full range
        copter.motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start_m(constrain_float(g.pilot_takeoff_alt_cm * 0.01, 0.0, 10.0));
        }

        // get avoidance adjusted climb rate
        target_climb_rate_ms = get_avoidance_adjusted_climbrate_ms(target_climb_rate_ms);

        // set position controller targets adjusted for pilot input
        takeoff.do_pilot_takeoff_ms(target_climb_rate_ms);
        break;

    case AltHoldModeState::Landed_Ground_Idle:
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case AltHoldModeState::Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pos_control->D_relax_controller(0.0f);   // forces throttle output to decay to zero
        break;

    case AltHoldModeState::Flying:
        copter.motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // get avoidance adjusted climb rate
        target_climb_rate_ms = get_avoidance_adjusted_climbrate_ms(target_climb_rate_ms);

#if AP_RANGEFINDER_ENABLED
        // update the vertical offset based on the surface measurement
        copter.surface_tracking.update_surface_offset();
#endif

        // Send the commanded climb rate to the position controller
        pos_control->D_set_pos_target_from_climb_rate_ms(target_climb_rate_ms);
        break;
    }

    // flowhold attitude target calculations
    Vector2f bf_angles_rad;

    // calculate alt-hold angles
    int16_t roll_in = copter.channel_roll->get_control_in();
    int16_t pitch_in = copter.channel_pitch->get_control_in();
    const float angle_max_rad = cd_to_rad(copter.aparm.angle_max);

    float target_roll_rad, target_pitch_rad;
    get_pilot_desired_lean_angles_rad(target_roll_rad, target_pitch_rad, attitude_control->lean_angle_max_rad(), attitude_control->get_althold_lean_angle_max_rad());
    bf_angles_rad.x = target_roll_rad;
    bf_angles_rad.y = target_pitch_rad;

    if (quality_filtered >= flow_min_quality &&
        AP_HAL::millis() - copter.arm_time_ms > 3000) {
        // don't use for first 3s when we are just taking off
        Vector2f flow_angles;

        flowhold_flow_to_angle(flow_angles, (roll_in != 0) || (pitch_in != 0));
        flow_angles.x = constrain_float(flow_angles.x, -angle_max_rad/2, angle_max_rad/2);
        flow_angles.y = constrain_float(flow_angles.y, -angle_max_rad/2, angle_max_rad/2);
        bf_angles_rad += flow_angles;
    }
    bf_angles_rad.x = constrain_float(bf_angles_rad.x, -angle_max_rad, angle_max_rad);
    bf_angles_rad.y = constrain_float(bf_angles_rad.y, -angle_max_rad, angle_max_rad);

#if AP_AVOIDANCE_ENABLED
    // apply avoidance
    copter.avoid.adjust_roll_pitch_rad(bf_angles_rad.x, bf_angles_rad.y, attitude_control->lean_angle_max_rad());
#endif

    // call attitude controller
    copter.attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(bf_angles_rad.x, bf_angles_rad.y, target_yaw_rate_rads);

    // run the vertical position controller and set output throttle
    pos_control->D_update_controller();
}

/*
  update height estimate using integrated accelerometer ratio with optical flow
 */
void ModeFlowHold::update_height_estimate(void)
{
    float ins_height_m = copter.pos_control->get_pos_estimate_U_m();

#if 1
    // assume on ground when disarmed, or if we have only just started spooling the motors up
    if (!hal.util->get_soft_armed() ||
        copter.motors->get_desired_spool_state() != AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED ||
        AP_HAL::millis() - copter.arm_time_ms < 1500) {
        height_offset_m = -ins_height_m;
        last_ins_height_m = ins_height_m;
        return;
    }
#endif

    // get delta velocity in body frame
    Vector3f delta_vel_ms;
    float delta_vel_dt;
    if (!copter.ins.get_delta_velocity(delta_vel_ms, delta_vel_dt)) {
        return;
    }

    // integrate delta velocity in earth frame
    const Matrix3f &rotMat = copter.ahrs.get_rotation_body_to_ned();
    delta_vel_ms = rotMat * delta_vel_ms;
    delta_velocity_ne_ms.x += delta_vel_ms.x;
    delta_velocity_ne_ms.y += delta_vel_ms.y;

    if (!copter.optflow.healthy()) {
        // can't update height model with no flow sensor
        last_flow_ms = AP_HAL::millis();
        delta_velocity_ne_ms.zero();
        return;
    }

    if (last_flow_ms == 0) {
        // just starting up
        last_flow_ms = copter.optflow.last_update();
        delta_velocity_ne_ms.zero();
        height_offset_m = 0;
        return;
    }

    if (copter.optflow.last_update() == last_flow_ms) {
        // no new flow data
        return;
    }

    // convert delta velocity back to body frame to match the flow sensor
    Vector2f delta_vel_bf_ms = copter.ahrs.earth_to_body2D(delta_velocity_ne_ms);

    // and convert to an rate equivalent, to be comparable to flow
    Vector2f delta_vel_rate_ms(-delta_vel_bf_ms.y, delta_vel_bf_ms.x);

    // get body flow rate in radians per second
    Vector2f flow_rate_rads = copter.optflow.flowRate() - copter.optflow.bodyRate();

    uint32_t dt_ms = copter.optflow.last_update() - last_flow_ms;
    if (dt_ms > 500) {
        // too long between updates, ignore
        last_flow_ms = copter.optflow.last_update();
        delta_velocity_ne_ms.zero();
        last_flow_rate_rads = flow_rate_rads;
        last_ins_height_m = ins_height_m;
        height_offset_m = 0;
        return;        
    }

    /*
      basic equation is:
      height_m = delta_velocity_ms / delta_flowrate_rads;
     */

    // get delta_flowrate_rps
    Vector2f delta_flowrate_rads = flow_rate_rads - last_flow_rate_rads;
    last_flow_rate_rads = flow_rate_rads;
    last_flow_ms = copter.optflow.last_update();

    /*
      update height estimate
     */
    const float min_velocity_change_ms = 0.04;
    const float min_flow_change = 0.04;
    const float height_delta_max_m = 0.25;

    /*
      for each axis update the height estimate
     */
    float delta_height_m = 0;
    uint8_t total_weight = 0;
    float height_estimate_m = ins_height_m + height_offset_m;

    for (uint8_t i=0; i<2; i++) {
        // only use height estimates when we have significant delta-velocity and significant delta-flow
        float abs_flow = fabsf(delta_flowrate_rads[i]);
        if (abs_flow < min_flow_change ||
            fabsf(delta_vel_rate_ms[i]) < min_velocity_change_ms) {
            continue;
        }
        // get instantaneous height estimate
        float height_m = delta_vel_rate_ms[i] / delta_flowrate_rads[i];
        if (height_m <= 0) {
            // discard negative heights
            continue;
        }
        delta_height_m += (height_m - height_estimate_m) * abs_flow;
        total_weight += abs_flow;
    }
    if (total_weight > 0) {
        delta_height_m /= total_weight;
    }

    if (delta_height_m < 0) {
        // bias towards lower heights, as we'd rather have too low
        // gain than have oscillation. This also compensates a bit for
        // the discard of negative heights above
        delta_height_m *= 2;
    }

    // don't update height by more than height_delta_max_m, this is a simple way of rejecting noise
    float new_offset_m = height_offset_m + constrain_float(delta_height_m, -height_delta_max_m, height_delta_max_m);

    // apply a simple filter
    height_offset_m = 0.8 * height_offset_m + 0.2 * new_offset_m;

    if (ins_height_m + height_offset_m < height_min_m) {
        // height estimate is never allowed below the minimum
        height_offset_m = height_min_m - ins_height_m;
    }

    // new height estimate for logging
    height_estimate_m = ins_height_m + height_offset_m;

#if HAL_LOGGING_ENABLED
// @LoggerMessage: FHXY
// @Description: Height estimation using optical flow sensor 
// @Field: TimeUS: Time since system startup
// @Field: DFx: Delta flow rate, X-Axis
// @Field: DFy: Delta flow rate, Y-Axis
// @Field: DVx: Integrated delta velocity rate, X-Axis
// @Field: DVy: Integrated delta velocity rate, Y-Axis
// @Field: Hest: Estimated Height
// @Field: DH: Delta Height
// @Field: Hofs: Height offset
// @Field: InsH: Height estimate from inertial navigation library
// @Field: LastInsH: Last used INS height in optical flow sensor height estimation calculations 
// @Field: DTms: Time between optical flow sensor updates. This should be less than 500ms for performing the height estimation calculations

    AP::logger().WriteStreaming("FHXY", "TimeUS,DFx,DFy,DVx,DVy,Hest,DH,Hofs,InsH,LastInsH,DTms", "QfffffffffI",
                                           AP_HAL::micros64(),
                                           (double)delta_flowrate_rads.x,
                                           (double)delta_flowrate_rads.y,
                                           (double)delta_vel_rate_ms.x,
                                           (double)delta_vel_rate_ms.y,
                                           (double)height_estimate_m,
                                           (double)delta_height_m,
                                           (double)height_offset_m,
                                           (double)ins_height_m,
                                           (double)last_ins_height_m,
                                           dt_ms);
#endif

    gcs().send_named_float("HEST", height_estimate_m);
    delta_velocity_ne_ms.zero();
    last_ins_height_m = ins_height_m;
}

#endif // MODE_FLOWHOLD_ENABLED
