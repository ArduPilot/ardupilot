#include "Sub.h"

/*
 * Init and run calls for guided flight mode
 */

#define GUIDED_POSVEL_TIMEOUT_MS    3000    // guided mode's position-velocity controller times out after 3seconds with no new updates
#define GUIDED_ATTITUDE_TIMEOUT_MS  1000    // guided mode's attitude controller times out after 1 second with no new updates

static Vector3p posvel_pos_target_cm;
static Vector3f posvel_vel_target_cms;
static uint32_t update_time_ms;

struct {
    uint32_t update_time_ms;
    float roll_cd;
    float pitch_cd;
    float yaw_cd;
    float climb_rate_cms;
} static guided_angle_state = {0,0.0f, 0.0f, 0.0f, 0.0f};

struct Guided_Limit {
    uint32_t timeout_ms;  // timeout (in seconds) from the time that guided is invoked
    float alt_min_cm;   // lower altitude limit in cm above home (0 = no limit)
    float alt_max_cm;   // upper altitude limit in cm above home (0 = no limit)
    float horiz_max_cm; // horizontal position limit in cm from where guided mode was initiated (0 = no limit)
    uint32_t start_time;// system time in milliseconds that control was handed to the external computer
    Vector3f start_pos; // start position as a distance from home in cm.  used for checking horiz_max limit
} guided_limit;

// guided_init - initialise guided controller
bool ModeGuided::init(bool ignore_checks)
{
    if (!sub.position_ok() && !ignore_checks) {
        return false;
    }

    // start in position control mode
    guided_pos_control_start();
    return true;
}

// get_default_auto_yaw_mode - returns auto_yaw_mode based on WP_YAW_BEHAVIOR parameter
// set rtl parameter to true if this is during an RTL
autopilot_yaw_mode ModeGuided::get_default_auto_yaw_mode(bool rtl) const
{
    switch (g.wp_yaw_behavior) {

    case WP_YAW_BEHAVIOR_NONE:
        return AUTO_YAW_HOLD;
        break;

    case WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP_EXCEPT_RTL:
        if (rtl) {
            return AUTO_YAW_HOLD;
        } else {
            return AUTO_YAW_LOOK_AT_NEXT_WP;
        }
        break;

    case WP_YAW_BEHAVIOR_LOOK_AHEAD:
        return AUTO_YAW_LOOK_AHEAD;
        break;

    case WP_YAW_BEHAVIOR_CORRECT_XTRACK:
        return AUTO_YAW_CORRECT_XTRACK;
        break;

    case WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP:
    default:
        return AUTO_YAW_LOOK_AT_NEXT_WP;
        break;
    }
}


// initialise guided mode's position controller
void ModeGuided::guided_pos_control_start()
{
    // set to position control mode
    sub.guided_mode = Guided_WP;

    // initialise waypoint controller
    sub.wp_nav.wp_and_spline_init();

    // initialise wpnav to stopping point at current altitude
    // To-Do: set to current location if disarmed?
    // To-Do: set to stopping point altitude?
    Vector3f stopping_point;
    sub.wp_nav.get_wp_stopping_point(stopping_point);

    // no need to check return status because terrain data is not used
    sub.wp_nav.set_wp_destination(stopping_point, false);

    // initialise yaw
    set_auto_yaw_mode(get_default_auto_yaw_mode(false));
}

// initialise guided mode's velocity controller
void ModeGuided::guided_vel_control_start()
{
    // set guided_mode to velocity controller
    sub.guided_mode = Guided_Velocity;

    // initialize vertical maximum speeds and acceleration
    position_control->set_max_speed_accel_z(-sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    position_control->set_correction_speed_accel_z(-sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // initialise velocity controller
    position_control->init_z_controller();
    position_control->init_xy_controller();

    // pilot always controls yaw
    set_auto_yaw_mode(AUTO_YAW_HOLD);
}

// initialise guided mode's posvel controller
void ModeGuided::guided_posvel_control_start()
{
    // set guided_mode to velocity controller
    sub.guided_mode = Guided_PosVel;

    // set vertical speed and acceleration
    position_control->set_max_speed_accel_z(sub.wp_nav.get_default_speed_down(), sub.wp_nav.get_default_speed_up(), sub.wp_nav.get_accel_z());
    position_control->set_correction_speed_accel_z(sub.wp_nav.get_default_speed_down(), sub.wp_nav.get_default_speed_up(), sub.wp_nav.get_accel_z());

    // initialise velocity controller
    position_control->init_z_controller();
    position_control->init_xy_controller();

    // pilot always controls yaw
    set_auto_yaw_mode(AUTO_YAW_HOLD);
}

// initialise guided mode's angle controller
void ModeGuided::guided_angle_control_start()
{
    // set guided_mode to velocity controller
    sub.guided_mode = Guided_Angle;

    // set vertical speed and acceleration
    position_control->set_max_speed_accel_z(sub.wp_nav.get_default_speed_down(), sub.wp_nav.get_default_speed_up(), sub.wp_nav.get_accel_z());
    position_control->set_correction_speed_accel_z(sub.wp_nav.get_default_speed_down(), sub.wp_nav.get_default_speed_up(), sub.wp_nav.get_accel_z());

    // initialise velocity controller
    position_control->init_z_controller();

    // initialise targets
    guided_angle_state.update_time_ms = AP_HAL::millis();
    guided_angle_state.roll_cd = ahrs.roll_sensor;
    guided_angle_state.pitch_cd = ahrs.pitch_sensor;
    guided_angle_state.yaw_cd = ahrs.yaw_sensor;
    guided_angle_state.climb_rate_cms = 0.0f;

    // pilot always controls yaw
    set_auto_yaw_mode(AUTO_YAW_HOLD);
}

// guided_set_destination - sets guided mode's target destination
// Returns true if the fence is enabled and guided waypoint is within the fence
// else return false if the waypoint is outside the fence
bool ModeGuided::guided_set_destination(const Vector3f& destination)
{
    // ensure we are in position control mode
    if (sub.guided_mode != Guided_WP) {
        guided_pos_control_start();
    }

#if AP_FENCE_ENABLED
    // reject destination if outside the fence
    const Location dest_loc(destination, Location::AltFrame::ABOVE_ORIGIN);
    if (!sub.fence.check_destination_within_fence(dest_loc)) {
        LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
        // failure is propagated to GCS with NAK
        return false;
    }
#endif

    // no need to check return status because terrain data is not used
    sub.wp_nav.set_wp_destination(destination, false);

#if HAL_LOGGING_ENABLED
    // log target
    sub.Log_Write_GuidedTarget(sub.guided_mode, destination, Vector3f());
#endif

    return true;
}

// sets guided mode's target from a Location object
// returns false if destination could not be set (probably caused by missing terrain data)
// or if the fence is enabled and guided waypoint is outside the fence
bool ModeGuided::guided_set_destination(const Location& dest_loc)
{
    // ensure we are in position control mode
    if (sub.guided_mode != Guided_WP) {
        guided_pos_control_start();
    }

#if AP_FENCE_ENABLED
    // reject destination outside the fence.
    // Note: there is a danger that a target specified as a terrain altitude might not be checked if the conversion to alt-above-home fails
    if (!sub.fence.check_destination_within_fence(dest_loc)) {
        LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
        // failure is propagated to GCS with NAK
        return false;
    }
#endif

    if (!sub.wp_nav.set_wp_destination_loc(dest_loc)) {
        // failure to set destination can only be because of missing terrain data
        LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::FAILED_TO_SET_DESTINATION);
        // failure is propagated to GCS with NAK
        return false;
    }

#if HAL_LOGGING_ENABLED
    // log target
    sub.Log_Write_GuidedTarget(sub.guided_mode, Vector3f(dest_loc.lat, dest_loc.lng, dest_loc.alt),Vector3f());
#endif

    return true;
}

// guided_set_destination - sets guided mode's target destination and target heading
// Returns true if the fence is enabled and guided waypoint is within the fence
// else return false if the waypoint is outside the fence
bool ModeGuided::guided_set_destination(const Vector3f& destination, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw)
{
    // ensure we are in position control mode
    if (sub.guided_mode != Guided_WP) {
        guided_pos_control_start();
    }

#if AP_FENCE_ENABLED
    // reject destination if outside the fence
    const Location dest_loc(destination, Location::AltFrame::ABOVE_ORIGIN);
    if (!sub.fence.check_destination_within_fence(dest_loc)) {
        LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
        // failure is propagated to GCS with NAK
        return false;
    }
#endif

    // set yaw state
    guided_set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

    update_time_ms = AP_HAL::millis();

    // no need to check return status because terrain data is not used
    sub.wp_nav.set_wp_destination(destination, false);

#if HAL_LOGGING_ENABLED
    // log target
    sub.Log_Write_GuidedTarget(sub.guided_mode, destination, Vector3f());
#endif

    return true;
}

// guided_set_velocity - sets guided mode's target velocity
void ModeGuided::guided_set_velocity(const Vector3f& velocity)
{
    // check we are in velocity control mode
    if (sub.guided_mode != Guided_Velocity) {
        guided_vel_control_start();
    }

    update_time_ms = AP_HAL::millis();

    // set position controller velocity target
    position_control->set_vel_desired_cms(velocity);
}

// guided_set_velocity - sets guided mode's target velocity
void ModeGuided::guided_set_velocity(const Vector3f& velocity, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw)
{
   // check we are in velocity control mode
    if (sub.guided_mode != Guided_Velocity) {
        guided_vel_control_start();
    }

    // set yaw state
    guided_set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

    update_time_ms = AP_HAL::millis();

    // set position controller velocity target
    position_control->set_vel_desired_cms(velocity);

}

// set guided mode posvel target
bool ModeGuided::guided_set_destination_posvel(const Vector3f& destination, const Vector3f& velocity)
{
    // check we are in velocity control mode
    if (sub.guided_mode != Guided_PosVel) {
        guided_posvel_control_start();
    }

#if AP_FENCE_ENABLED
    // reject destination if outside the fence
    const Location dest_loc(destination, Location::AltFrame::ABOVE_ORIGIN);
    if (!sub.fence.check_destination_within_fence(dest_loc)) {
        LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
        // failure is propagated to GCS with NAK
        return false;
    }
#endif

    update_time_ms = AP_HAL::millis();
    posvel_pos_target_cm = destination.topostype();
    posvel_vel_target_cms = velocity;

    position_control->input_pos_vel_accel_xy(posvel_pos_target_cm.xy(), posvel_vel_target_cms.xy(), Vector2f());
    float dz = posvel_pos_target_cm.z;
    position_control->input_pos_vel_accel_z(dz, posvel_vel_target_cms.z, 0);
    posvel_pos_target_cm.z = dz;

#if HAL_LOGGING_ENABLED
    // log target
    sub.Log_Write_GuidedTarget(sub.guided_mode, destination, velocity);
#endif

    return true;
}

// set guided mode posvel target
bool ModeGuided::guided_set_destination_posvel(const Vector3f& destination, const Vector3f& velocity, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw)
{
    // check we are in velocity control mode
    if (sub.guided_mode != Guided_PosVel) {
        guided_posvel_control_start();
    }

    #if AP_FENCE_ENABLED
    // reject destination if outside the fence
    const Location dest_loc(destination, Location::AltFrame::ABOVE_ORIGIN);
    if (!sub.fence.check_destination_within_fence(dest_loc)) {
        LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
        // failure is propagated to GCS with NAK
        return false;
    }
    #endif

    // set yaw state
    guided_set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

    update_time_ms = AP_HAL::millis();

    posvel_pos_target_cm = destination.topostype();
    posvel_vel_target_cms = velocity;

    position_control->input_pos_vel_accel_xy(posvel_pos_target_cm.xy(), posvel_vel_target_cms.xy(), Vector2f());
    float dz = posvel_pos_target_cm.z;
    position_control->input_pos_vel_accel_z(dz, posvel_vel_target_cms.z, 0);
    posvel_pos_target_cm.z = dz;

#if HAL_LOGGING_ENABLED
    // log target
    sub.Log_Write_GuidedTarget(sub.guided_mode, destination, velocity);
#endif

    return true;
}

// set guided mode angle target
void ModeGuided::guided_set_angle(const Quaternion &q, float climb_rate_cms)
{
    // check we are in velocity control mode
    if (sub.guided_mode != Guided_Angle) {
        guided_angle_control_start();
    }

    // convert quaternion to euler angles
    q.to_euler(guided_angle_state.roll_cd, guided_angle_state.pitch_cd, guided_angle_state.yaw_cd);
    guided_angle_state.roll_cd = ToDeg(guided_angle_state.roll_cd) * 100.0f;
    guided_angle_state.pitch_cd = ToDeg(guided_angle_state.pitch_cd) * 100.0f;
    guided_angle_state.yaw_cd = wrap_180_cd(ToDeg(guided_angle_state.yaw_cd) * 100.0f);

    guided_angle_state.climb_rate_cms = climb_rate_cms;
    guided_angle_state.update_time_ms = AP_HAL::millis();
}

// helper function to set yaw state and targets
void ModeGuided::guided_set_yaw_state(bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_angle)
{    
    float current_yaw = wrap_2PI(AP::ahrs().get_yaw());
    float euler_yaw_angle;
    float yaw_error;

    euler_yaw_angle = wrap_2PI((yaw_cd * 0.01f));
    yaw_error = wrap_PI(euler_yaw_angle - current_yaw);

    int direction = 0;
    if (yaw_error < 0){
        direction = -1;
    } else {
        direction = 1;
    }

    /*
    case 1: target yaw only
    case 2: target yaw and yaw rate
    case 3: target yaw rate only
    case 4: hold current yaw
    */
    if (use_yaw && !use_yaw_rate) {
        sub.yaw_rate_only = false;
        sub.mode_auto.set_auto_yaw_look_at_heading(yaw_cd * 0.01f, 0.0f, direction, relative_angle);
    } else if (use_yaw && use_yaw_rate) { 
        sub.yaw_rate_only = false;
        sub.mode_auto.set_auto_yaw_look_at_heading(yaw_cd * 0.01f, yaw_rate_cds * 0.01f, direction, relative_angle);
    } else if (!use_yaw && use_yaw_rate) {
        sub.yaw_rate_only = true;
        sub.mode_auto.set_yaw_rate(yaw_rate_cds * 0.01f);
    } else{
        sub.yaw_rate_only = false;
        set_auto_yaw_mode(AUTO_YAW_HOLD);
    }
}

// guided_run - runs the guided controller
// should be called at 100hz or more
void ModeGuided::run()
{
    // call the correct auto controller
    switch (sub.guided_mode) {

    case Guided_WP:
        // run position controller
        guided_pos_control_run();
        break;

    case Guided_Velocity:
        // run velocity controller
        guided_vel_control_run();
        break;

    case Guided_PosVel:
        // run position-velocity controller
        guided_posvel_control_run();
        break;

    case Guided_Angle:
        // run angle controller
        guided_angle_control_run();
        break;
    }
}

// guided_pos_control_run - runs the guided position controller
// called from guided_run
void ModeGuided::guided_pos_control_run()
{
    // if motors not enabled set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // Sub vehicles do not stabilize roll/pitch/yaw when disarmed
        attitude_control->set_throttle_out(0,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        sub.wp_nav.wp_and_spline_init();
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!sub.failsafe.pilot_input) {
        // get pilot's desired yaw rate
        target_yaw_rate = sub.get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        } else{
            if (sub.yaw_rate_only){
                set_auto_yaw_mode(AUTO_YAW_RATE);
            } else{
                set_auto_yaw_mode(AUTO_YAW_LOOK_AT_HEADING);
            }
        }
    }

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    sub.failsafe_terrain_set_status(sub.wp_nav.update_wpnav());

    float lateral_out, forward_out;
    sub.translate_wpnav_rp(lateral_out, forward_out);

    // Send to forward/lateral outputs
    motors.set_lateral(lateral_out);
    motors.set_forward(forward_out);

    // WP_Nav has set the vertical position control targets
    // run the vertical position controller and set output throttle
    position_control->update_z_controller();

    // call attitude controller
    if (sub.auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch & yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_yaw_rate);
    } else if (sub.auto_yaw_mode == AUTO_YAW_LOOK_AT_HEADING) {
        // roll, pitch from pilot, yaw & yaw_rate from auto_control
        target_yaw_rate = sub.yaw_look_at_heading_slew * 100.0;
        attitude_control->input_euler_angle_roll_pitch_slew_yaw(channel_roll->get_control_in(), channel_pitch->get_control_in(), get_auto_heading(), target_yaw_rate);
    } else if (sub.auto_yaw_mode == AUTO_YAW_RATE) {
        // roll, pitch from pilot, yaw_rate from auto_control
        target_yaw_rate = sub.yaw_look_at_heading_slew * 100.0;
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_yaw_rate);
    } else {
        // roll, pitch from pilot, yaw heading from auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(channel_roll->get_control_in(), channel_pitch->get_control_in(), get_auto_heading(), true);
    }
}

// guided_vel_control_run - runs the guided velocity controller
// called from guided_run
void ModeGuided::guided_vel_control_run()
{
    // ifmotors not enabled set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // Sub vehicles do not stabilize roll/pitch/yaw when disarmed
        attitude_control->set_throttle_out(0,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        // initialise velocity controller
        position_control->init_z_controller();
        position_control->init_xy_controller();
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!sub.failsafe.pilot_input) {
        // get pilot's desired yaw rate
        target_yaw_rate = sub.get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        } else{
            if (sub.yaw_rate_only){
                set_auto_yaw_mode(AUTO_YAW_RATE);
            } else{
                set_auto_yaw_mode(AUTO_YAW_LOOK_AT_HEADING);
            }
        }
    }

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // set velocity to zero if no updates received for 3 seconds
    uint32_t tnow = AP_HAL::millis();
    if (tnow - update_time_ms > GUIDED_POSVEL_TIMEOUT_MS && !position_control->get_vel_desired_cms().is_zero()) {
        position_control->set_vel_desired_cms(Vector3f(0,0,0));
    }

    position_control->stop_pos_xy_stabilisation();
    // call velocity controller which includes z axis controller
    position_control->update_xy_controller();

    position_control->set_pos_target_z_from_climb_rate_cm(position_control->get_vel_desired_cms().z);
    position_control->update_z_controller();

    float lateral_out, forward_out;
    sub.translate_pos_control_rp(lateral_out, forward_out);

    // Send to forward/lateral outputs
    motors.set_lateral(lateral_out);
    motors.set_forward(forward_out);

    // call attitude controller
    if (sub.auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch & yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_yaw_rate);
    } else if (sub.auto_yaw_mode == AUTO_YAW_LOOK_AT_HEADING) {
        // roll, pitch from pilot, yaw & yaw_rate from auto_control
        target_yaw_rate = sub.yaw_look_at_heading_slew * 100.0;
        attitude_control->input_euler_angle_roll_pitch_slew_yaw(channel_roll->get_control_in(), channel_pitch->get_control_in(), get_auto_heading(), target_yaw_rate);
    } else if (sub.auto_yaw_mode == AUTO_YAW_RATE) {
        // roll, pitch from pilot, yaw_rate from auto_control
        target_yaw_rate = sub.yaw_look_at_heading_slew * 100.0;
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_yaw_rate);
    } else {
        // roll, pitch from pilot, yaw heading from auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(channel_roll->get_control_in(), channel_pitch->get_control_in(), get_auto_heading(), true);
    }
}

// guided_posvel_control_run - runs the guided posvel controller
// called from guided_run
void ModeGuided::guided_posvel_control_run()
{
    // if motors not enabled set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // Sub vehicles do not stabilize roll/pitch/yaw when disarmed
        attitude_control->set_throttle_out(0,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        // initialise velocity controller
        position_control->init_z_controller();
        position_control->init_xy_controller();
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;

    if (!sub.failsafe.pilot_input) {
        // get pilot's desired yaw rate
        target_yaw_rate = sub.get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        } else{
            if (sub.yaw_rate_only){
                set_auto_yaw_mode(AUTO_YAW_RATE);
            } else{
                set_auto_yaw_mode(AUTO_YAW_LOOK_AT_HEADING);
            }
        }
    }

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // set velocity to zero if no updates received for 3 seconds
    uint32_t tnow = AP_HAL::millis();
    if (tnow - update_time_ms > GUIDED_POSVEL_TIMEOUT_MS && !posvel_vel_target_cms.is_zero()) {
        posvel_vel_target_cms.zero();
    }

    // advance position target using velocity target
    posvel_pos_target_cm += (posvel_vel_target_cms * position_control->get_dt()).topostype();

    // send position and velocity targets to position controller
    position_control->input_pos_vel_accel_xy(posvel_pos_target_cm.xy(), posvel_vel_target_cms.xy(), Vector2f());
    float pz = posvel_pos_target_cm.z;
    position_control->input_pos_vel_accel_z(pz, posvel_vel_target_cms.z, 0);
    posvel_pos_target_cm.z = pz;

    // run position controller
    position_control->update_xy_controller();
    position_control->update_z_controller();

    float lateral_out, forward_out;
    sub.translate_pos_control_rp(lateral_out, forward_out);

    // Send to forward/lateral outputs
    motors.set_lateral(lateral_out);
    motors.set_forward(forward_out);

    // call attitude controller
    if (sub.auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch & yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_yaw_rate);
    } else if (sub.auto_yaw_mode == AUTO_YAW_LOOK_AT_HEADING) {
        // roll, pitch from pilot, yaw & yaw_rate from auto_control
        target_yaw_rate = sub.yaw_look_at_heading_slew * 100.0;
        attitude_control->input_euler_angle_roll_pitch_slew_yaw(channel_roll->get_control_in(), channel_pitch->get_control_in(), get_auto_heading(), target_yaw_rate);
    } else if (sub.auto_yaw_mode == AUTO_YAW_RATE) {
        // roll, pitch from pilot, and yaw_rate from auto_control
        target_yaw_rate = sub.yaw_look_at_heading_slew * 100.0;
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_yaw_rate);
    } else {
        // roll, pitch from pilot, yaw heading from auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(channel_roll->get_control_in(), channel_pitch->get_control_in(), get_auto_heading(), true);
    }
}

// guided_angle_control_run - runs the guided angle controller
// called from guided_run
void ModeGuided::guided_angle_control_run()
{
    // if motors not enabled set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // Sub vehicles do not stabilize roll/pitch/yaw when disarmed
        attitude_control->set_throttle_out(0.0f,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        // initialise velocity controller
        position_control->init_z_controller();
        return;
    }

    // constrain desired lean angles
    float roll_in = guided_angle_state.roll_cd;
    float pitch_in = guided_angle_state.pitch_cd;
    float total_in = norm(roll_in, pitch_in);
    float angle_max = MIN(attitude_control->get_althold_lean_angle_max_cd(), sub.aparm.angle_max);
    if (total_in > angle_max) {
        float ratio = angle_max / total_in;
        roll_in *= ratio;
        pitch_in *= ratio;
    }

    // wrap yaw request
    float yaw_in = wrap_180_cd(guided_angle_state.yaw_cd);

    // constrain climb rate
    float climb_rate_cms = constrain_float(guided_angle_state.climb_rate_cms, -sub.wp_nav.get_default_speed_down(), sub.wp_nav.get_default_speed_up());

    // check for timeout - set lean angles and climb rate to zero if no updates received for 3 seconds
    uint32_t tnow = AP_HAL::millis();
    if (tnow - guided_angle_state.update_time_ms > GUIDED_ATTITUDE_TIMEOUT_MS) {
        roll_in = 0.0f;
        pitch_in = 0.0f;
        climb_rate_cms = 0.0f;
    }

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_yaw(roll_in, pitch_in, yaw_in, true);

    // call position controller
    position_control->set_pos_target_z_from_climb_rate_cm(climb_rate_cms);
    position_control->update_z_controller();
}

// Guided Limit code

// guided_limit_clear - clear/turn off guided limits
void ModeGuided::guided_limit_clear()
{
    guided_limit.timeout_ms = 0;
    guided_limit.alt_min_cm = 0.0f;
    guided_limit.alt_max_cm = 0.0f;
    guided_limit.horiz_max_cm = 0.0f;
}


// set_auto_yaw_mode - sets the yaw mode for auto
void ModeGuided::set_auto_yaw_mode(autopilot_yaw_mode yaw_mode)
{
    // return immediately if no change
    if (sub.auto_yaw_mode == yaw_mode) {
        return;
    }
    sub.auto_yaw_mode = yaw_mode;

    // perform initialisation
    switch (sub.auto_yaw_mode) {
    
    case AUTO_YAW_HOLD:
        // pilot controls the heading
        break;

    case AUTO_YAW_LOOK_AT_NEXT_WP:
        // wpnav will initialise heading when wpnav's set_destination method is called
        break;

    case AUTO_YAW_ROI:
        // point towards a location held in yaw_look_at_WP
        sub.yaw_look_at_WP_bearing = ahrs.yaw_sensor;
        break;

    case AUTO_YAW_LOOK_AT_HEADING:
        // keep heading pointing in the direction held in yaw_look_at_heading
        // caller should set the yaw_look_at_heading
        break;

    case AUTO_YAW_LOOK_AHEAD:
        // Commanded Yaw to automatically look ahead.
        sub.yaw_look_ahead_bearing = ahrs.yaw_sensor;
        break;

    case AUTO_YAW_RESETTOARMEDYAW:
        // initial_armed_bearing will be set during arming so no init required
        break;
    
    case AUTO_YAW_RATE:
        // set target yaw rate to yaw_look_at_heading_slew
        break;
    }
}

// get_auto_heading - returns target heading depending upon auto_yaw_mode
// 100hz update rate
float ModeGuided::get_auto_heading()
{
    switch (sub.auto_yaw_mode) {

    case AUTO_YAW_ROI:
        // point towards a location held in roi_WP
        return sub.get_roi_yaw();
        break;

    case AUTO_YAW_LOOK_AT_HEADING:
        // keep heading pointing in the direction held in yaw_look_at_heading with no pilot input allowed
        return sub.yaw_look_at_heading;
        break;

    case AUTO_YAW_LOOK_AHEAD:
        // Commanded Yaw to automatically look ahead.
        return sub.get_look_ahead_yaw();
        break;

    case AUTO_YAW_RESETTOARMEDYAW:
        // changes yaw to be same as when quad was armed
        return sub.initial_armed_bearing;
        break;

    case AUTO_YAW_CORRECT_XTRACK: {
        // TODO return current yaw if not in appropriate mode
        // Bearing of current track (centidegrees)
        float track_bearing = get_bearing_cd(sub.wp_nav.get_wp_origin().xy(), sub.wp_nav.get_wp_destination().xy());

        // Bearing from current position towards intermediate position target (centidegrees)
        const Vector2f target_vel_xy{position_control->get_vel_target_cms().x, position_control->get_vel_target_cms().y};
        float angle_error = 0.0f;
        if (target_vel_xy.length() >= position_control->get_max_speed_xy_cms() * 0.1f) {
            const float desired_angle_cd = degrees(target_vel_xy.angle()) * 100.0f;
            angle_error = wrap_180_cd(desired_angle_cd - track_bearing);
        }
        float angle_limited = constrain_float(angle_error, -g.xtrack_angle_limit * 100.0f, g.xtrack_angle_limit * 100.0f);
        return wrap_360_cd(track_bearing + angle_limited);
    }
    break;

    case AUTO_YAW_LOOK_AT_NEXT_WP:
    default:
        // point towards next waypoint.
        // we don't use wp_bearing because we don't want the vehicle to turn too much during flight
        return sub.wp_nav.get_yaw();
        break;
    }
}
// guided_limit_set - set guided timeout and movement limits
void ModeGuided::guided_limit_set(uint32_t timeout_ms, float alt_min_cm, float alt_max_cm, float horiz_max_cm)
{
    guided_limit.timeout_ms = timeout_ms;
    guided_limit.alt_min_cm = alt_min_cm;
    guided_limit.alt_max_cm = alt_max_cm;
    guided_limit.horiz_max_cm = horiz_max_cm;
}

// guided_limit_init_time_and_pos - initialise guided start time and position as reference for limit checking
//  only called from AUTO mode's auto_nav_guided_start function
void ModeGuided::guided_limit_init_time_and_pos()
{
    // initialise start time
    guided_limit.start_time = AP_HAL::millis();

    // initialise start position from current position
    guided_limit.start_pos = inertial_nav.get_position_neu_cm();
}

// guided_limit_check - returns true if guided mode has breached a limit
//  used when guided is invoked from the NAV_GUIDED_ENABLE mission command
bool ModeGuided::guided_limit_check()
{
    // check if we have passed the timeout
    if ((guided_limit.timeout_ms > 0) && (AP_HAL::millis() - guided_limit.start_time >= guided_limit.timeout_ms)) {
        return true;
    }

    // get current location
    const Vector3f& curr_pos = inertial_nav.get_position_neu_cm();

    // check if we have gone below min alt
    if (!is_zero(guided_limit.alt_min_cm) && (curr_pos.z < guided_limit.alt_min_cm)) {
        return true;
    }

    // check if we have gone above max alt
    if (!is_zero(guided_limit.alt_max_cm) && (curr_pos.z > guided_limit.alt_max_cm)) {
        return true;
    }

    // check if we have gone beyond horizontal limit
    if (guided_limit.horiz_max_cm > 0.0f) {
        const float horiz_move = get_horizontal_distance_cm(guided_limit.start_pos.xy(), curr_pos.xy());
        if (horiz_move > guided_limit.horiz_max_cm) {
            return true;
        }
    }

    // if we got this far we must be within limits
    return false;
}
