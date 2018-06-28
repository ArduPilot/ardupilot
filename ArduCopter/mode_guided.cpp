#include "Copter.h"

/*
 * Init and run calls for guided flight mode
 */

#ifndef GUIDED_LOOK_AT_TARGET_MIN_DISTANCE_CM
 # define GUIDED_LOOK_AT_TARGET_MIN_DISTANCE_CM     500     // point nose at target if it is more than 5m away
#endif

#define GUIDED_POSVEL_TIMEOUT_MS    3000    // guided mode's position-velocity controller times out after 3seconds with no new updates
#define GUIDED_ATTITUDE_TIMEOUT_MS  1000    // guided mode's attitude controller times out after 1 second with no new updates

static Vector3f guided_pos_target_cm;       // position target (used by posvel controller only)
static Vector3f guided_vel_target_cms;      // velocity target (used by velocity controller and posvel controller)
static uint32_t posvel_update_time_ms;      // system time of last target update to posvel controller (i.e. position and velocity update)
static uint32_t vel_update_time_ms;         // system time of last target update to velocity controller

struct {
    uint32_t update_time_ms;
    float roll_cd;
    float pitch_cd;
    float yaw_cd;
    float yaw_rate_cds;
    float climb_rate_cms;
    bool use_yaw_rate;
} static guided_angle_state;

struct Guided_Limit {
    uint32_t timeout_ms;  // timeout (in seconds) from the time that guided is invoked
    float alt_min_cm;   // lower altitude limit in cm above home (0 = no limit)
    float alt_max_cm;   // upper altitude limit in cm above home (0 = no limit)
    float horiz_max_cm; // horizontal position limit in cm from where guided mode was initiated (0 = no limit)
    uint32_t start_time;// system time in milliseconds that control was handed to the external computer
    Vector3f start_pos; // start position as a distance from home in cm.  used for checking horiz_max limit
} guided_limit;

// guided_init - initialise guided controller
bool Copter::ModeGuided::init(bool ignore_checks)
{
    if (copter.position_ok() || ignore_checks) {
        // start in position control mode
        pos_control_start();
        return true;
    }else{
        return false;
    }
}


// do_user_takeoff_start - initialises waypoint controller to implement take-off
bool Copter::ModeGuided::do_user_takeoff_start(float final_alt_above_home)
{
    guided_mode = Guided_TakeOff;

    // initialise wpnav destination
    Location_Class target_loc = copter.current_loc;
    target_loc.set_alt_cm(final_alt_above_home, Location_Class::ALT_FRAME_ABOVE_HOME);

    if (!wp_nav->set_wp_destination(target_loc)) {
        // failure to set destination can only be because of missing terrain data
        copter.Log_Write_Error(ERROR_SUBSYSTEM_NAVIGATION, ERROR_CODE_FAILED_TO_SET_DESTINATION);
        // failure is propagated to GCS with NAK
        return false;
    }

    // initialise yaw
    auto_yaw.set_mode(AUTO_YAW_HOLD);

    // clear i term when we're taking off
    set_throttle_takeoff();

    // get initial alt for WP_NAVALT_MIN
    copter.auto_takeoff_set_start_alt();

    return true;
}

// initialise guided mode's position controller
void Copter::ModeGuided::pos_control_start()
{
    // set to position control mode
    guided_mode = Guided_WP;

    // initialise waypoint and spline controller
    wp_nav->wp_and_spline_init();

    // initialise wpnav to stopping point
    Vector3f stopping_point;
    wp_nav->get_wp_stopping_point(stopping_point);

    // no need to check return status because terrain data is not used
    wp_nav->set_wp_destination(stopping_point, false);

    // initialise yaw
    auto_yaw.set_mode_to_default(false);
}

// initialise guided mode's velocity controller
void Copter::ModeGuided::vel_control_start()
{
    // set guided_mode to velocity controller
    guided_mode = Guided_Velocity;

    // initialise horizontal speed, acceleration
    pos_control->set_speed_xy(wp_nav->get_speed_xy());
    pos_control->set_accel_xy(wp_nav->get_wp_acceleration());

    // initialize vertical speeds and acceleration
    pos_control->set_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_accel_z(g.pilot_accel_z);

    // initialise velocity controller
    pos_control->init_vel_controller_xyz();
}

// initialise guided mode's posvel controller
void Copter::ModeGuided::posvel_control_start()
{
    // set guided_mode to velocity controller
    guided_mode = Guided_PosVel;

    pos_control->init_xy_controller();

    // set speed and acceleration from wpnav's speed and acceleration
    pos_control->set_speed_xy(wp_nav->get_speed_xy());
    pos_control->set_accel_xy(wp_nav->get_wp_acceleration());

    const Vector3f& curr_pos = inertial_nav.get_position();
    const Vector3f& curr_vel = inertial_nav.get_velocity();

    // set target position and velocity to current position and velocity
    pos_control->set_xy_target(curr_pos.x, curr_pos.y);
    pos_control->set_desired_velocity_xy(curr_vel.x, curr_vel.y);

    // set vertical speed and acceleration
    pos_control->set_speed_z(wp_nav->get_speed_down(), wp_nav->get_speed_up());
    pos_control->set_accel_z(wp_nav->get_accel_z());

    // pilot always controls yaw
    auto_yaw.set_mode(AUTO_YAW_HOLD);
}

// initialise guided mode's angle controller
void Copter::ModeGuided::angle_control_start()
{
    // set guided_mode to velocity controller
    guided_mode = Guided_Angle;

    // set vertical speed and acceleration
    pos_control->set_speed_z(wp_nav->get_speed_down(), wp_nav->get_speed_up());
    pos_control->set_accel_z(wp_nav->get_accel_z());

    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    // initialise targets
    guided_angle_state.update_time_ms = millis();
    guided_angle_state.roll_cd = ahrs.roll_sensor;
    guided_angle_state.pitch_cd = ahrs.pitch_sensor;
    guided_angle_state.yaw_cd = ahrs.yaw_sensor;
    guided_angle_state.climb_rate_cms = 0.0f;
    guided_angle_state.yaw_rate_cds = 0.0f;
    guided_angle_state.use_yaw_rate = false;

    // pilot always controls yaw
    auto_yaw.set_mode(AUTO_YAW_HOLD);
}

// guided_set_destination - sets guided mode's target destination
// Returns true if the fence is enabled and guided waypoint is within the fence
// else return false if the waypoint is outside the fence
bool Copter::ModeGuided::set_destination(const Vector3f& destination, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw)
{
    // ensure we are in position control mode
    if (guided_mode != Guided_WP) {
        pos_control_start();
    }

#if AC_FENCE == ENABLED
    // reject destination if outside the fence
    Location_Class dest_loc(destination);
    if (!copter.fence.check_destination_within_fence(dest_loc)) {
        copter.Log_Write_Error(ERROR_SUBSYSTEM_NAVIGATION, ERROR_CODE_DEST_OUTSIDE_FENCE);
        // failure is propagated to GCS with NAK
        return false;
    }
#endif

    // set yaw state
    set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

    // no need to check return status because terrain data is not used
    wp_nav->set_wp_destination(destination, false);

    // log target
    copter.Log_Write_GuidedTarget(guided_mode, destination, Vector3f());
    return true;
}

bool Copter::ModeGuided::get_wp(Location_Class& destination)
{
    if (guided_mode != Guided_WP) {
        return false;
    }
    return wp_nav->get_wp_destination(destination);
}

// sets guided mode's target from a Location object
// returns false if destination could not be set (probably caused by missing terrain data)
// or if the fence is enabled and guided waypoint is outside the fence
bool Copter::ModeGuided::set_destination(const Location_Class& dest_loc, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw)
{
    // ensure we are in position control mode
    if (guided_mode != Guided_WP) {
        pos_control_start();
    }

#if AC_FENCE == ENABLED
    // reject destination outside the fence.
    // Note: there is a danger that a target specified as a terrain altitude might not be checked if the conversion to alt-above-home fails
    if (!copter.fence.check_destination_within_fence(dest_loc)) {
        copter.Log_Write_Error(ERROR_SUBSYSTEM_NAVIGATION, ERROR_CODE_DEST_OUTSIDE_FENCE);
        // failure is propagated to GCS with NAK
        return false;
    }
#endif

    if (!wp_nav->set_wp_destination(dest_loc)) {
        // failure to set destination can only be because of missing terrain data
        copter.Log_Write_Error(ERROR_SUBSYSTEM_NAVIGATION, ERROR_CODE_FAILED_TO_SET_DESTINATION);
        // failure is propagated to GCS with NAK
        return false;
    }

    // set yaw state
    set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

    // log target
    copter.Log_Write_GuidedTarget(guided_mode, Vector3f(dest_loc.lat, dest_loc.lng, dest_loc.alt),Vector3f());
    return true;
}

// guided_set_velocity - sets guided mode's target velocity
void Copter::ModeGuided::set_velocity(const Vector3f& velocity, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw, bool log_request)
{
    // check we are in velocity control mode
    if (guided_mode != Guided_Velocity) {
        vel_control_start();
    }

    // set yaw state
    set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

    // record velocity target
    guided_vel_target_cms = velocity;
    vel_update_time_ms = millis();

    // log target
    if (log_request) {
        copter.Log_Write_GuidedTarget(guided_mode, Vector3f(), velocity);
    }
}

// set guided mode posvel target
bool Copter::ModeGuided::set_destination_posvel(const Vector3f& destination, const Vector3f& velocity, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw)
{
    // check we are in velocity control mode
    if (guided_mode != Guided_PosVel) {
        posvel_control_start();
    }

#if AC_FENCE == ENABLED
    // reject destination if outside the fence
    Location_Class dest_loc(destination);
    if (!copter.fence.check_destination_within_fence(dest_loc)) {
        copter.Log_Write_Error(ERROR_SUBSYSTEM_NAVIGATION, ERROR_CODE_DEST_OUTSIDE_FENCE);
        // failure is propagated to GCS with NAK
        return false;
    }
#endif

    // set yaw state
    set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

    posvel_update_time_ms = millis();
    guided_pos_target_cm = destination;
    guided_vel_target_cms = velocity;

    copter.pos_control->set_pos_target(guided_pos_target_cm);

    // log target
    copter.Log_Write_GuidedTarget(guided_mode, destination, velocity);
    return true;
}

// set guided mode angle target
void Copter::ModeGuided::set_angle(const Quaternion &q, float climb_rate_cms, bool use_yaw_rate, float yaw_rate_rads)
{
    // check we are in velocity control mode
    if (guided_mode != Guided_Angle) {
        angle_control_start();
    }

    // convert quaternion to euler angles
    q.to_euler(guided_angle_state.roll_cd, guided_angle_state.pitch_cd, guided_angle_state.yaw_cd);
    guided_angle_state.roll_cd = ToDeg(guided_angle_state.roll_cd) * 100.0f;
    guided_angle_state.pitch_cd = ToDeg(guided_angle_state.pitch_cd) * 100.0f;
    guided_angle_state.yaw_cd = wrap_180_cd(ToDeg(guided_angle_state.yaw_cd) * 100.0f);
    guided_angle_state.yaw_rate_cds = ToDeg(yaw_rate_rads) * 100.0f;
    guided_angle_state.use_yaw_rate = use_yaw_rate;

    guided_angle_state.climb_rate_cms = climb_rate_cms;
    guided_angle_state.update_time_ms = millis();

    // interpret positive climb rate as triggering take-off
    if (motors->armed() && !ap.auto_armed && (guided_angle_state.climb_rate_cms > 0.0f)) {
        copter.set_auto_armed(true);
    }

    // log target
    copter.Log_Write_GuidedTarget(guided_mode,
                           Vector3f(guided_angle_state.roll_cd, guided_angle_state.pitch_cd, guided_angle_state.yaw_cd),
                           Vector3f(0.0f, 0.0f, guided_angle_state.climb_rate_cms));
}

// guided_run - runs the guided controller
// should be called at 100hz or more
void Copter::ModeGuided::run()
{
    // call the correct auto controller
    switch (guided_mode) {

    case Guided_TakeOff:
        // run takeoff controller
        takeoff_run();
        break;

    case Guided_WP:
        // run position controller
        pos_control_run();
        break;

    case Guided_Velocity:
        // run velocity controller
        vel_control_run();
        break;

    case Guided_PosVel:
        // run position-velocity controller
        posvel_control_run();
        break;

    case Guided_Angle:
        // run angle controller
        angle_control_run();
        break;
    }
 }

// guided_takeoff_run - takeoff in guided mode
//      called by guided_run at 100hz or more
void Copter::ModeGuided::takeoff_run()
{
    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock()) {
        // initialise wpnav targets
        wp_nav->shift_wp_origin_to_current_pos();
        zero_throttle_and_relax_ac();
        // clear i term when we're taking off
        set_throttle_takeoff();
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    }

#if FRAME_CONFIG == HELI_FRAME
    // helicopters stay in landed state until rotor speed runup has finished
    if (motors->rotor_runup_complete()) {
        set_land_complete(false);
    } else {
        // initialise wpnav targets
        wp_nav->shift_wp_origin_to_current_pos();
    }
#else
    set_land_complete(false);
#endif

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    copter.pos_control->update_z_controller();

    // call attitude controller
    copter.auto_takeoff_attitude_run(target_yaw_rate);
}

// guided_pos_control_run - runs the guided position controller
// called from guided_run
void Copter::ModeGuided::pos_control_run()
{
    // if not auto armed or motors not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock() || ap.land_complete) {
        zero_throttle_and_relax_ac();
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate);
    } else if (auto_yaw.mode() == AUTO_YAW_RATE) {
        // roll & pitch from waypoint controller, yaw rate from mavlink command or mission item
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.rate_cds());
    } else {
        // roll, pitch from waypoint controller, yaw heading from GCS or auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.yaw(), true);
    }
}

// guided_vel_control_run - runs the guided velocity controller
// called from guided_run
void Copter::ModeGuided::vel_control_run()
{
    // if not auto armed or motors not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock() || ap.land_complete) {
        // initialise velocity controller
        pos_control->init_vel_controller_xyz();
        zero_throttle_and_relax_ac();
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // set velocity to zero and stop rotating if no updates received for 3 seconds
    uint32_t tnow = millis();
    if (tnow - vel_update_time_ms > GUIDED_POSVEL_TIMEOUT_MS) {
        if (!pos_control->get_desired_velocity().is_zero()) {
            set_desired_velocity_with_accel_and_fence_limits(Vector3f(0.0f, 0.0f, 0.0f));
        }
        if (auto_yaw.mode() == AUTO_YAW_RATE) {
            auto_yaw.set_rate(0.0f);
        }
    } else {
        set_desired_velocity_with_accel_and_fence_limits(guided_vel_target_cms);
    }

    // call velocity controller which includes z axis controller
    pos_control->update_vel_controller_xyz(ekfNavVelGainScaler);

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(pos_control->get_roll(), pos_control->get_pitch(), target_yaw_rate);
    } else if (auto_yaw.mode() == AUTO_YAW_RATE) {
        // roll & pitch from velocity controller, yaw rate from mavlink command or mission item
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(pos_control->get_roll(), pos_control->get_pitch(), auto_yaw.rate_cds());
    } else {
        // roll, pitch from waypoint controller, yaw heading from GCS or auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(pos_control->get_roll(), pos_control->get_pitch(), auto_yaw.yaw(), true);
    }
}

// guided_posvel_control_run - runs the guided spline controller
// called from guided_run
void Copter::ModeGuided::posvel_control_run()
{
    // if not auto armed or motors not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock() || ap.land_complete) {
        // set target position and velocity to current position and velocity
        pos_control->set_pos_target(inertial_nav.get_position());
        pos_control->set_desired_velocity(Vector3f(0,0,0));
        zero_throttle_and_relax_ac();
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;

    if (!copter.failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // set velocity to zero and stop rotating if no updates received for 3 seconds
    uint32_t tnow = millis();
    if (tnow - posvel_update_time_ms > GUIDED_POSVEL_TIMEOUT_MS) {
        guided_vel_target_cms.zero();
        if (auto_yaw.mode() == AUTO_YAW_RATE) {
            auto_yaw.set_rate(0.0f);
        }
    }

    // calculate dt
    float dt = pos_control->time_since_last_xy_update();

    // sanity check dt
    if (dt >= 0.2f) {
        dt = 0.0f;
    }

    // advance position target using velocity target
    guided_pos_target_cm += guided_vel_target_cms * dt;

    // send position and velocity targets to position controller
    pos_control->set_pos_target(guided_pos_target_cm);
    pos_control->set_desired_velocity_xy(guided_vel_target_cms.x, guided_vel_target_cms.y);

    // run position controllers
    pos_control->update_xy_controller(ekfNavVelGainScaler);
    pos_control->update_z_controller();

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(pos_control->get_roll(), pos_control->get_pitch(), target_yaw_rate);
    } else if (auto_yaw.mode() == AUTO_YAW_RATE) {
        // roll & pitch from position-velocity controller, yaw rate from mavlink command or mission item
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(pos_control->get_roll(), pos_control->get_pitch(), auto_yaw.rate_cds());
    } else {
        // roll, pitch from waypoint controller, yaw heading from GCS or auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(pos_control->get_roll(), pos_control->get_pitch(), auto_yaw.yaw(), true);
    }
}

// guided_angle_control_run - runs the guided angle controller
// called from guided_run
void Copter::ModeGuided::angle_control_run()
{
    // if not auto armed or motors not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock() || (ap.land_complete && guided_angle_state.climb_rate_cms <= 0.0f)) {
#if FRAME_CONFIG == HELI_FRAME
        attitude_control->set_yaw_target_to_current_heading();
#endif
        zero_throttle_and_relax_ac();
        pos_control->relax_alt_hold_controllers(0.0f);
        return;
    }

    // constrain desired lean angles
    float roll_in = guided_angle_state.roll_cd;
    float pitch_in = guided_angle_state.pitch_cd;
    float total_in = norm(roll_in, pitch_in);
    float angle_max = MIN(attitude_control->get_althold_lean_angle_max(), copter.aparm.angle_max);
    if (total_in > angle_max) {
        float ratio = angle_max / total_in;
        roll_in *= ratio;
        pitch_in *= ratio;
    }

    // wrap yaw request
    float yaw_in = wrap_180_cd(guided_angle_state.yaw_cd);
    float yaw_rate_in = wrap_180_cd(guided_angle_state.yaw_rate_cds);

    // constrain climb rate
    float climb_rate_cms = constrain_float(guided_angle_state.climb_rate_cms, -fabsf(wp_nav->get_speed_down()), wp_nav->get_speed_up());

    // get avoidance adjusted climb rate
    climb_rate_cms = get_avoidance_adjusted_climbrate(climb_rate_cms);

    // check for timeout - set lean angles and climb rate to zero if no updates received for 3 seconds
    uint32_t tnow = millis();
    if (tnow - guided_angle_state.update_time_ms > GUIDED_ATTITUDE_TIMEOUT_MS) {
        roll_in = 0.0f;
        pitch_in = 0.0f;
        climb_rate_cms = 0.0f;
        yaw_rate_in = 0.0f;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // call attitude controller
    if (guided_angle_state.use_yaw_rate) {
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(roll_in, pitch_in, yaw_rate_in);
    } else {
        attitude_control->input_euler_angle_roll_pitch_yaw(roll_in, pitch_in, yaw_in, true);
    }

    // call position controller
    pos_control->set_alt_target_from_climb_rate_ff(climb_rate_cms, G_Dt, false);
    pos_control->update_z_controller();
}

// helper function to update position controller's desired velocity while respecting acceleration limits
void Copter::ModeGuided::set_desired_velocity_with_accel_and_fence_limits(const Vector3f& vel_des)
{
    // get current desired velocity
    Vector3f curr_vel_des = pos_control->get_desired_velocity();

    // get change in desired velocity
    Vector3f vel_delta = vel_des - curr_vel_des;

    // limit xy change
    float vel_delta_xy = safe_sqrt(sq(vel_delta.x)+sq(vel_delta.y));
    float vel_delta_xy_max = G_Dt * pos_control->get_accel_xy();
    float ratio_xy = 1.0f;
    if (!is_zero(vel_delta_xy) && (vel_delta_xy > vel_delta_xy_max)) {
        ratio_xy = vel_delta_xy_max / vel_delta_xy;
    }
    curr_vel_des.x += (vel_delta.x * ratio_xy);
    curr_vel_des.y += (vel_delta.y * ratio_xy);

    // limit z change
    float vel_delta_z_max = G_Dt * pos_control->get_accel_z();
    curr_vel_des.z += constrain_float(vel_delta.z, -vel_delta_z_max, vel_delta_z_max);

#if AC_AVOID_ENABLED
    // limit the velocity to prevent fence violations
    copter.avoid.adjust_velocity(pos_control->get_pos_xy_p().kP(), pos_control->get_accel_xy(), curr_vel_des, G_Dt);
    // get avoidance adjusted climb rate
    curr_vel_des.z = get_avoidance_adjusted_climbrate(curr_vel_des.z);    
#endif

    // update position controller with new target
    pos_control->set_desired_velocity(curr_vel_des);
}

// helper function to set yaw state and targets
void Copter::ModeGuided::set_yaw_state(bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_angle)
{
    if (use_yaw) {
        auto_yaw.set_fixed_yaw(yaw_cd / 100.0f, 0.0f, 0, relative_angle);
    } else if (use_yaw_rate) {
        auto_yaw.set_rate(yaw_rate_cds);
    }
}

// Guided Limit code

// guided_limit_clear - clear/turn off guided limits
void Copter::ModeGuided::limit_clear()
{
    guided_limit.timeout_ms = 0;
    guided_limit.alt_min_cm = 0.0f;
    guided_limit.alt_max_cm = 0.0f;
    guided_limit.horiz_max_cm = 0.0f;
}

// guided_limit_set - set guided timeout and movement limits
void Copter::ModeGuided::limit_set(uint32_t timeout_ms, float alt_min_cm, float alt_max_cm, float horiz_max_cm)
{
    guided_limit.timeout_ms = timeout_ms;
    guided_limit.alt_min_cm = alt_min_cm;
    guided_limit.alt_max_cm = alt_max_cm;
    guided_limit.horiz_max_cm = horiz_max_cm;
}

// guided_limit_init_time_and_pos - initialise guided start time and position as reference for limit checking
//  only called from AUTO mode's auto_nav_guided_start function
void Copter::ModeGuided::limit_init_time_and_pos()
{
    // initialise start time
    guided_limit.start_time = AP_HAL::millis();

    // initialise start position from current position
    guided_limit.start_pos = inertial_nav.get_position();
}

// guided_limit_check - returns true if guided mode has breached a limit
//  used when guided is invoked from the NAV_GUIDED_ENABLE mission command
bool Copter::ModeGuided::limit_check()
{
    // check if we have passed the timeout
    if ((guided_limit.timeout_ms > 0) && (millis() - guided_limit.start_time >= guided_limit.timeout_ms)) {
        return true;
    }

    // get current location
    const Vector3f& curr_pos = inertial_nav.get_position();

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
        float horiz_move = get_horizontal_distance_cm(guided_limit.start_pos, curr_pos);
        if (horiz_move > guided_limit.horiz_max_cm) {
            return true;
        }
    }

    // if we got this far we must be within limits
    return false;
}


uint32_t Copter::ModeGuided::wp_distance() const
{
    switch(mode()) {
    case Guided_WP:
        return wp_nav->get_wp_distance_to_destination();
        break;
    case Guided_PosVel:
        return pos_control->get_distance_to_target();
        break;
    default:
        return 0;
    }
}

int32_t Copter::ModeGuided::wp_bearing() const
{
    switch(mode()) {
    case Guided_WP:
        return wp_nav->get_wp_bearing_to_destination();
        break;
    case Guided_PosVel:
        return pos_control->get_bearing_to_target();
        break;
    default:
        return 0;
    }
}

float Copter::ModeGuided::crosstrack_error() const
{
    if (mode() == Guided_WP) {
        return wp_nav->crosstrack_error();
    } else {
        return 0;
    }
}
