#include "Copter.h"

#if MODE_4DAUTO_ENABLED == ENABLED

/*
 * Init and run calls for 4D flight mode
 */

#ifndef GUIDED_LOOK_AT_TARGET_MIN_DISTANCE_CM
 # define GUIDED_LOOK_AT_TARGET_MIN_DISTANCE_CM     500     // point nose at target if it is more than 5m away
#endif

#define GUIDED_POSVEL_TIMEOUT_MS    3000    // guided mode's position-velocity controller times out after 3seconds with no new updates
#define GUIDED_ATTITUDE_TIMEOUT_MS  1000    // guided mode's attitude controller times out after 1 second with no new updates

static Vector3f four_d_pos_target_cm;       // position target (used by posvel controller only)
static Vector3f four_d_vel_target_cms;      // velocity target (used by velocity controller and posvel controller)
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
} static four_d_auto_angle_state;

struct FourDAuto_Limit {
    uint32_t timeout_ms;  // timeout (in seconds) from the time that guided is invoked
    float alt_min_cm;   // lower altitude limit in cm above home (0 = no limit)
    float alt_max_cm;   // upper altitude limit in cm above home (0 = no limit)
    float horiz_max_cm; // horizontal position limit in cm from where guided mode was initiated (0 = no limit)
    uint32_t start_time;// system time in milliseconds that control was handed to the external computer
    Vector3f start_pos; // start position as a distance from home in cm.  used for checking horiz_max limit
} four_d_auto_limit;

// 4Dauto_init - initialise 4D controller
bool Mode4DAuto::init(bool ignore_checks)
{
	pos_control_start();
	return true;
}

// do_user_takeoff_start - initialises waypoint controller to implement take-off
bool Mode4DAuto::do_user_takeoff_start(float takeoff_alt_cm)
{
    four_d_mode = FourDAuto_TakeOff;

    // initialise wpnav destination
    Location target_loc = copter.current_loc;
    Location::AltFrame frame = Location::AltFrame::ABOVE_HOME;
    if (wp_nav->rangefinder_used_and_healthy() &&
            wp_nav->get_terrain_source() == AC_WPNav::TerrainSource::TERRAIN_FROM_RANGEFINDER &&
            takeoff_alt_cm < copter.rangefinder.max_distance_cm_orient(ROTATION_PITCH_270)) {
        // can't takeoff downwards
        if (takeoff_alt_cm <= copter.rangefinder_state.alt_cm) {
            return false;
        }
        frame = Location::AltFrame::ABOVE_TERRAIN;
    }
    target_loc.set_alt_cm(takeoff_alt_cm, frame);

    if (!wp_nav->set_wp_destination(target_loc)) {
        // failure to set destination can only be because of missing terrain data
        AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::FAILED_TO_SET_DESTINATION);
        // failure is propagated to GCS with NAK
        return false;
    }

    // initialise yaw
    auto_yaw.set_mode(AUTO_YAW_HOLD);

    // clear i term when we're taking off
    set_throttle_takeoff();

    // get initial alt for WP_NAVALT_MIN
    auto_takeoff_set_start_alt();

    return true;
}

// initialise 4D mode's position controller (from guided mode)
void Mode4DAuto::pos_control_start()
{
    // set to position control mode
	four_d_mode = FourDAuto_WP;

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

// initialise 4D mode's velocity controller
void Mode4DAuto::vel_control_start()
{
    // set four_d_mode to velocity controller
	four_d_mode = FourDAuto_Velocity;

    // initialise horizontal speed, acceleration
    pos_control->set_max_speed_xy(wp_nav->get_default_speed_xy());
    pos_control->set_max_accel_xy(wp_nav->get_wp_acceleration());

    // initialize vertical speeds and acceleration
    pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_max_accel_z(g.pilot_accel_z);

    // initialise velocity controller
    pos_control->init_vel_controller_xyz();
}

// initialise 4D mode's posvel controller
void Mode4DAuto::posvel_control_start()
{
    // set four_d_mode to velocity controller
	four_d_mode = FourDAuto_PosVel;

    pos_control->init_xy_controller();

    // set speed and acceleration from wpnav's speed and acceleration
    pos_control->set_max_speed_xy(wp_nav->get_default_speed_xy());
    pos_control->set_max_accel_xy(wp_nav->get_wp_acceleration());

    const Vector3f& curr_pos = inertial_nav.get_position();
    const Vector3f& curr_vel = inertial_nav.get_velocity();

    // set target position and velocity to current position and velocity
    pos_control->set_xy_target(curr_pos.x, curr_pos.y);
    pos_control->set_desired_velocity_xy(curr_vel.x, curr_vel.y);

    // set vertical speed and acceleration
    pos_control->set_max_speed_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up());
    pos_control->set_max_accel_z(wp_nav->get_accel_z());

    // pilot always controls yaw
    auto_yaw.set_mode(AUTO_YAW_HOLD);
}

bool Mode4DAuto::is_taking_off() const
{
    return four_d_mode == FourDAuto_TakeOff;
}

// initialise 4D mode's angle controller
void Mode4DAuto::angle_control_start()
{
    // set four_d_mode to velocity controller
	four_d_mode = FourDAuto_Angle;

    // set vertical speed and acceleration
    pos_control->set_max_speed_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up());
    pos_control->set_max_accel_z(wp_nav->get_accel_z());

    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    // initialise targets
    four_d_auto_angle_state.update_time_ms = millis();
    four_d_auto_angle_state.roll_cd = ahrs.roll_sensor;
    four_d_auto_angle_state.pitch_cd = ahrs.pitch_sensor;
    four_d_auto_angle_state.yaw_cd = ahrs.yaw_sensor;
    four_d_auto_angle_state.climb_rate_cms = 0.0f;
    four_d_auto_angle_state.yaw_rate_cds = 0.0f;
    four_d_auto_angle_state.use_yaw_rate = false;

    // pilot always controls yaw
    auto_yaw.set_mode(AUTO_YAW_HOLD);
}

// guided_set_destination - sets guided mode's target destination
// Returns true if the fence is enabled and guided waypoint is within the fence
// else return false if the waypoint is outside the fence
bool Mode4DAuto::set_destination(const Vector3f& destination, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw, bool terrain_alt)
{
    // ensure we are in position control mode
    if (four_d_mode != FourDAuto_WP) {
        pos_control_start();
    }

#if AC_FENCE == ENABLED
    // reject destination if outside the fence
    const Location dest_loc(destination);
    if (!copter.fence.check_destination_within_fence(dest_loc)) {
        AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
        // failure is propagated to GCS with NAK
        return false;
    }
#endif

    // set yaw state
    set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

    // no need to check return status because terrain data is not used
    wp_nav->set_wp_destination(destination, terrain_alt);

    // log target
    copter.Log_Write_GuidedTarget(four_d_mode, destination, Vector3f());
    return true;
}

bool Mode4DAuto::get_wp(Location& destination)
{
    if (four_d_mode != FourDAuto_WP) {
        return false;
    }
    return wp_nav->get_oa_wp_destination(destination);
}

// sets guided mode's target from a Location object
// returns false if destination could not be set (probably caused by missing terrain data)
// or if the fence is enabled and guided waypoint is outside the fence
bool Mode4DAuto::set_destination(const Location& dest_loc, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw)
{
    // ensure we are in position control mode
    if (four_d_mode != FourDAuto_WP) {
        pos_control_start();
    }

#if AC_FENCE == ENABLED
    // reject destination outside the fence.
    // Note: there is a danger that a target specified as a terrain altitude might not be checked if the conversion to alt-above-home fails
    if (!copter.fence.check_destination_within_fence(dest_loc)) {
        AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
        // failure is propagated to GCS with NAK
        return false;
    }
#endif

    if (!wp_nav->set_wp_destination(dest_loc)) {
        // failure to set destination can only be because of missing terrain data
        AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::FAILED_TO_SET_DESTINATION);
        // failure is propagated to GCS with NAK
        return false;
    }

    // set yaw state
    set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

    // log target
    copter.Log_Write_GuidedTarget(four_d_mode, Vector3f(dest_loc.lat, dest_loc.lng, dest_loc.alt),Vector3f());
    return true;
}

// fourdauto_set_velocity - sets 4D mode's target velocity
void Mode4DAuto::set_velocity(const Vector3f& velocity, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw, bool log_request)
{
    // check we are in velocity control mode
    if (four_d_mode != FourDAuto_Velocity) {
        vel_control_start();
    }

    // set yaw state
    set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

    // record velocity target
    guided_vel_target_cms = velocity;
    vel_update_time_ms = millis();

    // log target
    if (log_request) {
        copter.Log_Write_GuidedTarget(four_d_mode, Vector3f(), velocity);
    }
}

// set guided mode posvel target
bool Mode4DAuto::set_destination_posvel(const Vector3f& destination, const Vector3f& velocity, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw)
{
    // check we are in velocity control mode
    if (four_d_mode != FourDAuto_PosVel) {
        posvel_control_start();
    }

#if AC_FENCE == ENABLED
    // reject destination if outside the fence
    const Location dest_loc(destination);
    if (!copter.fence.check_destination_within_fence(dest_loc)) {
        AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
        // failure is propagated to GCS with NAK
        return false;
    }
#endif

    // set yaw state
    set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

    posvel_update_time_ms = millis();
    four_d_pos_target_cm = destination;
    four_d_vel_target_cms = velocity;

    copter.pos_control->set_pos_target(four_d_pos_target_cm);

    // log target #TODO: check if this new function works. If not try to use the same log function for guided mode and change the target_type to fourd mode
    copter.Log_Write_4DAutoTarget(four_d_mode, destination, velocity);
    return true;
}

// set 4D mode angle target
void Mode4DAuto::set_angle(const Quaternion &q, float climb_rate_cms, bool use_yaw_rate, float yaw_rate_rads)
{
    // check we are in velocity control mode
    if (four_d_mode != FourDAuto_Angle) {
        angle_control_start();
    }

    // convert quaternion to euler angles
    q.to_euler(four_d_auto_angle_state.roll_cd, four_d_auto_angle_state.pitch_cd, four_d_auto_angle_state.yaw_cd);
    four_d_auto_angle_state.roll_cd = ToDeg(four_d_auto_angle_state.roll_cd) * 100.0f;
    four_d_auto_angle_state.pitch_cd = ToDeg(four_d_auto_angle_state.pitch_cd) * 100.0f;
    four_d_auto_angle_state.yaw_cd = wrap_180_cd(ToDeg(four_d_auto_angle_state.yaw_cd) * 100.0f);
    four_d_auto_angle_state.yaw_rate_cds = ToDeg(yaw_rate_rads) * 100.0f;
    four_d_auto_angle_state.use_yaw_rate = use_yaw_rate;

    four_d_auto_angle_state.climb_rate_cms = climb_rate_cms;
    four_d_auto_angle_state.update_time_ms = millis();

    // interpret positive climb rate as triggering take-off
    if (motors->armed() && !copter.ap.auto_armed && (four_d_auto_angle_state.climb_rate_cms > 0.0f)) {
        copter.set_auto_armed(true);
    }

    // log target
    copter.Log_Write_4DAutoTarget(four_d_mode,
                           Vector3f(four_d_auto_angle_state.roll_cd, four_d_auto_angle_state.pitch_cd, four_d_auto_angle_state.yaw_cd),
                           Vector3f(0.0f, 0.0f, four_d_auto_angle_state.climb_rate_cms));
}

// 4dauto_run - runs 4d controller
// should be called at 100hz or more
void Mode4DAuto::run()
{
    // call the correct auto controller
    switch (four_d_mode) {

    case FourDAuto_TakeOff:
        takeoff_run();
        break;

    case FourDAuto_WP:
    	pos_control_run();
    	break;

    }
}

// 4D_takeoff_run - takeoff in 4D mode
//      called by 4D_run at 100hz or more
void Mode4DAuto::takeoff_run()
{
    auto_takeoff_run();
    if (wp_nav->reached_wp_destination()) {
        // optionally retract landing gear
        copter.landinggear.retract_after_takeoff();

        // switch to position control mode but maintain current target
        const Vector3f target = wp_nav->get_wp_destination();
        set_destination(target, false, 0, false, 0, false, wp_nav->origin_and_destination_are_terrain_alt());
    }
}

// 4D_pos_control_run - runs the 4D position controller
// called from guided_run
void Mode4DAuto::pos_control_run()
{
    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_spool_down();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

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

// helper function to set yaw state and targets
void Mode4DAuto::set_yaw_state(bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_angle)
{
    if (use_yaw) {
        auto_yaw.set_fixed_yaw(yaw_cd * 0.01f, 0.0f, 0, relative_angle);
    } else if (use_yaw_rate) {
        auto_yaw.set_rate(yaw_rate_cds);
    }
}

#endif
