/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

/*
 * control_guided.pde - init and run calls for guided flight mode
 */

#ifndef GUIDED_LOOK_AT_TARGET_MIN_DISTANCE_CM
 # define GUIDED_LOOK_AT_TARGET_MIN_DISTANCE_CM     500     // point nose at target if it is more than 5m away
#endif

#define GUIDED_POSVEL_TIMEOUT_MS    3000    // guided mode's position-velocity controller times out after 3seconds with no new updates
#define GUIDED_ATTITUDE_TIMEOUT_MS  1000    // guided mode's attitude controller times out after 1 second with no new updates

static Vector3f posvel_pos_target_cm;
static Vector3f posvel_vel_target_cms;
static uint32_t posvel_update_time_ms;
static uint32_t vel_update_time_ms;

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
bool Copter::guided_init(bool ignore_checks)
{
    if (position_ok() || ignore_checks) {
        // initialise yaw
        set_auto_yaw_mode(get_default_auto_yaw_mode(false));
        // start in position control mode
        guided_pos_control_start();
        return true;
    }else{
        return false;
    }
}


// guided_takeoff_start - initialises waypoint controller to implement take-off
void Copter::guided_takeoff_start(float final_alt_above_home)
{
    guided_mode = Guided_TakeOff;
    
    // initialise wpnav destination
    Vector3f target_pos = inertial_nav.get_position();
    target_pos.z = pv_alt_above_origin(final_alt_above_home);
    wp_nav.set_wp_destination(target_pos);

    // initialise yaw
    set_auto_yaw_mode(AUTO_YAW_HOLD);

    // clear i term when we're taking off
    set_throttle_takeoff();
}

// initialise guided mode's position controller
void Copter::guided_pos_control_start()
{
    // set to position control mode
    guided_mode = Guided_WP;

    // initialise waypoint and spline controller
    wp_nav.wp_and_spline_init();

    // initialise wpnav to stopping point at current altitude
    // To-Do: set to current location if disarmed?
    // To-Do: set to stopping point altitude?
    Vector3f stopping_point;
    stopping_point.z = inertial_nav.get_altitude();
    wp_nav.get_wp_stopping_point_xy(stopping_point);
    wp_nav.set_wp_destination(stopping_point);

    // initialise yaw
    set_auto_yaw_mode(get_default_auto_yaw_mode(false));
}

// initialise guided mode's velocity controller
void Copter::guided_vel_control_start()
{
    // set guided_mode to velocity controller
    guided_mode = Guided_Velocity;

    // initialize vertical speeds and leash lengths
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);

    // initialise velocity controller
    pos_control.init_vel_controller_xyz();
}

// initialise guided mode's posvel controller
void Copter::guided_posvel_control_start()
{
    // set guided_mode to velocity controller
    guided_mode = Guided_PosVel;

    pos_control.init_xy_controller();

    // set speed and acceleration from wpnav's speed and acceleration
    pos_control.set_speed_xy(wp_nav.get_speed_xy());
    pos_control.set_accel_xy(wp_nav.get_wp_acceleration());

    const Vector3f& curr_pos = inertial_nav.get_position();
    const Vector3f& curr_vel = inertial_nav.get_velocity();

    // set target position and velocity to current position and velocity
    pos_control.set_xy_target(curr_pos.x, curr_pos.y);
    pos_control.set_desired_velocity_xy(curr_vel.x, curr_vel.y);

    // set vertical speed and acceleration
    pos_control.set_speed_z(wp_nav.get_speed_down(), wp_nav.get_speed_up());
    pos_control.set_accel_z(wp_nav.get_accel_z());

    // pilot always controls yaw
    set_auto_yaw_mode(AUTO_YAW_HOLD);
}

// initialise guided mode's angle controller
void Copter::guided_angle_control_start()
{
    // set guided_mode to velocity controller
    guided_mode = Guided_Angle;

    // set vertical speed and acceleration
    pos_control.set_speed_z(wp_nav.get_speed_down(), wp_nav.get_speed_up());
    pos_control.set_accel_z(wp_nav.get_accel_z());

    // initialise altitude target to stopping point
    pos_control.set_target_to_stopping_point_z();

    // initialise targets
    guided_angle_state.update_time_ms = millis();
    guided_angle_state.roll_cd = ahrs.roll_sensor;
    guided_angle_state.pitch_cd = ahrs.pitch_sensor;
    guided_angle_state.yaw_cd = ahrs.yaw_sensor;
    guided_angle_state.climb_rate_cms = 0.0f;

    // pilot always controls yaw
    set_auto_yaw_mode(AUTO_YAW_HOLD);
}

// guided_set_destination - sets guided mode's target destination
void Copter::guided_set_destination(const Vector3f& destination)
{
    // ensure we are in position control mode
    if (guided_mode != Guided_WP) {
        guided_pos_control_start();
    }

    wp_nav.set_wp_destination(destination);
}

// guided_set_velocity - sets guided mode's target velocity
void Copter::guided_set_velocity(const Vector3f& velocity)
{
    // check we are in velocity control mode
    if (guided_mode != Guided_Velocity) {
        guided_vel_control_start();
    }

    vel_update_time_ms = millis();

    // set position controller velocity target
    pos_control.set_desired_velocity(velocity);
}

// set guided mode posvel target
void Copter::guided_set_destination_posvel(const Vector3f& destination, const Vector3f& velocity) {
    // check we are in velocity control mode
    if (guided_mode != Guided_PosVel) {
        guided_posvel_control_start();
    }

    posvel_update_time_ms = millis();
    posvel_pos_target_cm = destination;
    posvel_vel_target_cms = velocity;

    pos_control.set_pos_target(posvel_pos_target_cm);
}

// set guided mode angle target
void Copter::guided_set_angle(const Quaternion &q, float climb_rate_cms)
{
    // check we are in velocity control mode
    if (guided_mode != Guided_Angle) {
        guided_angle_control_start();
    }

    // convert quaternion to euler angles
    q.to_euler(guided_angle_state.roll_cd, guided_angle_state.pitch_cd, guided_angle_state.yaw_cd);
    guided_angle_state.roll_cd = ToDeg(guided_angle_state.roll_cd) * 100.0f;
    guided_angle_state.pitch_cd = ToDeg(guided_angle_state.pitch_cd) * 100.0f;
    guided_angle_state.yaw_cd = wrap_180_cd_float(ToDeg(guided_angle_state.yaw_cd) * 100.0f);

    guided_angle_state.climb_rate_cms = climb_rate_cms;
    guided_angle_state.update_time_ms = millis();
}

// guided_run - runs the guided controller
// should be called at 100hz or more
void Copter::guided_run()
{
    // call the correct auto controller
    switch (guided_mode) {

    case Guided_TakeOff:
        // run takeoff controller
        guided_takeoff_run();
        break;

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

// guided_takeoff_run - takeoff in guided mode
//      called by guided_run at 100hz or more
void Copter::guided_takeoff_run()
{
    // if not auto armed or motors not enabled set throttle to zero and exit immediately
    if (!ap.auto_armed || !motors.get_interlock()) {
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(0, 0, 0, get_smoothing_gain());
        attitude_control.set_throttle_out(0,false,g.throttle_filt);
#else   // multicopters do not stabilize roll/pitch/yaw when disarmed
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
#endif
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->control_in);
    }

    // run waypoint controller
    wp_nav.update_wpnav();

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control.update_z_controller();

    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
}

// guided_pos_control_run - runs the guided position controller
// called from guided_run
void Copter::guided_pos_control_run()
{
    // if not auto armed or motors not enabled set throttle to zero and exit immediately
    if (!ap.auto_armed || !motors.get_interlock() || ap.land_complete) {
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(0, 0, 0, get_smoothing_gain());
        attitude_control.set_throttle_out(0,false,g.throttle_filt);
#else   // multicopters do not stabilize roll/pitch/yaw when disarmed
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
#endif
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->control_in);
        if (!is_zero(target_yaw_rate)) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
    }

    // run waypoint controller
    wp_nav.update_wpnav();

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control.update_z_controller();

    // call attitude controller
    if (auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
    }else{
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control.angle_ef_roll_pitch_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), get_auto_heading(), true);
    }
}

// guided_vel_control_run - runs the guided velocity controller
// called from guided_run
void Copter::guided_vel_control_run()
{
    // if not auto armed or motors not enabled set throttle to zero and exit immediately
    if (!ap.auto_armed || !motors.get_interlock() || ap.land_complete) {
        // initialise velocity controller
        pos_control.init_vel_controller_xyz();
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(0, 0, 0, get_smoothing_gain());
        attitude_control.set_throttle_out(0,false,g.throttle_filt);
#else   // multicopters do not stabilize roll/pitch/yaw when disarmed
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
#endif
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->control_in);
        if (!is_zero(target_yaw_rate)) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
    }

    // set velocity to zero if no updates received for 3 seconds
    uint32_t tnow = millis();
    if (tnow - vel_update_time_ms > GUIDED_POSVEL_TIMEOUT_MS && !pos_control.get_desired_velocity().is_zero()) {
        pos_control.set_desired_velocity(Vector3f(0,0,0));
    }

    // calculate dt
    float dt = pos_control.time_since_last_xy_update();

    // update at poscontrol update rate
    if (dt >= pos_control.get_dt_xy()) {
        // sanity check dt
        if (dt >= 0.2f) {
            dt = 0.0f;
        }

        // call velocity controller which includes z axis controller
        pos_control.update_vel_controller_xyz(ekfNavVelGainScaler);
    }

    // call attitude controller
    if (auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw(pos_control.get_roll(), pos_control.get_pitch(), target_yaw_rate);
    }else{
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control.angle_ef_roll_pitch_yaw(pos_control.get_roll(), pos_control.get_pitch(), get_auto_heading(), true);
    }
}

// guided_posvel_control_run - runs the guided spline controller
// called from guided_run
void Copter::guided_posvel_control_run()
{
    // if not auto armed or motors not enabled set throttle to zero and exit immediately
    if (!ap.auto_armed || !motors.get_interlock() || ap.land_complete) {
        // set target position and velocity to current position and velocity
        pos_control.set_pos_target(inertial_nav.get_position());
        pos_control.set_desired_velocity(Vector3f(0,0,0));
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(0, 0, 0, get_smoothing_gain());
        attitude_control.set_throttle_out(0,false,g.throttle_filt);
#else   // multicopters do not stabilize roll/pitch/yaw when disarmed
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
#endif
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;

    if (!failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->control_in);
        if (!is_zero(target_yaw_rate)) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
    }

    // set velocity to zero if no updates received for 3 seconds
    uint32_t tnow = millis();
    if (tnow - posvel_update_time_ms > GUIDED_POSVEL_TIMEOUT_MS && !posvel_vel_target_cms.is_zero()) {
        posvel_vel_target_cms.zero();
    }

    // calculate dt
    float dt = pos_control.time_since_last_xy_update();

    // update at poscontrol update rate
    if (dt >= pos_control.get_dt_xy()) {
        // sanity check dt
        if (dt >= 0.2f) {
            dt = 0.0f;
        }

        // advance position target using velocity target
        posvel_pos_target_cm += posvel_vel_target_cms * dt;

        // send position and velocity targets to position controller
        pos_control.set_pos_target(posvel_pos_target_cm);
        pos_control.set_desired_velocity_xy(posvel_vel_target_cms.x, posvel_vel_target_cms.y);

        // run position controller
        pos_control.update_xy_controller(AC_PosControl::XY_MODE_POS_AND_VEL_FF, ekfNavVelGainScaler, false);
    }

    pos_control.update_z_controller();

    // call attitude controller
    if (auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw(pos_control.get_roll(), pos_control.get_pitch(), target_yaw_rate);
    }else{
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control.angle_ef_roll_pitch_yaw(pos_control.get_roll(), pos_control.get_pitch(), get_auto_heading(), true);
    }
}

// guided_angle_control_run - runs the guided angle controller
// called from guided_run
void Copter::guided_angle_control_run()
{
    // if not auto armed or motors not enabled set throttle to zero and exit immediately
    if (!ap.auto_armed || !motors.get_interlock() || ap.land_complete) {
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control.set_yaw_target_to_current_heading();
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(0.0f, 0.0f, 0.0f, get_smoothing_gain());
        attitude_control.set_throttle_out(0.0f,false,g.throttle_filt);
#else   // multicopters do not stabilize roll/pitch/yaw when disarmed
        attitude_control.set_throttle_out_unstabilized(0.0f,true,g.throttle_filt);
#endif
        pos_control.relax_alt_hold_controllers(0.0f);
        return;
    }

    // constrain desired lean angles
    float roll_in = guided_angle_state.roll_cd;
    float pitch_in = guided_angle_state.pitch_cd;
    float total_in = pythagorous2(roll_in, pitch_in);
    float angle_max = min(attitude_control.get_althold_lean_angle_max(), aparm.angle_max);
    if (total_in > angle_max) {
        float ratio = angle_max / total_in;
        roll_in *= ratio;
        pitch_in *= ratio;
    }

    // wrap yaw request
    float yaw_in = wrap_180_cd_float(guided_angle_state.yaw_cd);

    // constrain climb rate
    float climb_rate_cms = constrain_float(guided_angle_state.climb_rate_cms, -fabs(wp_nav.get_speed_down()), wp_nav.get_speed_up());

    // check for timeout - set lean angles and climb rate to zero if no updates received for 3 seconds
    uint32_t tnow = millis();
    if (tnow - guided_angle_state.update_time_ms > GUIDED_ATTITUDE_TIMEOUT_MS) {
        roll_in = 0.0f;
        pitch_in = 0.0f;
        climb_rate_cms = 0.0f;
    }

    // call attitude controller
    attitude_control.angle_ef_roll_pitch_yaw(roll_in, pitch_in, yaw_in, true);

    // call position controller
    pos_control.set_alt_target_from_climb_rate(climb_rate_cms, G_Dt, false);
    pos_control.update_z_controller();
}

// Guided Limit code

// guided_limit_clear - clear/turn off guided limits
void Copter::guided_limit_clear()
{
    guided_limit.timeout_ms = 0;
    guided_limit.alt_min_cm = 0.0f;
    guided_limit.alt_max_cm = 0.0f;
    guided_limit.horiz_max_cm = 0.0f;
}

// guided_limit_set - set guided timeout and movement limits
void Copter::guided_limit_set(uint32_t timeout_ms, float alt_min_cm, float alt_max_cm, float horiz_max_cm)
{
    guided_limit.timeout_ms = timeout_ms;
    guided_limit.alt_min_cm = alt_min_cm;
    guided_limit.alt_max_cm = alt_max_cm;
    guided_limit.horiz_max_cm = horiz_max_cm;
}

// guided_limit_init_time_and_pos - initialise guided start time and position as reference for limit checking
//  only called from AUTO mode's auto_nav_guided_start function
void Copter::guided_limit_init_time_and_pos()
{
    // initialise start time
    guided_limit.start_time = hal.scheduler->millis();

    // initialise start position from current position
    guided_limit.start_pos = inertial_nav.get_position();
}

// guided_limit_check - returns true if guided mode has breached a limit
//  used when guided is invoked from the NAV_GUIDED_ENABLE mission command
bool Copter::guided_limit_check()
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
        float horiz_move = pv_get_horizontal_distance_cm(guided_limit.start_pos, curr_pos);
        if (horiz_move > guided_limit.horiz_max_cm) {
            return true;
        }
    }

    // if we got this far we must be within limits
    return false;
}
