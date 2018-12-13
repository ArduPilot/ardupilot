#include "mode.h"
#include "Rover.h"

Mode::Mode() :
    ahrs(rover.ahrs),
    g(rover.g),
    g2(rover.g2),
    channel_steer(rover.channel_steer),
    channel_throttle(rover.channel_throttle),
    channel_lateral(rover.channel_lateral),
    attitude_control(rover.g2.attitude_control)
{ }

void Mode::exit()
{
    // call sub-classes exit
    _exit();
}

bool Mode::enter()
{
    const bool ignore_checks = !hal.util->get_soft_armed();   // allow switching to any mode if disarmed.  We rely on the arming check to perform
    if (!ignore_checks) {

        // get EKF filter status
        nav_filter_status filt_status;
        rover.ahrs.get_filter_status(filt_status);

        // check position estimate.  requires origin and at least one horizontal position flag to be true
        const bool position_ok = rover.ekf_position_ok() && !rover.failsafe.ekf;
        if (requires_position() && !position_ok) {
            return false;
        }

        // check velocity estimate (if we have position estimate, we must have velocity estimate)
        if (requires_velocity() && !position_ok && !filt_status.flags.horiz_vel) {
            return false;
        }
    }

    bool ret = _enter();

    // initialisation common to all modes
    if (ret) {
        set_reversed(false);

        // clear sailboat tacking flags
        rover.sailboat_clear_tack();
    }

    return ret;
}

// decode pilot steering and throttle inputs and return in steer_out and throttle_out arguments
// steering_out is in the range -4500 ~ +4500 with positive numbers meaning rotate clockwise
// throttle_out is in the range -100 ~ +100
void Mode::get_pilot_input(float &steering_out, float &throttle_out)
{
    // no RC input means no throttle and centered steering
    if (rover.failsafe.bits & FAILSAFE_EVENT_THROTTLE) {
        steering_out = 0;
        throttle_out = 0;
        return;
    }

    // apply RC skid steer mixing
    switch ((enum pilot_steer_type_t)rover.g.pilot_steer_type.get())
    {
        case PILOT_STEER_TYPE_DEFAULT:
        case PILOT_STEER_TYPE_DIR_REVERSED_WHEN_REVERSING:
        default: {
            // by default regular and skid-steering vehicles reverse their rotation direction when backing up
            throttle_out = rover.channel_throttle->get_control_in();
            const float steering_dir = is_negative(throttle_out) ? -1 : 1;
            steering_out = steering_dir * rover.channel_steer->get_control_in();
            break;
        }

        case PILOT_STEER_TYPE_TWO_PADDLES: {
            // convert the two radio_in values from skid steering values
            // left paddle from steering input channel, right paddle from throttle input channel
            // steering = left-paddle - right-paddle
            // throttle = average(left-paddle, right-paddle)
            const float left_paddle = rover.channel_steer->norm_input();
            const float right_paddle = rover.channel_throttle->norm_input();

            throttle_out = 0.5f * (left_paddle + right_paddle) * 100.0f;
            steering_out = (left_paddle - right_paddle) * 0.5f * 4500.0f;
            break;
        }

        case PILOT_STEER_TYPE_DIR_UNCHANGED_WHEN_REVERSING: {
            throttle_out = rover.channel_throttle->get_control_in();
            steering_out = rover.channel_steer->get_control_in();
            break;
        }
    }
}

// decode pilot steering and throttle inputs and return in steer_out and throttle_out arguments
// steering_out is in the range -4500 ~ +4500 with positive numbers meaning rotate clockwise
// throttle_out is in the range -100 ~ +100
void Mode::get_pilot_desired_steering_and_throttle(float &steering_out, float &throttle_out)
{
    // do basic conversion
    get_pilot_input(steering_out, throttle_out);

    // check for special case of input and output throttle being in opposite directions
    float throttle_out_limited = g2.motors.get_slew_limited_throttle(throttle_out, rover.G_Dt);
    if ((is_negative(throttle_out) != is_negative(throttle_out_limited)) &&
        ((g.pilot_steer_type == PILOT_STEER_TYPE_DEFAULT) ||
         (g.pilot_steer_type == PILOT_STEER_TYPE_DIR_REVERSED_WHEN_REVERSING))) {
        steering_out *= -1;
    }
    throttle_out = throttle_out_limited;
}

// decode pilot steering and return steering_out and speed_out (in m/s)
void Mode::get_pilot_desired_steering_and_speed(float &steering_out, float &speed_out)
{
    float desired_throttle;
    get_pilot_input(steering_out, desired_throttle);
    speed_out = desired_throttle * 0.01f * calc_speed_max(g.speed_cruise, g.throttle_cruise * 0.01f);
    // check for special case of input and output throttle being in opposite directions
    float speed_out_limited = g2.attitude_control.get_desired_speed_accel_limited(speed_out, rover.G_Dt);
    if ((is_negative(speed_out) != is_negative(speed_out_limited)) &&
        ((g.pilot_steer_type == PILOT_STEER_TYPE_DEFAULT) ||
         (g.pilot_steer_type == PILOT_STEER_TYPE_DIR_REVERSED_WHEN_REVERSING))) {
        steering_out *= -1;
    }
    speed_out = speed_out_limited;
}

// decode pilot lateral movement input and return in lateral_out argument
void Mode::get_pilot_desired_lateral(float &lateral_out)
{
    // no RC input means no lateral input
    if (rover.failsafe.bits & FAILSAFE_EVENT_THROTTLE) {
        lateral_out = 0;
        return;
    }

    // get pilot lateral input
    lateral_out = rover.channel_lateral->get_control_in();
}

// decode pilot's input and return heading_out (in cd) and speed_out (in m/s)
void Mode::get_pilot_desired_heading_and_speed(float &heading_out, float &speed_out)
{
    // get steering and throttle in the -1 to +1 range
    const float desired_steering = constrain_float(rover.channel_steer->norm_input_dz(), -1.0f, 1.0f);
    const float desired_throttle = constrain_float(rover.channel_throttle->norm_input_dz(), -1.0f, 1.0f);

    // calculate angle of input stick vector
    heading_out = wrap_360_cd(atan2f(desired_steering, desired_throttle) * DEGX100);

    // calculate throttle using magnitude of input stick vector
    const float throttle = MIN(safe_sqrt(sq(desired_throttle) + sq(desired_steering)), 1.0f);
    speed_out = throttle * calc_speed_max(g.speed_cruise, g.throttle_cruise * 0.01f);
}

// return heading (in degrees) to target destination (aka waypoint)
float Mode::wp_bearing() const
{
    if (!is_autopilot_mode()) {
        return 0.0f;
    }
    return rover.nav_controller->target_bearing_cd() * 0.01f;
}

// return short-term target heading in degrees (i.e. target heading back to line between waypoints)
float Mode::nav_bearing() const
{
    if (!is_autopilot_mode()) {
        return 0.0f;
    }
    return rover.nav_controller->nav_bearing_cd() * 0.01f;
}

// return cross track error (i.e. vehicle's distance from the line between waypoints)
float Mode::crosstrack_error() const
{
    if (!is_autopilot_mode()) {
        return 0.0f;
    }
    return rover.nav_controller->crosstrack_error();
}

// set desired location
void Mode::set_desired_location(const struct Location& destination, float next_leg_bearing_cd)
{
    // set origin to last destination if waypoint controller active
    if ((AP_HAL::millis() - last_steer_to_wp_ms < 100) && _reached_destination) {
        _origin = _destination;
    } else {
        // otherwise use reasonable stopping point
        calc_stopping_location(_origin);
    }
    _destination = destination;

    // initialise distance
    _distance_to_destination = get_distance(_origin, _destination);
    _reached_destination = false;

    // set final desired speed
    _desired_speed_final = 0.0f;
    if (!is_equal(next_leg_bearing_cd, MODE_NEXT_HEADING_UNKNOWN)) {
        const float curr_leg_bearing_cd = get_bearing_cd(_origin, _destination);
        const float turn_angle_cd = wrap_180_cd(next_leg_bearing_cd - curr_leg_bearing_cd);
        if (is_zero(turn_angle_cd)) {
            // if not turning can continue at full speed
            _desired_speed_final = _desired_speed;
        } else if (rover.use_pivot_steering_at_next_WP(turn_angle_cd)) {
            // pivoting so we will stop
            _desired_speed_final = 0.0f;
        } else {
            // calculate maximum speed that keeps overshoot within bounds
            const float radius_m = fabsf(g.waypoint_overshoot / (cosf(radians(turn_angle_cd * 0.01f)) - 1.0f));
            _desired_speed_final = MIN(_desired_speed, safe_sqrt(g.turn_max_g * GRAVITY_MSS * radius_m));
        }
    }
}

// set desired location as an offset from the EKF origin in NED frame
bool Mode::set_desired_location_NED(const Vector3f& destination, float next_leg_bearing_cd)
{
    Location destination_ned;
    // initialise destination to ekf origin
    if (!ahrs.get_origin(destination_ned)) {
        return false;
    }
    // apply offset
    location_offset(destination_ned, destination.x, destination.y);
    set_desired_location(destination_ned, next_leg_bearing_cd);
    return true;
}

// set desired heading and speed
void Mode::set_desired_heading_and_speed(float yaw_angle_cd, float target_speed)
{
    // handle initialisation
    _reached_destination = false;

    // record targets
    _desired_yaw_cd = yaw_angle_cd;
    _desired_speed = target_speed;
}

// get default speed for this mode (held in (CRUISE_SPEED, WP_SPEED or RTL_SPEED)
float Mode::get_speed_default(bool rtl) const
{
    if (rtl && is_positive(g2.rtl_speed)) {
        return g2.rtl_speed;
    } else if (is_positive(g2.wp_speed)) {
        return g2.wp_speed;
    } else {
        return g.speed_cruise;
    }
}

// restore desired speed to default from parameter values (CRUISE_SPEED or WP_SPEED)
void Mode::set_desired_speed_to_default(bool rtl)
{
    _desired_speed = get_speed_default(rtl);
}

// set desired speed in m/s
bool Mode::set_desired_speed(float speed)
{
    if (!is_negative(speed)) {
        _desired_speed = speed;
        return true;
    }
    return false;
}

// execute the mission in reverse (i.e. backing up)
void Mode::set_reversed(bool value)
{
    _reversed = value;
}

// handle tacking request (from auxiliary switch) in sailboats
void Mode::handle_tack_request()
{
    // autopilot modes handle tacking
    if (is_autopilot_mode()) {
        rover.sailboat_handle_tack_request_auto();
    }
}

void Mode::calc_throttle(float target_speed, bool nudge_allowed, bool avoidance_enabled)
{
    // add in speed nudging
    if (nudge_allowed) {
        target_speed = calc_speed_nudge(target_speed, g.speed_cruise, g.throttle_cruise * 0.01f);
    }

    // get acceleration limited target speed
    target_speed = attitude_control.get_desired_speed_accel_limited(target_speed, rover.G_Dt);

    // apply object avoidance to desired speed using half vehicle's maximum deceleration
    if (avoidance_enabled) {
        g2.avoid.adjust_speed(0.0f, 0.5f * attitude_control.get_decel_max(), ahrs.yaw, target_speed, rover.G_Dt);
    }

    // call throttle controller and convert output to -100 to +100 range
    float throttle_out;

    // call speed or stop controller
    if (is_zero(target_speed) && !rover.is_balancebot()) {
        bool stopped;
        throttle_out = 100.0f * attitude_control.get_throttle_out_stop(g2.motors.limit.throttle_lower, g2.motors.limit.throttle_upper, g.speed_cruise, g.throttle_cruise * 0.01f, rover.G_Dt, stopped);
    } else {
        throttle_out = 100.0f * attitude_control.get_throttle_out_speed(target_speed, g2.motors.limit.throttle_lower, g2.motors.limit.throttle_upper, g.speed_cruise, g.throttle_cruise * 0.01f, rover.G_Dt);
    }

    // if vehicle is balance bot, calculate actual throttle required for balancing
    if (rover.is_balancebot()) {
        rover.balancebot_pitch_control(throttle_out);
    }

    // update mainsail position if present
    rover.sailboat_update_mainsail(target_speed);

    // send to motor
    g2.motors.set_throttle(throttle_out);
}

// performs a controlled stop with steering centered
bool Mode::stop_vehicle()
{
    // call throttle controller and convert output to -100 to +100 range
    bool stopped = false;
    float throttle_out = 100.0f * attitude_control.get_throttle_out_stop(g2.motors.limit.throttle_lower, g2.motors.limit.throttle_upper, g.speed_cruise, g.throttle_cruise * 0.01f, rover.G_Dt, stopped);

    // if vehicle is balance bot, calculate actual throttle required for balancing
    if (rover.is_balancebot()) {
        throttle_out = 100.0f * attitude_control.get_throttle_out_speed(0, g2.motors.limit.throttle_lower, g2.motors.limit.throttle_upper, g.speed_cruise, g.throttle_cruise * 0.01f, rover.G_Dt);
        rover.balancebot_pitch_control(throttle_out);
    }

    // relax mainsail if present
    g2.motors.set_mainsail(100.0f);

    // send to motor
    g2.motors.set_throttle(throttle_out);

    // do not attempt to steer
    g2.motors.set_steering(0.0f);

    // return true once stopped
    return stopped;
}

// estimate maximum vehicle speed (in m/s)
// cruise_speed is in m/s, cruise_throttle should be in the range -1 to +1
float Mode::calc_speed_max(float cruise_speed, float cruise_throttle) const
{
    float speed_max;

    // sanity checks
    if (cruise_throttle > 1.0f || cruise_throttle < 0.05f) {
        speed_max = cruise_speed;
    } else {
        // project vehicle's maximum speed
        speed_max = (1.0f / cruise_throttle) * cruise_speed;
    }

    // constrain to 30m/s (108km/h) and return
    return constrain_float(speed_max, 0.0f, 30.0f);
}

// calculate pilot input to nudge speed up or down
//  target_speed should be in meters/sec
//  cruise_speed is vehicle's cruising speed, cruise_throttle is the throttle (from -1 to +1) that achieves the cruising speed
//  return value is a new speed (in m/s) which up to the projected maximum speed based on the cruise speed and cruise throttle
float Mode::calc_speed_nudge(float target_speed, float cruise_speed, float cruise_throttle)
{
    // return immediately during RC/GCS failsafe
    if (rover.failsafe.bits & FAILSAFE_EVENT_THROTTLE) {
        return target_speed;
    }

    // return immediately if pilot is not attempting to nudge speed
    // pilot can nudge up speed if throttle (in range -100 to +100) is above 50% of center in direction of travel
    const int16_t pilot_throttle = constrain_int16(rover.channel_throttle->get_control_in(), -100, 100);
    if (((pilot_throttle <= 50) && (target_speed >= 0.0f)) ||
        ((pilot_throttle >= -50) && (target_speed <= 0.0f))) {
        return target_speed;
    }

    // sanity checks
    if (cruise_throttle > 1.0f || cruise_throttle < 0.05f) {
        return target_speed;
    }

    // project vehicle's maximum speed
    const float vehicle_speed_max = calc_speed_max(cruise_speed, cruise_throttle);

    // return unadjusted target if already over vehicle's projected maximum speed
    if (fabsf(target_speed) >= vehicle_speed_max) {
        return target_speed;
    }

    const float speed_increase_max = vehicle_speed_max - fabsf(target_speed);
    float speed_nudge = ((static_cast<float>(abs(pilot_throttle)) - 50.0f) * 0.02f) * speed_increase_max;
    if (pilot_throttle < 0) {
        speed_nudge = -speed_nudge;
    }

    return target_speed + speed_nudge;
}

// calculated a reduced speed(in m/s) based on yaw error and lateral acceleration and/or distance to a waypoint
// should be called after calc_lateral_acceleration and before calc_throttle
// relies on these internal members being updated: _yaw_error_cd, _distance_to_destination
float Mode::calc_reduced_speed_for_turn_or_distance(float desired_speed)
{
    // reduce speed to zero during pivot turns
    if (rover.use_pivot_steering(_yaw_error_cd)) {
        return 0.0f;
    }

    // reduce speed to limit overshoot from line between origin and destination
    // calculate number of degrees vehicle must turn to face waypoint
    const float heading_cd = is_negative(desired_speed) ? wrap_180_cd(ahrs.yaw_sensor + 18000) : ahrs.yaw_sensor;
    const float wp_yaw_diff = wrap_180_cd(rover.nav_controller->target_bearing_cd() - heading_cd);
    const float turn_angle_rad = fabsf(radians(wp_yaw_diff * 0.01f));

    // calculate distance from vehicle to line + wp_overshoot
    const float line_yaw_diff = wrap_180_cd(get_bearing_cd(_origin, _destination) - heading_cd);
    const float xtrack_error = rover.nav_controller->crosstrack_error();
    const float dist_from_line = fabsf(xtrack_error);
    const bool heading_away = is_positive(line_yaw_diff) == is_positive(xtrack_error);
    const float wp_overshoot_adj = heading_away ? -dist_from_line : dist_from_line;

    // calculate radius of circle that touches vehicle's current position and heading and target position and heading
    float radius_m = 999.0f;
    float radius_calc_denom = fabsf(1.0f - cosf(turn_angle_rad));
    if (!is_zero(radius_calc_denom)) {
        radius_m = MAX(0.0f, rover.g.waypoint_overshoot + wp_overshoot_adj) / radius_calc_denom;
    }

    // calculate and limit speed to allow vehicle to stay on circle
    float overshoot_speed_max = safe_sqrt(g.turn_max_g * GRAVITY_MSS * MAX(g2.turn_radius, radius_m));
    float speed_max = constrain_float(desired_speed, -overshoot_speed_max, overshoot_speed_max);

    // limit speed based on distance to waypoint and max acceleration/deceleration
    if (is_positive(_distance_to_destination) && is_positive(attitude_control.get_decel_max())) {
        const float dist_speed_max = safe_sqrt(2.0f * _distance_to_destination * attitude_control.get_decel_max() + sq(_desired_speed_final));
        speed_max = constrain_float(speed_max, -dist_speed_max, dist_speed_max);
    }

    // return minimum speed
    return speed_max;
}

// calculate the lateral acceleration target to cause the vehicle to drive along the path from origin to destination
// this function updates the _yaw_error_cd value
void Mode::calc_steering_to_waypoint(const struct Location &origin, const struct Location &destination, bool reversed)
{
    // record system time of call
    last_steer_to_wp_ms = AP_HAL::millis();

    // Calculate the required turn of the wheels
    // negative error = left turn
    // positive error = right turn
    rover.nav_controller->set_reverse(reversed);
    rover.nav_controller->update_waypoint(origin, destination, g.waypoint_radius);
    float desired_lat_accel = rover.nav_controller->lateral_acceleration();
    float desired_heading = rover.nav_controller->target_bearing_cd();
    if (reversed) {
        desired_heading = wrap_360_cd(desired_heading + 18000);
        desired_lat_accel *= -1.0f;
    }
    _yaw_error_cd = wrap_180_cd(desired_heading - ahrs.yaw_sensor);

    if (rover.sailboat_use_indirect_route(desired_heading)) {
        // sailboats use heading controller when tacking upwind
        desired_heading = rover.sailboat_calc_heading(desired_heading);
        calc_steering_to_heading(desired_heading, g2.pivot_turn_rate);
    } else if (rover.use_pivot_steering(_yaw_error_cd)) {
        // for pivot turns use heading controller
        calc_steering_to_heading(desired_heading, g2.pivot_turn_rate);
    } else {
        // call lateral acceleration to steering controller
        calc_steering_from_lateral_acceleration(desired_lat_accel, reversed);
    }
}

/*
    calculate steering output given lateral_acceleration
*/
void Mode::calc_steering_from_lateral_acceleration(float lat_accel, bool reversed)
{
    // add obstacle avoidance response to lateral acceleration target
    // ToDo: replace this type of object avoidance with path planning
    if (!reversed) {
        lat_accel += (rover.obstacle.turn_angle / 45.0f) * g.turn_max_g;
    }

    // constrain to max G force
    lat_accel = constrain_float(lat_accel, -g.turn_max_g * GRAVITY_MSS, g.turn_max_g * GRAVITY_MSS);

    // send final steering command to motor library
    const float steering_out = attitude_control.get_steering_out_lat_accel(lat_accel,
                                                                           g2.motors.limit.steer_left,
                                                                           g2.motors.limit.steer_right,
                                                                           rover.G_Dt);
    g2.motors.set_steering(steering_out * 4500.0f);
}

// calculate steering output to drive towards desired heading
// rate_max is a maximum turn rate in deg/s.  set to zero to use default turn rate limits
void Mode::calc_steering_to_heading(float desired_heading_cd, float rate_max_degs)
{
    // calculate yaw error so it can be used for reporting and slowing the vehicle
    _yaw_error_cd = wrap_180_cd(desired_heading_cd - ahrs.yaw_sensor);

    // call heading controller
    const float steering_out = attitude_control.get_steering_out_heading(radians(desired_heading_cd*0.01f),
                                                                         radians(rate_max_degs),
                                                                         g2.motors.limit.steer_left,
                                                                         g2.motors.limit.steer_right,
                                                                         rover.G_Dt);
    g2.motors.set_steering(steering_out * 4500.0f);
}

// calculate vehicle stopping point using current location, velocity and maximum acceleration
void Mode::calc_stopping_location(Location& stopping_loc)
{
    // default stopping location
    stopping_loc = rover.current_loc;

    // get current velocity vector and speed
    const Vector2f velocity = ahrs.groundspeed_vector();
    const float speed = velocity.length();

    // avoid divide by zero
    if (!is_positive(speed)) {
        stopping_loc = rover.current_loc;
        return;
    }

    // get stopping distance in meters
    const float stopping_dist = attitude_control.get_stopping_distance(speed);

    // calculate stopping position from current location in meters
    const Vector2f stopping_offset = velocity.normalized() * stopping_dist;

    location_offset(stopping_loc, stopping_offset.x, stopping_offset.y);
}
