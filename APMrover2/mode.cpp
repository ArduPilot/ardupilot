#include "mode.h"
#include "Rover.h"

Mode::Mode() :
    ahrs(rover.ahrs),
    g(rover.g),
    g2(rover.g2),
    channel_steer(rover.channel_steer),
    channel_throttle(rover.channel_throttle),
    mission(rover.mission),
    attitude_control(rover.g2.attitude_control)
{ }

void Mode::exit()
{
    // call sub-classes exit
    _exit();

    lateral_acceleration = 0.0f;
}

bool Mode::enter()
{
    g2.motors.slew_limit_throttle(false);
    return _enter();
}

// set desired location
void Mode::set_desired_location(const struct Location& destination, float next_leg_bearing_cd)
{
    // record targets
    _origin = rover.current_loc;
    _destination = destination;
    _desired_speed = g.speed_cruise;

    // initialise distance
    _distance_to_destination = get_distance(_origin, _destination);
    _reached_destination = false;

    // set final desired speed
    _desired_speed_final = 0.0f;
    if (!is_equal(next_leg_bearing_cd, MODE_NEXT_HEADING_UNKNOWN)) {
        // if not turning can continue at full speed
        if (is_zero(next_leg_bearing_cd)) {
            _desired_speed_final = _desired_speed;
        } else {
            // calculate maximum speed that keeps overshoot within bounds
            const float curr_leg_bearing_cd = get_bearing_cd(_origin, _destination);
            const float turn_angle_cd = wrap_180_cd(next_leg_bearing_cd - curr_leg_bearing_cd);
            const float radius_m = fabsf(g.waypoint_overshoot / (cosf(radians(turn_angle_cd * 0.01f)) - 1.0f));
            _desired_speed_final = MIN(_desired_speed, safe_sqrt(g.turn_max_g * GRAVITY_MSS * radius_m));
        }
    }
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

void Mode::calc_throttle(float target_speed, bool nudge_allowed)
{
    // add in speed nudging
    if (nudge_allowed) {
        target_speed = calc_speed_nudge(target_speed, g.speed_cruise, g.throttle_cruise * 0.01f);
    }

    // call throttle controller and convert output to -100 to +100 range
    float throttle_out = 100.0f * attitude_control.get_throttle_out_speed(target_speed, g2.motors.have_skid_steering(), g2.motors.limit.throttle_lower, g2.motors.limit.throttle_upper, g.speed_cruise, g.throttle_cruise * 0.01f);

    // send to motor
    g2.motors.set_throttle(throttle_out);
}

// performs a controlled stop with steering centered
bool Mode::stop_vehicle()
{
    // call throttle controller and convert output to -100 to +100 range
    bool stopped = false;
    float throttle_out = 100.0f * attitude_control.get_throttle_out_stop(g2.motors.have_skid_steering(), g2.motors.limit.throttle_lower, g2.motors.limit.throttle_upper, g.speed_cruise, g.throttle_cruise * 0.01f, stopped);

    // send to motor
    g2.motors.set_throttle(throttle_out);

    // do not attempt to steer
    g2.motors.set_steering(0.0f);

    // return true once stopped
    return stopped;
}

// estimate maximum vehicle speed (in m/s)
float Mode::calc_speed_max(float cruise_speed, float cruise_throttle)
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
    // return immediately if pilot is not attempting to nudge speed
    // pilot can nudge up speed if throttle (in range -100 to +100) is above 50% of center in direction of travel
    const int16_t pilot_throttle = constrain_int16(rover.channel_throttle->get_control_in(), -100, 100);
    if ((pilot_throttle <= 50 && is_positive(target_speed)) ||
        (pilot_throttle >= -50 && is_negative(target_speed))) {
        return target_speed;
    }

    // sanity checks
    if (cruise_throttle > 1.0f || cruise_throttle < 0.05f) {
        return target_speed;
    }

    // project vehicle's maximum speed
    const float vehicle_speed_max = calc_speed_max(cruise_speed, cruise_throttle);

    // return unadjusted target if already over vehicle's projected maximum speed
    if (target_speed >= vehicle_speed_max) {
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
// relies on these internal members being updated: lateral_acceleration, _yaw_error_cd, _distance_to_destination
float Mode::calc_reduced_speed_for_turn_or_distance(float desired_speed)
{
    // this method makes use the following internal variables
    const float yaw_error_cd = _yaw_error_cd;
    const float target_lateral_accel_G = lateral_acceleration;
    const float distance_to_waypoint = _distance_to_destination;

    // calculate the yaw_error_ratio which is the error (capped at 90degrees) expressed as a ratio (from 0 ~ 1)
    float yaw_error_ratio = constrain_float(fabsf(yaw_error_cd / 9000.0f), 0.0f, 1.0f);

    // apply speed_turn_gain parameter (expressed as a percentage) to yaw_error_ratio
    yaw_error_ratio *= (100 - g.speed_turn_gain) * 0.01f;

    // calculate absolute lateral acceleration expressed as a ratio (from 0 ~ 1) of the vehicle's maximum lateral acceleration
    const float lateral_accel_ratio = constrain_float(fabsf(target_lateral_accel_G / (g.turn_max_g * GRAVITY_MSS)), 0.0f, 1.0f);

    // calculate a lateral acceleration based speed scaling
    const float lateral_accel_speed_scaling = 1.0f - lateral_accel_ratio * yaw_error_ratio;

    // calculate a pivot steering based speed scaling (default to no reduction)
    float pivot_speed_scaling = 1.0f;
    if (rover.use_pivot_steering(yaw_error_cd)) {
        pivot_speed_scaling = 0.0f;
    }

    // scaled speed
    float speed_scaled = desired_speed * MIN(lateral_accel_speed_scaling, pivot_speed_scaling);

    // limit speed based on distance to waypoint and max acceleration/deceleration
    if (is_positive(distance_to_waypoint) && is_positive(attitude_control.get_accel_max())) {
        const float speed_max = safe_sqrt(2.0f * distance_to_waypoint * attitude_control.get_accel_max() + sq(_desired_speed_final));
        speed_scaled = constrain_float(speed_scaled, -speed_max, speed_max);
    }

    // return minimum speed
    return speed_scaled;
}

// calculate the lateral acceleration target to cause the vehicle to drive along the path from origin to destination
// this function update lateral_acceleration and _yaw_error_cd members
void Mode::calc_lateral_acceleration(const struct Location &origin, const struct Location &destination, bool reversed)
{
    // Calculate the required turn of the wheels
    // negative error = left turn
    // positive error = right turn
    rover.nav_controller->set_reverse(reversed);
    rover.nav_controller->update_waypoint(origin, destination);
    lateral_acceleration = rover.nav_controller->lateral_acceleration();
    if (reversed) {
        _yaw_error_cd = wrap_180_cd(rover.nav_controller->target_bearing_cd() - ahrs.yaw_sensor + 18000);
    } else {
        _yaw_error_cd = wrap_180_cd(rover.nav_controller->target_bearing_cd() - ahrs.yaw_sensor);
    }
    if (rover.use_pivot_steering(_yaw_error_cd)) {
        if (is_positive(_yaw_error_cd)) {
            lateral_acceleration = g.turn_max_g * GRAVITY_MSS;
        }
        if (is_negative(_yaw_error_cd)) {
            lateral_acceleration = -g.turn_max_g * GRAVITY_MSS;
        }
    }
}

/*
    calculate steering angle given lateral_acceleration
*/
void Mode::calc_nav_steer(bool reversed)
{
    // add obstacle avoidance response to lateral acceleration target
    if (!reversed) {
        lateral_acceleration += (rover.obstacle.turn_angle / 45.0f) * g.turn_max_g;
    }

    // constrain to max G force
    lateral_acceleration = constrain_float(lateral_acceleration, -g.turn_max_g * GRAVITY_MSS, g.turn_max_g * GRAVITY_MSS);

    // send final steering command to motor library
    float steering_out = attitude_control.get_steering_out_lat_accel(lateral_acceleration, g2.motors.have_skid_steering(), g2.motors.limit.steer_left, g2.motors.limit.steer_right);
    g2.motors.set_steering(steering_out * 4500.0f);
}
