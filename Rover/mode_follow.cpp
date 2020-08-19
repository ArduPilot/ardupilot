#include "mode.h"
#include "Rover.h"

// initialize follow mode
bool ModeFollow::_enter()
{
    if (!g2.follow.enabled()) {
        return false;
    }

    // initialise speed to waypoint speed
    _desired_speed = g2.wp_nav.get_default_speed();

    return true;
}

// exit handling
void ModeFollow::_exit()
{
    g2.follow.clear_offsets_if_required();
}

void ModeFollow::update()
{
    // stop vehicle if no speed estimate
    float speed;
    if (!attitude_control.get_forward_speed(speed)) {
        // no valid speed so stop
        g2.motors.set_throttle(0.0f);
        g2.motors.set_steering(0.0f);
        return;
    }

    Vector3f dist_vec;  // vector to lead vehicle
    Vector3f dist_vec_offs; // vector to lead vehicle + offset
    Vector3f vel_of_target; // velocity of lead vehicle

    // if no target simply stop the vehicle
    if (!g2.follow.get_target_dist_and_vel_ned(dist_vec, dist_vec_offs, vel_of_target)) {
        _reached_destination = true;
        stop_vehicle();
        return;
    }

    // calculate desired velocity vector
    Vector2f desired_velocity_ne;
    const float kp = g2.follow.get_pos_p().kP();
    desired_velocity_ne.x = vel_of_target.x + (dist_vec_offs.x * kp);
    desired_velocity_ne.y = vel_of_target.y + (dist_vec_offs.y * kp);

    // if desired velocity is zero stop vehicle
    if (is_zero(desired_velocity_ne.x) && is_zero(desired_velocity_ne.y)) {
        _reached_destination = true;
        stop_vehicle();
        return;
    }

    // we have not reached the target
    _reached_destination = false;

    // scale desired velocity to stay within horizontal speed limit
    float desired_speed = safe_sqrt(sq(desired_velocity_ne.x) + sq(desired_velocity_ne.y));
    if (!is_zero(desired_speed) && (desired_speed > _desired_speed)) {
        const float scalar_xy = _desired_speed / desired_speed;
        desired_velocity_ne *= scalar_xy;
        desired_speed = _desired_speed;
    }

    // calculate vehicle heading
    const float desired_yaw_cd = wrap_180_cd(atan2f(desired_velocity_ne.y, desired_velocity_ne.x) * DEGX100);

    // run steering and throttle controllers
    calc_steering_to_heading(desired_yaw_cd);
    calc_throttle(desired_speed, true);
}

// return desired heading (in degrees) for reporting to ground station (NAV_CONTROLLER_OUTPUT message)
float ModeFollow::wp_bearing() const
{
    return g2.follow.get_bearing_to_target();
}

// return distance (in meters) to destination
float ModeFollow::get_distance_to_destination() const
{
    return g2.follow.get_distance_to_target();
}

// set desired speed in m/s
bool ModeFollow::set_desired_speed(float speed)
{
    if (is_negative(speed)) {
        return false;
    }
    _desired_speed = speed;
    return true;
}
