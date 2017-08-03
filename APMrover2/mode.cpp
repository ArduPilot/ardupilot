#include "mode.h"
#include "Rover.h"

Mode::Mode() :
    ahrs(rover.ahrs),
    g(rover.g),
    g2(rover.g2),
    channel_steer(rover.channel_steer),
    channel_throttle(rover.channel_throttle),
    mission(rover.mission)
{ }

void Mode::exit()
{
    // call sub-classes exit
    _exit();

    lateral_acceleration = 0.0f;
    rover.g.pidSpeedThrottle.reset_I();
}

bool Mode::enter()
{
    g2.motors.slew_limit_throttle(false);
    return _enter();
}

// set desired location
void Mode::set_desired_location(const struct Location& destination)
{
    // record targets
    _origin = rover.current_loc;
    _destination = destination;
    _desired_speed = g.speed_cruise;

    // initialise distance
    _distance_to_destination = get_distance(rover.current_loc, _destination);
    _reached_destination = false;
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

void Mode::calc_throttle(float target_speed, bool reversed)
{
    // get ground speed from vehicle
    const float &groundspeed = rover.ground_speed;

    // calculate ground speed and ground speed error
    _speed_error = fabsf(target_speed) - groundspeed;

    const float throttle_base = (fabsf(target_speed) / g.speed_cruise) * g.throttle_cruise;
    const float throttle_target = throttle_base + calc_throttle_nudge();

    float throttle = throttle_target + (g.pidSpeedThrottle.get_pid(_speed_error * 100.0f) / 100.0f);

    if (reversed) {
        g2.motors.set_throttle(constrain_int16(-throttle, -g.throttle_max, -g.throttle_min));
    } else {
        g2.motors.set_throttle(constrain_int16(throttle, g.throttle_min, g.throttle_max));
    }

    if (!reversed && g.braking_percent != 0 && _speed_error < -g.braking_speederr) {
        // the user has asked to use reverse throttle to brake. Apply
        // it in proportion to the ground speed error, but only when
        // our ground speed error is more than BRAKING_SPEEDERR.
        //
        // We use a linear gain, with 0 gain at a ground speed error
        // of braking_speederr, and 100% gain when groundspeed_error
        // is 2*braking_speederr
        const float brake_gain = constrain_float(((-_speed_error)-g.braking_speederr)/g.braking_speederr, 0.0f, 1.0f);
        const int16_t braking_throttle = g.throttle_max * (g.braking_percent * 0.01f) * brake_gain;
        g2.motors.set_throttle(constrain_int16(-braking_throttle, -g.throttle_max, -g.throttle_min));
    }
}


// calculate pilot input to nudge throttle up or down
int16_t Mode::calc_throttle_nudge()
{
    // get pilot throttle input (-100 to +100)
    int16_t pilot_throttle = rover.channel_throttle->get_control_in();
    int16_t throttle_nudge = 0;

    // Check if the throttle value is above 50% and we need to nudge
    // Make sure its above 50% in the direction we are travelling
    if ((fabsf(pilot_throttle) > 50.0f) &&
        (((pilot_throttle < 0) && rover.in_reverse) ||
         ((pilot_throttle > 0) && !rover.in_reverse))) {
        throttle_nudge = (rover.g.throttle_max - rover.g.throttle_cruise) * ((fabsf(rover.channel_throttle->norm_input()) - 0.5f) / 0.5f);
    }

    return throttle_nudge;
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
    float lateral_accel_ratio = constrain_float(fabsf(target_lateral_accel_G / (g.turn_max_g * GRAVITY_MSS)), 0.0f, 1.0f);

    // calculate a lateral acceleration based speed scaling
    float lateral_accel_speed_scaling = 1.0f - lateral_accel_ratio * yaw_error_ratio;

    // calculate a pivot steering based speed scaling (default to no reduction)
    float pivot_speed_scaling = 1.0f;
    if (rover.use_pivot_steering(yaw_error_cd)) {
        pivot_speed_scaling = 0.0f;
    }

    // calculate a waypoint distance based scaling (default to no reduction)
    float distance_speed_scaling = 1.0f;
    if (is_positive(distance_to_waypoint)) {
        distance_speed_scaling = 1.0f - yaw_error_ratio;
    }

    // return minimum speed
    return desired_speed * MIN(MIN(lateral_accel_speed_scaling, distance_speed_scaling), pivot_speed_scaling);
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

    // set controller reversal
    rover.steerController.set_reverse(reversed);

    // send final steering command to motor library
    g2.motors.set_steering(rover.steerController.get_steering_out_lat_accel(lateral_acceleration));
}
