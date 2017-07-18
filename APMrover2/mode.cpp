#include "mode.h"
#include "Rover.h"

Mode::Mode() :
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
    rover.throttle = 500;
    rover.g.pidSpeedThrottle.reset_I();
    if (!rover.in_auto_reverse) {
        rover.set_reverse(false);
    }
    rover.rtl_complete = false;
}

bool Mode::enter()
{
    g2.motors.slew_limit_throttle(false);
    return _enter();
}

void Mode::calc_throttle(float target_speed)
{
    int16_t &throttle = rover.throttle;
    const int32_t next_navigation_leg_cd = rover.next_navigation_leg_cd;
    const AP_AHRS &ahrs = rover.ahrs;
    const float wp_distance = rover.wp_distance;
    float &groundspeed_error = rover.groundspeed_error;
    const float ground_speed = rover.ground_speed;

    const float throttle_base = (fabsf(target_speed) / g.speed_cruise) * g.throttle_cruise;
    const int throttle_target = throttle_base + calc_throttle_nudge();

    /*
        reduce target speed in proportion to turning rate, up to the
        SPEED_TURN_GAIN percentage.
    */
    float steer_rate = fabsf(lateral_acceleration / (g.turn_max_g * GRAVITY_MSS));
    steer_rate = constrain_float(steer_rate, 0.0f, 1.0f);

    // use g.speed_turn_gain for a 90 degree turn, and in proportion
    // for other turn angles
    const int32_t turn_angle = wrap_180_cd(next_navigation_leg_cd - ahrs.yaw_sensor);
    const float speed_turn_ratio = constrain_float(fabsf(turn_angle / 9000.0f), 0.0f, 1.0f);
    const float speed_turn_reduction = (100 - g.speed_turn_gain) * speed_turn_ratio * 0.01f;

    float reduction = 1.0f - steer_rate * speed_turn_reduction;

    if (is_autopilot_mode() && rover.mode_guided.guided_mode != ModeGuided::Guided_Velocity && wp_distance <= g.speed_turn_dist) {
        // in auto-modes we reduce speed when approaching waypoints
        const float reduction2 = 1.0f - speed_turn_reduction;
        if (reduction2 < reduction) {
            reduction = reduction2;
        }
    }

    // reduce the target speed by the reduction factor
    target_speed *= reduction;

    groundspeed_error = fabsf(target_speed) - ground_speed;

    throttle = throttle_target + (g.pidSpeedThrottle.get_pid(groundspeed_error * 100.0f) / 100.0f);

    // also reduce the throttle by the reduction factor. This gives a
    // much faster response in turns
    throttle *= reduction;

    if (rover.in_reverse) {
        g2.motors.set_throttle(constrain_int16(-throttle, -g.throttle_max, -g.throttle_min));
    } else {
        g2.motors.set_throttle(constrain_int16(throttle, g.throttle_min, g.throttle_max));
    }

    if (!rover.in_reverse && g.braking_percent != 0 && groundspeed_error < -g.braking_speederr) {
        // the user has asked to use reverse throttle to brake. Apply
        // it in proportion to the ground speed error, but only when
        // our ground speed error is more than BRAKING_SPEEDERR.
        //
        // We use a linear gain, with 0 gain at a ground speed error
        // of braking_speederr, and 100% gain when groundspeed_error
        // is 2*braking_speederr
        const float brake_gain = constrain_float(((-groundspeed_error)-g.braking_speederr)/g.braking_speederr, 0.0f, 1.0f);
        const int16_t braking_throttle = g.throttle_max * (g.braking_percent * 0.01f) * brake_gain;
        g2.motors.set_throttle(constrain_int16(-braking_throttle, -g.throttle_max, -g.throttle_min));

        // temporarily set us in reverse to allow the PWM setting to
        // go negative
        rover.set_reverse(true);
    }

    if (rover.mode_guided.guided_mode != ModeGuided::Guided_Velocity) {
        if (rover.use_pivot_steering()) {
            // In Guided Velocity, only the steering input is used to calculate the pivot turn.
            g2.motors.set_throttle(0.0f);
        }
    }
}

void Mode::calc_lateral_acceleration()
{
    calc_lateral_acceleration(rover.current_loc, rover.next_WP);
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

/*
 * Calculate desired turn angles (in medium freq loop)
 */
void Mode::calc_lateral_acceleration(const struct Location &last_WP, const struct Location &next_WP)
{
    // Calculate the required turn of the wheels
    // negative error = left turn
    // positive error = right turn
    rover.nav_controller->update_waypoint(last_WP, next_WP);
    lateral_acceleration = rover.nav_controller->lateral_acceleration();
    if (rover.use_pivot_steering()) {
        const int16_t bearing_error = wrap_180_cd(rover.nav_controller->target_bearing_cd() - rover.ahrs.yaw_sensor) / 100;
        if (bearing_error > 0) {
            lateral_acceleration = g.turn_max_g * GRAVITY_MSS;
        } else {
            lateral_acceleration = -g.turn_max_g * GRAVITY_MSS;
        }
    }
}

/*
    calculate steering angle given lateral_acceleration
*/
void Mode::calc_nav_steer()
{
    // add in obstacle avoidance
    if (!rover.in_reverse) {
        lateral_acceleration += (rover.obstacle.turn_angle / 45.0f) * g.turn_max_g;
    }

    // constrain to max G force
    lateral_acceleration = constrain_float(lateral_acceleration, -g.turn_max_g * GRAVITY_MSS, g.turn_max_g * GRAVITY_MSS);

    // send final steering command to motor library
    g2.motors.set_steering(rover.steerController.get_steering_out_lat_accel(lateral_acceleration));
}
