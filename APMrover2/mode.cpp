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
        rover.g2.sailboat.clear_tack();
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
    if ((rover.failsafe.bits & FAILSAFE_EVENT_THROTTLE) || (rover.channel_lateral == nullptr)) {
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
    return g2.wp_nav.wp_bearing_cd() * 0.01f;
}

// return short-term target heading in degrees (i.e. target heading back to line between waypoints)
float Mode::nav_bearing() const
{
    if (!is_autopilot_mode()) {
        return 0.0f;
    }
    return g2.wp_nav.nav_bearing_cd() * 0.01f;
}

// return cross track error (i.e. vehicle's distance from the line between waypoints)
float Mode::crosstrack_error() const
{
    if (!is_autopilot_mode()) {
        return 0.0f;
    }
    return g2.wp_nav.crosstrack_error();
}

// return desired lateral acceleration
float Mode::get_desired_lat_accel() const
{
    if (!is_autopilot_mode()) {
        return 0.0f;
    }
    return g2.wp_nav.get_lat_accel();
}

// set desired location
bool Mode::set_desired_location(const struct Location& destination, float next_leg_bearing_cd)
{
    if (!g2.wp_nav.set_desired_location(destination, next_leg_bearing_cd)) {
        return false;
    }

    // initialise distance
    _distance_to_destination = g2.wp_nav.get_distance_to_destination();
    _reached_destination = false;

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

// get default speed for this mode (held in WP_SPEED or RTL_SPEED)
float Mode::get_speed_default(bool rtl) const
{
    if (rtl && is_positive(g2.rtl_speed)) {
        return g2.rtl_speed;
    }

    return g2.wp_nav.get_default_speed();
}

// restore desired speed to default from parameter values (WP_SPEED)
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
    g2.wp_nav.set_reversed(value);
}

// handle tacking request (from auxiliary switch) in sailboats
void Mode::handle_tack_request()
{
    // autopilot modes handle tacking
    if (is_autopilot_mode()) {
        rover.g2.sailboat.handle_tack_request_auto();
    }
}

void Mode::calc_throttle(float target_speed, bool avoidance_enabled)
{
    // get acceleration limited target speed
    target_speed = attitude_control.get_desired_speed_accel_limited(target_speed, rover.G_Dt);

    // apply object avoidance to desired speed using half vehicle's maximum deceleration
    if (avoidance_enabled) {
        g2.avoid.adjust_speed(0.0f, 0.5f * attitude_control.get_decel_max(), ahrs.yaw, target_speed, rover.G_Dt);
    }

    // call throttle controller and convert output to -100 to +100 range
    float throttle_out = 0.0f;

    if (rover.g2.sailboat.sail_enabled()) {
        // sailboats use special throttle and mainsail controller
        float mainsail_out = 0.0f;
        rover.g2.sailboat.get_throttle_and_mainsail_out(target_speed, throttle_out, mainsail_out);
        rover.g2.motors.set_mainsail(mainsail_out);
    } else {
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
    }

    // send to motor
    g2.motors.set_throttle(throttle_out);
}

// performs a controlled stop with steering centered
bool Mode::stop_vehicle()
{
    // call throttle controller and convert output to -100 to +100 range
    bool stopped = false;
    float throttle_out;

    // if vehicle is balance bot, calculate throttle required for balancing
    if (rover.is_balancebot()) {
        throttle_out = 100.0f * attitude_control.get_throttle_out_speed(0, g2.motors.limit.throttle_lower, g2.motors.limit.throttle_upper, g.speed_cruise, g.throttle_cruise * 0.01f, rover.G_Dt);
        rover.balancebot_pitch_control(throttle_out);
    } else {
        throttle_out = 100.0f * attitude_control.get_throttle_out_stop(g2.motors.limit.throttle_lower, g2.motors.limit.throttle_upper, g.speed_cruise, g.throttle_cruise * 0.01f, rover.G_Dt, stopped);
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
    } else if (is_positive(g2.speed_max)) {
        speed_max = g2.speed_max;
    } else {
        // project vehicle's maximum speed
        speed_max = (1.0f / cruise_throttle) * cruise_speed;
    }

    // constrain to 30m/s (108km/h) and return
    return constrain_float(speed_max, 0.0f, 30.0f);
}

// calculate pilot input to nudge speed up or down
//  target_speed should be in meters/sec
//  reversed should be true if the vehicle is intentionally backing up which allows the pilot to increase the backing up speed by pulling the throttle stick down
float Mode::calc_speed_nudge(float target_speed, bool reversed)
{
    // return immediately during RC/GCS failsafe
    if (rover.failsafe.bits & FAILSAFE_EVENT_THROTTLE) {
        return target_speed;
    }

    // return immediately if pilot is not attempting to nudge speed
    // pilot can nudge up speed if throttle (in range -100 to +100) is above 50% of center in direction of travel
    const int16_t pilot_throttle = constrain_int16(rover.channel_throttle->get_control_in(), -100, 100);
    if (((pilot_throttle <= 50) && !reversed) ||
        ((pilot_throttle >= -50) && reversed)) {
        return target_speed;
    }

    // sanity checks
    if (g.throttle_cruise > 100 || g.throttle_cruise < 5) {
        return target_speed;
    }

    // project vehicle's maximum speed
    const float vehicle_speed_max = calc_speed_max(g.speed_cruise, g.throttle_cruise * 0.01f);

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

// high level call to navigate to waypoint
// uses wp_nav to calculate turn rate and speed to drive along the path from origin to destination
// this function updates _distance_to_destination
void Mode::navigate_to_waypoint()
{
    // update navigation controller
    g2.wp_nav.update(rover.G_Dt);
    _distance_to_destination = g2.wp_nav.get_distance_to_destination();

    // pass speed to throttle controller after applying nudge from pilot
    float desired_speed = g2.wp_nav.get_speed();
    desired_speed = calc_speed_nudge(desired_speed, g2.wp_nav.get_reversed());
    calc_throttle(desired_speed, true);

    float desired_heading_cd = g2.wp_nav.oa_wp_bearing_cd();
    if (g2.sailboat.use_indirect_route(desired_heading_cd)) {
        // sailboats use heading controller when tacking upwind
        desired_heading_cd = g2.sailboat.calc_heading(desired_heading_cd);
        // use pivot turn rate for tacks
        const float turn_rate = g2.sailboat.tacking() ? g2.wp_nav.get_pivot_rate() : 0.0f;
        calc_steering_to_heading(desired_heading_cd, turn_rate);
    } else {
        // call turn rate steering controller
        calc_steering_from_turn_rate(g2.wp_nav.get_turn_rate_rads(), desired_speed, g2.wp_nav.get_reversed());
    }
}

// calculate steering output given a turn rate and speed
void Mode::calc_steering_from_turn_rate(float turn_rate, float speed, bool reversed)
{
    // calculate and send final steering command to motor library
    const float steering_out = attitude_control.get_steering_out_rate(turn_rate,
                                                                      g2.motors.limit.steer_left,
                                                                      g2.motors.limit.steer_right,
                                                                      rover.G_Dt);
    g2.motors.set_steering(steering_out * 4500.0f);
}

/*
    calculate steering output given lateral_acceleration
*/
void Mode::calc_steering_from_lateral_acceleration(float lat_accel, bool reversed)
{
    // constrain to max G force
    lat_accel = constrain_float(lat_accel, -g.turn_max_g * GRAVITY_MSS, g.turn_max_g * GRAVITY_MSS);

    // send final steering command to motor library
    const float steering_out = attitude_control.get_steering_out_lat_accel(lat_accel,
                                                                           g2.motors.limit.steer_left,
                                                                           g2.motors.limit.steer_right,
                                                                           rover.G_Dt);
    set_steering(steering_out * 4500.0f);
}

// calculate steering output to drive towards desired heading
// rate_max is a maximum turn rate in deg/s.  set to zero to use default turn rate limits
void Mode::calc_steering_to_heading(float desired_heading_cd, float rate_max_degs)
{
    // call heading controller
    const float steering_out = attitude_control.get_steering_out_heading(radians(desired_heading_cd*0.01f),
                                                                         radians(rate_max_degs),
                                                                         g2.motors.limit.steer_left,
                                                                         g2.motors.limit.steer_right,
                                                                         rover.G_Dt);
    set_steering(steering_out * 4500.0f);
}

void Mode::set_steering(float steering_value)
{
    if (allows_stick_mixing() && g2.stick_mixing > 0) {
        steering_value = channel_steer->stick_mixing((int16_t)steering_value);
    }
    steering_value = constrain_float(steering_value, -4500.0f, 4500.0f);
    g2.motors.set_steering(steering_value);
}

Mode *Rover::mode_from_mode_num(const enum Mode::Number num)
{
    Mode *ret = nullptr;
    switch (num) {
    case Mode::Number::MANUAL:
        ret = &mode_manual;
        break;
    case Mode::Number::ACRO:
        ret = &mode_acro;
        break;
    case Mode::Number::STEERING:
        ret = &mode_steering;
        break;
    case Mode::Number::HOLD:
        ret = &mode_hold;
        break;
    case Mode::Number::LOITER:
        ret = &mode_loiter;
        break;
    case Mode::Number::FOLLOW:
        ret = &mode_follow;
        break;
    case Mode::Number::SIMPLE:
        ret = &mode_simple;
        break;
    case Mode::Number::AUTO:
        ret = &mode_auto;
        break;
    case Mode::Number::RTL:
        ret = &mode_rtl;
        break;
    case Mode::Number::SMART_RTL:
        ret = &mode_smartrtl;
        break;
    case Mode::Number::GUIDED:
       ret = &mode_guided;
        break;
    case Mode::Number::INITIALISING:
        ret = &mode_initializing;
        break;
    default:
        break;
    }
    return ret;
}
