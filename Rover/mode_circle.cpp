#include "Rover.h"

#define AR_CIRCLE_ACCEL_DEFAULT         1.0 // default acceleration in m/s/s if not specified by user
#define AR_CIRCLE_RADIUS_MIN            0.5 // minimum radius in meters
#define AR_CIRCLE_REACHED_EDGE_DIST     1.0 // vehicle has reached edge if within 1m

const AP_Param::GroupInfo ModeCircle::var_info[] = {

    // @Param: _RADIUS
    // @DisplayName: Circle Radius
    // @Description: Vehicle will circle the center at this distance
    // @Units: m
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_RADIUS", 1, ModeCircle, radius, 20),

    // @Param: _SPEED
    // @DisplayName: Circle Speed
    // @Description: Vehicle will move at this speed around the circle.  If set to zero WP_SPEED will be used
    // @Units: m/s
    // @Range: 0 10
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("_SPEED", 2, ModeCircle, speed, 0),

    // @Param: _DIR
    // @DisplayName: Circle Direction
    // @Description: Circle Direction
    // @Values: 0:Clockwise, 1:Counter-Clockwise
    // @User: Standard
    AP_GROUPINFO("_DIR", 3, ModeCircle, direction, 0),

    AP_GROUPEND
};

ModeCircle::ModeCircle() : Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

// initialise with specific center location, radius (in meters) and direction
// replaces use of _enter when initialised from within Auto mode
bool ModeCircle::set_center(const Location& center_loc, float radius_m, bool dir_ccw)
{
    Vector2f center_pos_cm;
    if (!center_loc.get_vector_xy_from_origin_NE(center_pos_cm)) {
        return false;
    }
    if (!_enter()) {
        return false;
    }

    // convert center position from cm to m
    config.center_pos = center_pos_cm * 0.01;

    // set radius
    config.radius = MAX(fabsf(radius_m), AR_CIRCLE_RADIUS_MIN);

    // set direction
    config.dir = dir_ccw ? Direction::CCW : Direction::CW;

    // set target yaw rad (target point on circle)
    init_target_yaw_rad();

    // record center as location (only used for reporting)
    config.center_loc = center_loc;

    return true;
}

// initialize dock mode
bool ModeCircle::_enter()
{
    // capture starting point and yaw
    if (!AP::ahrs().get_relative_position_NE_origin(config.center_pos)) {
        return false;
    }
    config.radius = MAX(fabsf(radius), AR_CIRCLE_RADIUS_MIN);
    config.dir = (direction == 1) ? Direction::CCW : Direction::CW;
    config.speed = is_positive(speed) ? speed : g2.wp_nav.get_default_speed();
    target.yaw_rad = AP::ahrs().get_yaw();
    target.speed = 0;

    // calculate speed, accel and jerk limits
    // otherwise the vehicle uses wp_nav default speed limit
    float atc_accel_max = MIN(g2.attitude_control.get_accel_max(), g2.attitude_control.get_decel_max());
    if (!is_positive(atc_accel_max)) {
        atc_accel_max = AR_CIRCLE_ACCEL_DEFAULT;
    }
    const float accel_max = is_positive(g2.wp_nav.get_default_accel()) ? MIN(g2.wp_nav.get_default_accel(), atc_accel_max) : atc_accel_max;
    const float jerk_max = is_positive(g2.wp_nav.get_default_jerk()) ? g2.wp_nav.get_default_jerk() : accel_max;

    // initialise position controller
    g2.pos_control.set_limits(config.speed, accel_max, g2.attitude_control.get_turn_lat_accel_max(), jerk_max);
    g2.pos_control.init();

    // initialise angles covered and reached_edge state
    angle_total_rad = 0;
    reached_edge = false;
    dist_to_edge_m = 0;

    return true;
}

// initialise target_yaw_rad using the vehicle's position and yaw
// if there is no current position estimate target_yaw_rad is set to 0
void ModeCircle::init_target_yaw_rad()
{
    // if no position estimate use vehicle yaw
    Vector2f curr_pos_NE;
    if (!AP::ahrs().get_relative_position_NE_origin(curr_pos_NE)) {
        target.yaw_rad = AP::ahrs().yaw;
        return;
    }

    // calc vector from circle center to vehicle
    Vector2f center_to_veh = (curr_pos_NE - config.center_pos);
    float dist_m = center_to_veh.length();

    // if current position is exactly at the center of the circle return vehicle yaw
    if (is_zero(dist_m)) {
        target.yaw_rad = AP::ahrs().yaw;
    } else {
        target.yaw_rad = center_to_veh.angle();
    }
}

void ModeCircle::update()
{
    // get current position
    Vector2f curr_pos;
    const bool position_ok = AP::ahrs().get_relative_position_NE_origin(curr_pos);

    // if no position estimate stop vehicle
    if (!position_ok) {
        stop_vehicle();
        return;
    }

    // check if vehicle has reached edge of circle
    const Vector2f center_to_veh = curr_pos - config.center_pos;
    _distance_to_destination = center_to_veh.length();
    dist_to_edge_m = fabsf(_distance_to_destination - config.radius);
    if (!reached_edge) {
        const float dist_thresh_m = MAX(rover.g2.turn_radius, AR_CIRCLE_REACHED_EDGE_DIST);
        reached_edge = dist_to_edge_m <= dist_thresh_m;
    }

    // accelerate speed up to desired speed
    const float speed_max = reached_edge ? config.speed : 0.0;
    const float speed_change_max = (g2.pos_control.get_accel_max() * rover.G_Dt);
    target.speed = constrain_float(speed_max, target.speed - speed_change_max, target.speed + speed_change_max);

    // calculate angular rate and update target angle
    const float circumference = 2.0 * M_PI * config.radius;
    const float angular_rate_rad = (target.speed / circumference) * M_2PI * (config.dir == Direction::CW ? 1.0 : -1.0);
    const float angle_dt = angular_rate_rad * rover.G_Dt;
    target.yaw_rad = wrap_PI(target.yaw_rad + angle_dt);
    angle_total_rad += angle_dt;

    // calculate target point's position, velocity and acceleration
    target.pos = config.center_pos.topostype();
    target.pos.offset_bearing(degrees(target.yaw_rad), config.radius);

    // velocity is perpendicular to angle from the circle's center to the target point on the edge of the circle
    target.vel = Vector2f(target.speed, 0);
    target.vel.rotate(target.yaw_rad + radians(90));

    // acceleration is towards center of circle and is speed^2 / radius
    target.accel = Vector2f(sq(target.speed) / config.radius, 0);
    target.accel.rotate(target.yaw_rad + radians(180));

    g2.pos_control.input_pos_vel_accel_target(target.pos, target.vel, target.accel, rover.G_Dt);
    g2.pos_control.update(rover.G_Dt);

    // get desired speed and turn rate from pos_control
    const float desired_speed = g2.pos_control.get_desired_speed();
    const float desired_turn_rate = g2.pos_control.get_desired_turn_rate_rads();

    // run steering and throttle controllers
    calc_steering_from_turn_rate(desired_turn_rate);
    calc_throttle(desired_speed, true);
}

// return desired heading (in degrees) and cross track error (in meters) for reporting to ground station (NAV_CONTROLLER_OUTPUT message)
float ModeCircle::wp_bearing() const
{
    Vector2f curr_pos_NE;
    if (!AP::ahrs().get_relative_position_NE_origin(curr_pos_NE)) {
        return 0;
    }
    // calc vector from circle center to vehicle
    Vector2f veh_to_center = (config.center_pos - curr_pos_NE);
    if (veh_to_center.is_zero()) {
        return 0;
    }
    return degrees(veh_to_center.angle());
}

float ModeCircle::nav_bearing() const
{
    // get position error as a vector from the current position to the target position
    const Vector2p pos_error = g2.pos_control.get_pos_error();
    if (pos_error.is_zero()) {
        return 0;
    }
    return degrees(pos_error.angle());
}

float ModeCircle::get_desired_lat_accel() const
{
    return g2.pos_control.get_desired_lat_accel();
}

// set desired speed in m/s
bool ModeCircle::set_desired_speed(float speed_ms)
{
    if (is_positive(speed_ms)) {
        config.speed = speed_ms;

        // update position controller limits if required
        if (config.speed > g2.pos_control.get_speed_max()) {
            g2.pos_control.set_limits(config.speed, g2.pos_control.get_accel_max(), g2.pos_control.get_lat_accel_max(), g2.pos_control.get_jerk_max());
        }
        return true;
    }
 
    return false;
}

// return desired location
bool ModeCircle::get_desired_location(Location& destination) const
{
    destination = config.center_loc;
    return true;
}
