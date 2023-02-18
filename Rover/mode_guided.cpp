#include "Rover.h"

bool ModeGuided::_enter()
{
    // initialise submode to stop or loiter
    if (rover.is_boat()) {
        if (!start_loiter()) {
            start_stop();
        }
    } else {
        start_stop();
    }

    // initialise waypoint navigation library
    g2.wp_nav.init();

    send_notification = false;

    return true;
}

void ModeGuided::update()
{
    switch (_guided_mode) {
        case Guided_WP:
        {
            // check if we've reached the destination
            if (!g2.wp_nav.reached_destination()) {
                // update navigation controller
                navigate_to_waypoint();
            } else {
                // send notification
                if (send_notification) {
                    send_notification = false;
                    rover.gcs().send_mission_item_reached_message(0);
                }

                // we have reached the destination so stay here
                if (rover.is_boat()) {
                    if (!start_loiter()) {
                        stop_vehicle();
                    }
                } else {
                    stop_vehicle();
                }
                // update distance to destination
                _distance_to_destination = rover.current_loc.get_distance(g2.wp_nav.get_destination());
            }
            break;
        }

        case Guided_HeadingAndSpeed:
        {
            // stop vehicle if target not updated within 3 seconds
            if (have_attitude_target && (millis() - _des_att_time_ms) > 3000) {
                gcs().send_text(MAV_SEVERITY_WARNING, "target not received last 3secs, stopping");
                have_attitude_target = false;
            }
            if (have_attitude_target) {
                // run steering and throttle controllers
                calc_steering_to_heading(_desired_yaw_cd);
                calc_throttle(calc_speed_nudge(_desired_speed, is_negative(_desired_speed)), true);
            } else {
                // we have reached the destination so stay here
                if (rover.is_boat()) {
                    if (!start_loiter()) {
                        stop_vehicle();
                    }
                } else {
                    stop_vehicle();
                }
            }
            break;
        }

        case Guided_TurnRateAndSpeed:
        {
            // stop vehicle if target not updated within 3 seconds
            if (have_attitude_target && (millis() - _des_att_time_ms) > 3000) {
                gcs().send_text(MAV_SEVERITY_WARNING, "target not received last 3secs, stopping");
                have_attitude_target = false;
            }
            if (have_attitude_target) {
                // run steering and throttle controllers
                float steering_out = attitude_control.get_steering_out_rate(radians(_desired_yaw_rate_cds / 100.0f),
                                                                            g2.motors.limit.steer_left,
                                                                            g2.motors.limit.steer_right,
                                                                            rover.G_Dt);
                set_steering(steering_out * 4500.0f);
                calc_throttle(calc_speed_nudge(_desired_speed, is_negative(_desired_speed)), true);
            } else {
                // we have reached the destination so stay here
                if (rover.is_boat()) {
                    if (!start_loiter()) {
                        stop_vehicle();
                    }
                } else {
                    stop_vehicle();
                }
            }
            break;
        }

        case Guided_Loiter:
        {
            rover.mode_loiter.update();
            break;
        }

        case Guided_SteeringAndThrottle:
        {
            // handle timeout
            if (_have_strthr && (AP_HAL::millis() - _strthr_time_ms) > 3000) {
                _have_strthr = false;
                gcs().send_text(MAV_SEVERITY_WARNING, "target not received last 3secs, stopping");
            }
            if (_have_strthr) {
                // pass latest steering and throttle directly to motors library
                g2.motors.set_steering(_strthr_steering * 4500.0f, false);
                g2.motors.set_throttle(_strthr_throttle * 100.0f);
            } else {
                // loiter or stop vehicle
                if (rover.is_boat()) {
                    if (!start_loiter()) {
                        stop_vehicle();
                    }
                } else {
                    stop_vehicle();
                }
            }
            break;
        }

        case Guided_Stop:
            stop_vehicle();
            break;

        default:
            gcs().send_text(MAV_SEVERITY_WARNING, "Unknown GUIDED mode");
            break;
    }
}

// return heading (in degrees) and cross track error (in meters) for reporting to ground station (NAV_CONTROLLER_OUTPUT message)
float ModeGuided::wp_bearing() const
{
    switch (_guided_mode) {
    case Guided_WP:
        return g2.wp_nav.wp_bearing_cd() * 0.01f;
    case Guided_HeadingAndSpeed:
    case Guided_TurnRateAndSpeed:
        return 0.0f;
    case Guided_Loiter:
        return rover.mode_loiter.wp_bearing();
    case Guided_SteeringAndThrottle:
    case Guided_Stop:
        return 0.0f;
    }

    // we should never reach here but just in case, return 0
    return 0.0f;
}

float ModeGuided::nav_bearing() const
{
    switch (_guided_mode) {
    case Guided_WP:
        return g2.wp_nav.nav_bearing_cd() * 0.01f;
    case Guided_HeadingAndSpeed:
    case Guided_TurnRateAndSpeed:
        return 0.0f;
    case Guided_Loiter:
        return rover.mode_loiter.nav_bearing();
    case Guided_SteeringAndThrottle:
    case Guided_Stop:
        return 0.0f;
    }

    // we should never reach here but just in case, return 0
    return 0.0f;
}

float ModeGuided::crosstrack_error() const
{
    switch (_guided_mode) {
    case Guided_WP:
        return g2.wp_nav.crosstrack_error();
    case Guided_HeadingAndSpeed:
    case Guided_TurnRateAndSpeed:
        return 0.0f;
    case Guided_Loiter:
        return rover.mode_loiter.crosstrack_error();
    case Guided_SteeringAndThrottle:
    case Guided_Stop:
        return 0.0f;
    }

    // we should never reach here but just in case, return 0
    return 0.0f;
}

float ModeGuided::get_desired_lat_accel() const
{
    switch (_guided_mode) {
    case Guided_WP:
        return g2.wp_nav.get_lat_accel();
    case Guided_HeadingAndSpeed:
    case Guided_TurnRateAndSpeed:
        return 0.0f;
    case Guided_Loiter:
        return rover.mode_loiter.get_desired_lat_accel();
    case Guided_SteeringAndThrottle:
    case Guided_Stop:
        return 0.0f;
    }

    // we should never reach here but just in case, return 0
    return 0.0f;
}

// return distance (in meters) to destination
float ModeGuided::get_distance_to_destination() const
{
    switch (_guided_mode) {
    case Guided_WP:
        return _distance_to_destination;
    case Guided_HeadingAndSpeed:
    case Guided_TurnRateAndSpeed:
        return 0.0f;
    case Guided_Loiter:
        return rover.mode_loiter.get_distance_to_destination();
    case Guided_SteeringAndThrottle:
    case Guided_Stop:
        return 0.0f;
    }

    // we should never reach here but just in case, return 0
    return 0.0f;
}

// return true if vehicle has reached or even passed destination
bool ModeGuided::reached_destination() const
{
    switch (_guided_mode) {
    case Guided_WP:
        return g2.wp_nav.reached_destination();
    case Guided_HeadingAndSpeed:
    case Guided_TurnRateAndSpeed:
    case Guided_Loiter:
    case Guided_SteeringAndThrottle:
    case Guided_Stop:
        return true;
    }

    // we should never reach here but just in case, return true is the safer option
    return true;
}

// set desired speed in m/s
bool ModeGuided::set_desired_speed(float speed)
{
    switch (_guided_mode) {
    case Guided_WP:
        return g2.wp_nav.set_speed_max(speed);
    case Guided_HeadingAndSpeed:
    case Guided_TurnRateAndSpeed:
        // speed is set from mavlink message
        return false;
    case Guided_Loiter:
        return rover.mode_loiter.set_desired_speed(speed);
    case Guided_SteeringAndThrottle:
    case Guided_Stop:
        // no speed control
        return false;
    }
    return false;
}

// get desired location
bool ModeGuided::get_desired_location(Location& destination) const
{
    switch (_guided_mode) {
    case Guided_WP:
        if (g2.wp_nav.is_destination_valid()) {
            destination = g2.wp_nav.get_oa_destination();
            return true;
        }
        return false;
    case Guided_HeadingAndSpeed:
    case Guided_TurnRateAndSpeed:
        // not supported in these submodes
        return false;
    case Guided_Loiter:
        // get destination from loiter
        return rover.mode_loiter.get_desired_location(destination);
    case Guided_SteeringAndThrottle:
    case Guided_Stop:
        // no desired location in this submode
        break;
    }

    // should never get here but just in case
    return false;
}

// set desired location
bool ModeGuided::set_desired_location(const Location &destination, Location next_destination)
{
    if (use_scurves_for_navigation()) {
        // use scurves for navigation
        if (!g2.wp_nav.set_desired_location(destination, next_destination)) {
            return false;
        }
    } else {
        // use position controller input shaping for navigation
        // this does not support object avoidance but does allow faster updates of the target
        if (!g2.wp_nav.set_desired_location_expect_fast_update(destination)) {
            return false;
        }
    }

    // handle guided specific initialisation and logging
    _guided_mode = ModeGuided::Guided_WP;
    send_notification = true;
    rover.Log_Write_GuidedTarget(_guided_mode, Vector3f(destination.lat, destination.lng, 0), Vector3f(g2.wp_nav.get_speed_max(), 0.0f, 0.0f));
    return true;
}

// set desired attitude
void ModeGuided::set_desired_heading_and_speed(float yaw_angle_cd, float target_speed)
{
    // initialisation and logging
    _guided_mode = ModeGuided::Guided_HeadingAndSpeed;
    _des_att_time_ms = AP_HAL::millis();

    // record targets
    _desired_yaw_cd = yaw_angle_cd;
    _desired_speed = target_speed;
    have_attitude_target = true;

    // log new target
    rover.Log_Write_GuidedTarget(_guided_mode, Vector3f(_desired_yaw_cd, 0.0f, 0.0f), Vector3f(_desired_speed, 0.0f, 0.0f));
}

void ModeGuided::set_desired_heading_delta_and_speed(float yaw_delta_cd, float target_speed)
{
    // handle initialisation
    if (_guided_mode != ModeGuided::Guided_HeadingAndSpeed) {
        _guided_mode = ModeGuided::Guided_HeadingAndSpeed;
        _desired_yaw_cd = ahrs.yaw_sensor;
    }
    set_desired_heading_and_speed(wrap_180_cd(_desired_yaw_cd + yaw_delta_cd), target_speed);
}

// set desired velocity
void ModeGuided::set_desired_turn_rate_and_speed(float turn_rate_cds, float target_speed)
{
    // handle initialisation
    _guided_mode = ModeGuided::Guided_TurnRateAndSpeed;
    _des_att_time_ms = AP_HAL::millis();

    // record targets
    _desired_yaw_rate_cds = turn_rate_cds;
    _desired_speed = target_speed;
    have_attitude_target = true;

    // log new target
    rover.Log_Write_GuidedTarget(_guided_mode, Vector3f(_desired_yaw_rate_cds, 0.0f, 0.0f), Vector3f(_desired_speed, 0.0f, 0.0f));
}

// set steering and throttle (both in the range -1 to +1)
void ModeGuided::set_steering_and_throttle(float steering, float throttle)
{
    _guided_mode = ModeGuided::Guided_SteeringAndThrottle;
    _strthr_time_ms = AP_HAL::millis();
    _strthr_steering = constrain_float(steering, -1.0f, 1.0f);
    _strthr_throttle = constrain_float(throttle, -1.0f, 1.0f);
    _have_strthr = true;
}

bool ModeGuided::start_loiter()
{
    if (rover.mode_loiter.enter()) {
        _guided_mode = Guided_Loiter;
        return true;
    }
    return false;
}


// start stopping vehicle as quickly as possible
void ModeGuided::start_stop()
{
    _guided_mode = Guided_Stop;
}

// set guided timeout and movement limits
void ModeGuided::limit_set(uint32_t timeout_ms, float horiz_max)
{
    limit.timeout_ms = timeout_ms;
    limit.horiz_max = horiz_max;
}

// clear/turn off guided limits
void ModeGuided::limit_clear()
{
    limit.timeout_ms = 0;
    limit.horiz_max = 0.0f;
}

// initialise guided start time and location as reference for limit checking
//  only called from AUTO mode's start_guided method
void ModeGuided::limit_init_time_and_location()
{
    limit.start_time_ms = AP_HAL::millis();
    limit.start_loc = rover.current_loc;
}

// returns true if guided mode has breached a limit
bool ModeGuided::limit_breached() const
{
    // check if we have passed the timeout
    if ((limit.timeout_ms > 0) && (millis() - limit.start_time_ms >= limit.timeout_ms)) {
        return true;
    }

    // check if we have gone beyond horizontal limit
    if (is_positive(limit.horiz_max)) {
        return (rover.current_loc.get_distance(limit.start_loc) > limit.horiz_max);
    }

    // if we got this far we must be within limits
    return false;
}

// returns true if GUID_OPTIONS bit set to use scurve navigation instead of position controller input shaping
// scurves provide path planning and object avoidance but cannot handle fast updates to the destination (for fast updates use position controller input shaping)
bool ModeGuided::use_scurves_for_navigation() const
{
    return ((rover.g2.guided_options.get() & uint32_t(Options::SCurvesUsedForNavigation)) != 0);
}
