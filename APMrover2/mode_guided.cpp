#include "mode.h"
#include "Rover.h"

bool ModeGuided::_enter()
{
    // initialise waypoint speed
    set_desired_speed_to_default();

    // set desired location to reasonable stopping point
    calc_stopping_location(_destination);
    set_desired_location(_destination);

    return true;
}

void ModeGuided::update()
{
    switch (_guided_mode) {
        case Guided_WP:
        {
            _distance_to_destination = rover.current_loc.get_distance(_destination);
            const bool near_wp = _distance_to_destination <= rover.g.waypoint_radius;
            // check if we've reached the destination
            if (!_reached_destination && (near_wp || location_passed_point(rover.current_loc, _origin, _destination))) {
                _reached_destination = true;
                rover.gcs().send_mission_item_reached_message(0);
            }
            // determine if we should keep navigating
            if (!_reached_destination) {
                // drive towards destination
                calc_steering_to_waypoint(_reached_destination ? rover.current_loc : _origin, _destination, _reversed);
                calc_throttle(calc_reduced_speed_for_turn_or_distance(_reversed ? -_desired_speed : _desired_speed), true, true);
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
                calc_throttle(_desired_speed, true, true);
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
                g2.motors.set_steering(steering_out * 4500.0f);
                calc_throttle(_desired_speed, true, true);
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

        default:
            gcs().send_text(MAV_SEVERITY_WARNING, "Unknown GUIDED mode");
            break;
    }
}

// return distance (in meters) to destination
float ModeGuided::get_distance_to_destination() const
{
    if (_guided_mode != Guided_WP || _reached_destination) {
        return 0.0f;
    }
    return _distance_to_destination;
}

// return true if vehicle has reached or even passed destination
bool ModeGuided::reached_destination()
{
    switch (_guided_mode) {
    case Guided_WP:
        return _reached_destination;
        break;
    case Guided_HeadingAndSpeed:
    case Guided_TurnRateAndSpeed:
    case Guided_Loiter:
        return true;
        break;
    }

    // we should never reach here but just in case, return true is the safer option
    return true;
}

// set desired location
void ModeGuided::set_desired_location(const struct Location& destination,
                                      float next_leg_bearing_cd)
{
    // call parent
    Mode::set_desired_location(destination, next_leg_bearing_cd);

    // handle guided specific initialisation and logging
    _guided_mode = ModeGuided::Guided_WP;
    rover.Log_Write_GuidedTarget(_guided_mode, Vector3f(_destination.lat, _destination.lng, 0), Vector3f(_desired_speed, 0.0f, 0.0f));
}

// set desired attitude
void ModeGuided::set_desired_heading_and_speed(float yaw_angle_cd, float target_speed)
{
    // call parent
    Mode::set_desired_heading_and_speed(yaw_angle_cd, target_speed);

    // handle guided specific initialisation and logging
    _guided_mode = ModeGuided::Guided_HeadingAndSpeed;
    _des_att_time_ms = AP_HAL::millis();
    _reached_destination = false;

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
    _reached_destination = false;

    // record targets
    _desired_yaw_rate_cds = turn_rate_cds;
    _desired_speed = target_speed;
    have_attitude_target = true;

    // log new target
    rover.Log_Write_GuidedTarget(_guided_mode, Vector3f(_desired_yaw_rate_cds, 0.0f, 0.0f), Vector3f(_desired_speed, 0.0f, 0.0f));
}

bool ModeGuided::start_loiter()
{
    if (rover.mode_loiter.enter()) {
        _guided_mode = Guided_Loiter;
        return true;
    }
    return false;
}
