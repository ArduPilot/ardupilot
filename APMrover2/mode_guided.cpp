#include "mode.h"
#include "Rover.h"

bool ModeGuided::_enter()
{
    /*
      when entering guided mode we set the target as the current
      location. This matches the behaviour of the copter code.
    */
    lateral_acceleration = 0.0f;
    set_desired_location(rover.current_loc);

    // guided mode never travels in reverse
    rover.set_reverse(false);

    g2.motors.slew_limit_throttle(true);
    return true;
}

void ModeGuided::update()
{
    switch (_guided_mode) {
        case Guided_WP:
        {
            if (!_reached_destination) {
                // check if we've reached the destination
                _distance_to_destination = get_distance(rover.current_loc, _destination);
                if (_distance_to_destination <= rover.g.waypoint_radius || location_passed_point(rover.current_loc, _origin, _destination)) {
                    // trigger reached
                    _reached_destination = true;
                    rover.gcs().send_mission_item_reached_message(0);
                }
                // drive towards destination
                calc_lateral_acceleration(_origin, _destination);
                calc_nav_steer();
                calc_throttle(calc_reduced_speed_for_turn_or_distance(_desired_speed), true);
            } else {
                stop_vehicle();
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
                const float yaw_error = wrap_PI(radians((_desired_yaw_cd - ahrs.yaw_sensor) * 0.01f));
                const float steering_out = attitude_control.get_steering_out_angle_error(yaw_error, g2.motors.have_skid_steering(), g2.motors.limit.steer_left, g2.motors.limit.steer_right);
                g2.motors.set_steering(steering_out * 4500.0f);
                calc_throttle(_desired_speed, true);
            } else {
                stop_vehicle();
                g2.motors.set_steering(0.0f);
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
                float steering_out = attitude_control.get_steering_out_rate(radians(_desired_yaw_rate_cds / 100.0f), g2.motors.have_skid_steering(), g2.motors.limit.steer_left, g2.motors.limit.steer_right);
                g2.motors.set_steering(steering_out * 4500.0f);
                calc_throttle(_desired_speed, true);
            } else {
                stop_vehicle();
                g2.motors.set_steering(0.0f);
            }
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

// set desired location
void ModeGuided::set_desired_location(const struct Location& destination)
{
    // call parent
    Mode::set_desired_location(destination);

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
