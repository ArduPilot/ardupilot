#include "mode.h"
#include "Rover.h"

bool ModeGuided::_enter()
{
    /*
      when entering guided mode we set the target as the current
      location. This matches the behaviour of the copter code.
    */
    lateral_acceleration = 0.0f;
    rover.set_guided_WP(rover.current_loc);
    g2.motors.slew_limit_throttle(true);
    return true;
}

void ModeGuided::update()
{
    if (!rover.in_auto_reverse) {
        rover.set_reverse(false);
    }

    switch (guided_mode) {
        case Guided_Angle:
            rover.nav_set_yaw_speed();
            break;

        case Guided_WP:
            if (rover.rtl_complete || rover.verify_RTL()) {
                // we have reached destination so stop where we are
                if (fabsf(g2.motors.get_throttle()) > g.throttle_min.get()) {
                    rover.gcs().send_mission_item_reached_message(0);
                }
                g2.motors.set_throttle(g.throttle_min.get());
                g2.motors.set_steering(0.0f);
                lateral_acceleration = 0.0f;
            } else {
                calc_lateral_acceleration();
                calc_nav_steer();
                calc_throttle(rover.guided_control.target_speed);
                rover.Log_Write_GuidedTarget(guided_mode, Vector3f(rover.next_WP.lat, rover.next_WP.lng, rover.next_WP.alt),
                                       Vector3f(rover.guided_control.target_speed, g2.motors.get_throttle(), 0.0f));
            }
            break;

        case Guided_Velocity:
            rover.nav_set_speed();
            break;

        default:
            gcs().send_text(MAV_SEVERITY_WARNING, "Unknown GUIDED mode");
            break;
    }
}

void ModeGuided::update_navigation()
{
    // no loitering around the wp with the rover, goes direct to the wp position
    if (guided_mode == Guided_WP && (rover.rtl_complete || rover.verify_RTL())) {
        // we have reached destination so stop where we are
        g2.motors.set_throttle(g.throttle_min.get());
        g2.motors.set_steering(0.0f);
        lateral_acceleration = 0.0f;
    }
}
