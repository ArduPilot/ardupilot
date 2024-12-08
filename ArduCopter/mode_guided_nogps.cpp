#include "Copter.h"
#include <cmath>
#include <algorithm>

#if MODE_GUIDED_NOGPS_ENABLED

/*
 * Initialization and run calls for the guided_nogps flight mode
 */

// Initialize the guided_nogps controller
bool ModeGuidedNoGPS::init(bool ignore_checks)
{
    // Start the angle control mode
    ModeGuided::angle_control_start();

    // Set the parameters for the flight mode
    fly_angle = copter.aparm.angle_max / 100.0f; // Convert to a float value
    interval_ms = 100.0f;                        // Update interval in milliseconds

    // Minimum altitude and yaw settings
    fly_alt_min = g.rtl_altitude / 100.0f;       // Convert to meters
    home_yaw = g.dr_home_yaw < 1 ? copter.azimuth_to_home : static_cast<float>(g.dr_home_yaw);
    timeout = g2.failsafe_dr_timeout;

    // GCS Info Message
    gcs().send_text(MAV_SEVERITY_INFO, "DR Start");

    return true;
}

// Run the guided_nogps controller logic
void ModeGuidedNoGPS::run()
{
    // Run the angle control logic
    ModeGuided::angle_control_run();

    // Calculate the current altitude below home
    float curr_alt_below_home = 0.0f;
    AP::ahrs().get_relative_position_D_home(curr_alt_below_home);

    // Calculate the target altitude above the vehicle
    float target_alt_above_vehicle = fly_alt_min + curr_alt_below_home;

    float climb_rate_chg_max = interval_ms * 0.001f * (wp_nav->get_accel_z() * 0.01f);

    climb_rate = std::min(target_alt_above_vehicle * 0.1f,
                          std::min(wp_nav->get_default_speed_up() * 0.01f, climb_rate + climb_rate_chg_max));

    // Set the quaternion for yaw, pitch, and roll control
    q.from_euler(radians(0.0f), -radians(fly_angle), radians(home_yaw));

    // Set the target angle and climb rate
    ModeGuided::set_angle(q, Vector3f{}, climb_rate * 100.0f, false);
}

#endif