#include "Copter.h"
#include <cmath>
#include <algorithm>

#if MODE_GUIDED_NOGPS_ENABLED

/*
 * Initialization and run calls for the guided_nogps flight mode
 */

float ModeGuidedNoGPS::normalize_radians(float radians) {
    return std::fmod(radians + M_PI, 2.0f * M_PI) - M_PI;
}

float ModeGuidedNoGPS::degrees_to_radians(float degrees) {
    return degrees * M_PI / 180.0f;
}

float ModeGuidedNoGPS::normalize_angle_deg(float angle) {
    return std::fmod(std::fmod(angle, 360.0f) + 360.0f, 360.0f);
}

// Initialize the guided_nogps controller
bool ModeGuidedNoGPS::init(bool ignore_checks)
{
    // Start the angle control mode
    ModeGuided::angle_control_start();

    // Set parameters
    fly_angle = copter.aparm.angle_max / 100.0f; // maximum tilt angle in radians (angle_max in hundredths of a degree)
    interval_ms = 100.0f;                        // update interval in milliseconds

    // Minimum height and yaw
    fly_alt_min = g.rtl_altitude / 100.0f;       // minimum height above the home
    home_yaw = g.dr_home_yaw < 1 ? copter.azimuth_to_home : static_cast<float>(g.dr_home_yaw);
    home_yaw = degrees_to_radians(normalize_angle_deg(home_yaw)); // convert home_yaw to radians
    timeout = g2.failsafe_dr_timeout;

    // Initial value of climb_rate
    climb_rate = 0.0f;

    // Information message
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

    // Get the current yaw
    float current_yaw = AP::ahrs().get_yaw();

    // Calculate the difference between the desired (home_yaw) and the current yaw
    float error = home_yaw - current_yaw;
    while (error > M_PI) error -= 2.0f * M_PI;
    while (error < -M_PI) error += 2.0f * M_PI;

    // Пороговая ошибка для начала движения вперед (например, 10 градусов)
    float yaw_threshold = radians(10.0f);

    // Быстрый поворот: увеличим smoothing_factor
    // Чем больше error, тем выше factor
    float base_factor = 0.2f; // базовый фактор, выше чем 0.1f для более быстрого поворота
    // Можно сделать factor зависимым от ошибки:
    // например, factor растёт от base_factor до base_factor*2 при больших ошибках
    float factor_multiplier = 1.0f + std::min(fabsf(error) / M_PI, 1.0f);
    float smoothing_factor = base_factor * factor_multiplier;

    float new_yaw = current_yaw + error * smoothing_factor;

    float pitch_angle = 0.0f;

    // Если ошибка по yaw больше порога - просто поворачиваемся на месте, без наклона вперед.
    if (fabsf(error) < yaw_threshold) {
        // Теперь можно лететь вперед
        pitch_angle = -radians(fly_angle);
    } else {
        // Пока большая угловая ошибка - только поворот
        // pitch_angle = 0.0f; // уже задано выше
    }

    // roll оставляем 0, если нужно - можно добавить логику
    float roll_angle = 0.0f;

    q.from_euler(roll_angle, pitch_angle, new_yaw);

    // Set target angles and vertical speed
    ModeGuided::set_angle(q, Vector3f{}, climb_rate * 100.0f, false);
}

#endif
