#include "Sub.h"

// return barometric altitude in centimeters
void Sub::read_barometer()
{
    barometer.update();
    // If we are reading a positive altitude, the sensor needs calibration
    // Even a few meters above the water we should have no significant depth reading
    if(barometer.get_altitude() > 0) {
        barometer.update_calibration();
    }

    if (ap.depth_sensor_present) {
        sensor_health.depth = barometer.healthy(depth_sensor_idx);
    }
}

void Sub::init_rangefinder()
{
#if AP_RANGEFINDER_ENABLED
    rangefinder.set_log_rfnd_bit(MASK_LOG_CTUN);
    rangefinder.init(ROTATION_PITCH_270);
    rangefinder_state.alt_filt.set_cutoff_frequency(RANGEFINDER_WPNAV_FILT_HZ);
    rangefinder_state.enabled = rangefinder.has_orientation(ROTATION_PITCH_270);
#endif
}

// return rangefinder altitude in centimeters
void Sub::read_rangefinder()
{
#if AP_RANGEFINDER_ENABLED
    rangefinder.update();

    // signal quality ranges from 0 (worst) to 100 (perfect), -1 means n/a
    int8_t signal_quality_pct = rangefinder.signal_quality_pct_orient(ROTATION_PITCH_270);

    rangefinder_state.alt_healthy =
            (rangefinder.status_orient(ROTATION_PITCH_270) == RangeFinder::Status::Good) &&
            (rangefinder.range_valid_count_orient(ROTATION_PITCH_270) >= RANGEFINDER_HEALTH_MAX) &&
            (signal_quality_pct == -1 || signal_quality_pct >= g.rangefinder_signal_min);

    float temp_alt_m = rangefinder.distance_orient(ROTATION_PITCH_270);

#if RANGEFINDER_TILT_CORRECTION
    // correct alt for angle of the rangefinder
    temp_alt_m = temp_alt_m * MAX(0.707f, ahrs.get_rotation_body_to_ned().c.z);
#endif

    rangefinder_state.alt = temp_alt_m;
    rangefinder_state.inertial_alt_cm = inertial_nav.get_position_z_up_cm();
    rangefinder_state.min = rangefinder.min_distance_orient(ROTATION_PITCH_270);
    rangefinder_state.max = rangefinder.max_distance_orient(ROTATION_PITCH_270);

    // calculate rangefinder_terrain_offset_cm
    if (rangefinder_state.alt_healthy) {
        uint32_t now = AP_HAL::millis();
        if (now - rangefinder_state.last_healthy_ms > RANGEFINDER_TIMEOUT_MS) {
            // reset filter if we haven't used it within the last second
            rangefinder_state.alt_filt.reset(rangefinder_state.alt);
        } else {
            rangefinder_state.alt_filt.apply(rangefinder_state.alt, 0.05f);
        }
        rangefinder_state.last_healthy_ms = now;
        rangefinder_state.rangefinder_terrain_offset_cm =
            sub.rangefinder_state.inertial_alt_cm - (sub.rangefinder_state.alt_filt.get() * 100);
    }

    // send rangefinder altitude and health to waypoint navigation library
    wp_nav.set_rangefinder_terrain_U_cm(
            rangefinder_state.enabled,
            rangefinder_state.alt_healthy,
            rangefinder_state.rangefinder_terrain_offset_cm);
    circle_nav.set_rangefinder_terrain_U_cm(
            rangefinder_state.enabled && wp_nav.rangefinder_used(),
            rangefinder_state.alt_healthy,
            rangefinder_state.rangefinder_terrain_offset_cm);
#endif  // AP_RANGEFINDER_ENABLED
}

// return true if rangefinder_alt can be used
bool Sub::rangefinder_alt_ok() const
{
    uint32_t now = AP_HAL::millis();
    return (rangefinder_state.enabled && rangefinder_state.alt_healthy && now - rangefinder_state.last_healthy_ms < RANGEFINDER_TIMEOUT_MS);
}
