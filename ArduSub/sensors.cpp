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
#if RANGEFINDER_ENABLED == ENABLED
    rangefinder.init(ROTATION_PITCH_270);
    rangefinder_state.alt_cm_filt.set_cutoff_frequency(RANGEFINDER_WPNAV_FILT_HZ);
    rangefinder_state.enabled = rangefinder.has_orientation(ROTATION_PITCH_270);
#endif
}

// return rangefinder altitude in centimeters
void Sub::read_rangefinder()
{
#if RANGEFINDER_ENABLED == ENABLED
    rangefinder.update();

    rangefinder_state.alt_healthy = ((rangefinder.status_orient(ROTATION_PITCH_270) == RangeFinder::Status::Good) && (rangefinder.range_valid_count_orient(ROTATION_PITCH_270) >= RANGEFINDER_HEALTH_MAX));

    int16_t temp_alt = rangefinder.distance_cm_orient(ROTATION_PITCH_270);

#if RANGEFINDER_TILT_CORRECTION == ENABLED
    // correct alt for angle of the rangefinder
    temp_alt = (float)temp_alt * MAX(0.707f, ahrs.get_rotation_body_to_ned().c.z);
#endif

    rangefinder_state.alt_cm = temp_alt;

    // filter rangefinder for use by AC_WPNav
    uint32_t now = AP_HAL::millis();

    if (rangefinder_state.alt_healthy) {
        if (now - rangefinder_state.last_healthy_ms > RANGEFINDER_TIMEOUT_MS) {
            // reset filter if we haven't used it within the last second
            rangefinder_state.alt_cm_filt.reset(rangefinder_state.alt_cm);
        } else {
            rangefinder_state.alt_cm_filt.apply(rangefinder_state.alt_cm, 0.05f);
        }
        rangefinder_state.last_healthy_ms = now;
    }

    // send rangefinder altitude and health to waypoint navigation library
    wp_nav.set_rangefinder_alt(rangefinder_state.enabled, rangefinder_state.alt_healthy, rangefinder_state.alt_cm_filt.get());
    circle_nav.set_rangefinder_alt(rangefinder_state.enabled && wp_nav.rangefinder_used(), rangefinder_state.alt_healthy, rangefinder_state.alt_cm_filt.get());

#else
    rangefinder_state.enabled = false;
    rangefinder_state.alt_healthy = false;
    rangefinder_state.alt_cm = 0;
#endif
}

// return true if rangefinder_alt can be used
bool Sub::rangefinder_alt_ok() const
{
    return (rangefinder_state.enabled && rangefinder_state.alt_healthy);
}
