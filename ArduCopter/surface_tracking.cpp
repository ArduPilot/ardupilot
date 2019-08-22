#include "Copter.h"

// adjust_climb_rate - hold copter at the desired distance above the
//      ground; returns climb rate (in cm/s) which should be passed to
//      the position controller
float Copter::SurfaceTracking::adjust_climb_rate(float target_rate)
{
#if RANGEFINDER_ENABLED == ENABLED
    // if rangefinder alt is not ok or glitching, do not do surface tracking
    if (!copter.rangefinder_alt_ok() ||
        (copter.rangefinder_state.glitch_count != 0)) {
        return target_rate;
    }

    // calculate current ekf based altitude error
    const float current_alt_error = copter.pos_control->get_alt_target() - copter.inertial_nav.get_altitude();

    // reset target altitude if this controller has just been engaged
    const uint32_t now = millis();
    if (now - last_update_ms > SURFACE_TRACKING_TIMEOUT_MS) {
        target_alt_cm = copter.rangefinder_state.alt_cm + current_alt_error;
        last_glitch_cleared_ms = copter.rangefinder_state.glitch_cleared_ms;
    }
    last_update_ms = now;

    // adjust rangefinder target alt if motors have not hit their limits
    if ((target_rate<0 && !copter.motors->limit.throttle_lower) || (target_rate>0 && !copter.motors->limit.throttle_upper)) {
        target_alt_cm += target_rate * copter.G_Dt;
    }
    valid_for_logging = true;

    // handle glitch recovery by resetting target
    if (copter.rangefinder_state.glitch_cleared_ms != last_glitch_cleared_ms) {
        // shift to the new rangefinder reading
        target_alt_cm = copter.rangefinder_state.alt_cm + current_alt_error;
        last_glitch_cleared_ms = copter.rangefinder_state.glitch_cleared_ms;
    }

    // calc desired velocity correction from target rangefinder alt vs actual rangefinder alt (remove the error already passed to Altitude controller to avoid oscillations)
    const float distance_error = (target_alt_cm - copter.rangefinder_state.alt_cm) - current_alt_error;
    float velocity_correction = distance_error * copter.g.rangefinder_gain;
    velocity_correction = constrain_float(velocity_correction, -SURFACE_TRACKING_VELZ_MAX, SURFACE_TRACKING_VELZ_MAX);

    // return combined pilot climb rate + rate to correct rangefinder alt error
    return (target_rate + velocity_correction);
#else
    return target_rate;
#endif
}

// get target altitude (in cm) above ground
// returns true if there is a valid target
bool Copter::SurfaceTracking::get_target_alt_cm(float &_target_alt_cm) const
{
    // check target has been updated recently
    if (AP_HAL::millis() - last_update_ms > SURFACE_TRACKING_TIMEOUT_MS) {
        return false;
    }
    _target_alt_cm = target_alt_cm;
    return true;
}

// set target altitude (in cm) above ground
void Copter::SurfaceTracking::set_target_alt_cm(float _target_alt_cm)
{
    target_alt_cm = _target_alt_cm;
    last_update_ms = AP_HAL::millis();
}

float Copter::SurfaceTracking::logging_target_alt() const
{
    if (!valid_for_logging) {
        return AP::logger().quiet_nan();
    }
    return target_alt_cm * 0.01f; // cm->m
}
