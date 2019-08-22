#include "Copter.h"

// adjust_climb_rate - hold copter at the desired distance above the
//      ground; returns climb rate (in cm/s) which should be passed to
//      the position controller
float Copter::SurfaceTracking::adjust_climb_rate(float target_rate)
{
#if RANGEFINDER_ENABLED == ENABLED
    if (!copter.rangefinder_alt_ok()) {
        // if rangefinder is not ok, do not use surface tracking
        return target_rate;
    }

    const float current_alt = copter.inertial_nav.get_altitude();
    const float current_alt_target = copter.pos_control->get_alt_target();

    // reset target altitude if this controller has just been engaged
    const uint32_t now = millis();
    if (now - last_update_ms > SURFACE_TRACKING_TIMEOUT_MS) {
        target_alt_cm = copter.rangefinder_state.alt_cm + current_alt_target - current_alt;
    }
    last_update_ms = now;

    // adjust rangefinder target alt if motors have not hit their limits
    if ((target_rate<0 && !copter.motors->limit.throttle_lower) || (target_rate>0 && !copter.motors->limit.throttle_upper)) {
        target_alt_cm += target_rate * copter.G_Dt;
    }
    valid_for_logging = true;

    /*
      handle rangefinder glitches. When we get a rangefinder reading
      more than RANGEFINDER_GLITCH_ALT_CM different from the current
      rangefinder reading then we consider it a glitch and reject
      until we get RANGEFINDER_GLITCH_NUM_SAMPLES samples in a
      row. When that happens we reset the target altitude to the new
      reading
     */
    const int32_t glitch_cm = copter.rangefinder_state.alt_cm - target_alt_cm;
    if (glitch_cm >= RANGEFINDER_GLITCH_ALT_CM) {
        copter.rangefinder_state.glitch_count = MAX(copter.rangefinder_state.glitch_count+1,1);
    } else if (glitch_cm <= -RANGEFINDER_GLITCH_ALT_CM) {
        copter.rangefinder_state.glitch_count = MIN(copter.rangefinder_state.glitch_count-1,-1);
    } else {
        copter.rangefinder_state.glitch_count = 0;
    }
    if (abs(copter.rangefinder_state.glitch_count) >= RANGEFINDER_GLITCH_NUM_SAMPLES) {
        // shift to the new rangefinder reading
        target_alt_cm = copter.rangefinder_state.alt_cm;
        copter.rangefinder_state.glitch_count = 0;
    }
    if (copter.rangefinder_state.glitch_count != 0) {
        // we are currently glitching, just use the target rate
        return target_rate;
    }

    // calc desired velocity correction from target rangefinder alt vs actual rangefinder alt (remove the error already passed to Altitude controller to avoid oscillations)
    const float distance_error = (target_alt_cm - copter.rangefinder_state.alt_cm) - (current_alt_target - current_alt);
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
