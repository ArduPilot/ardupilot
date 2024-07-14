#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

// Assistance hysteresis helpers

// Reset state
void VTOL_Assist::Assist_Hysteresis::reset()
{
    start_ms = 0;
    last_ms = 0;
    active = false;
}

// Update state, return true when first triggered
bool VTOL_Assist::Assist_Hysteresis::update(const bool trigger, const uint32_t &now_ms, const uint32_t &trigger_delay_ms, const uint32_t &clear_delay_ms)
{
    bool ret = false;

    if (trigger) {
        last_ms = now_ms;
        if (start_ms == 0) {
            start_ms = now_ms;
        }
        if ((now_ms - start_ms) > trigger_delay_ms) {
            // trigger delay has elapsed
            if (!active) {
                // return true on first trigger
                ret = true;
            }
            active = true;
        }

    } else if (active) {
        if ((last_ms == 0) || ((now_ms - last_ms) > clear_delay_ms)) {
            // Clear delay passed
            reset();
        }

    } else {
        reset();
    }

    return ret;
}

// Assistance not needed, reset any state
void VTOL_Assist::reset()
{
    force_assist = false;
    speed_assist = false;
    angle_error.reset();
    alt_error.reset();
}

/*
  return true if the quadplane should provide stability assistance
 */
bool VTOL_Assist::should_assist(float aspeed, bool have_airspeed)
{
    if (!plane.arming.is_armed_and_safety_off() || (state == STATE::ASSIST_DISABLED) || quadplane.tailsitter.is_control_surface_tailsitter()) {
        // disarmed or disabled by aux switch or because a control surface tailsitter
        reset();
        return false;
    }

    if (!quadplane.tailsitter.enabled() && !( (plane.control_mode->does_auto_throttle() && !plane.throttle_suppressed)
                                                                      || is_positive(plane.get_throttle_input()) 
                                                                      || plane.is_flying() ) ) {
        // not in a flight mode and condition where it would be safe to turn on vertial lift motors
        // skip this check for tailsitters because the forward and vertial motors are the same and are controled directly by throttle imput unlike other quadplanes
        reset();
        return false;
    }

    if (plane.flare_mode != Plane::FlareMode::FLARE_DISABLED) {
        // Never active in fixed wing flare
        reset();
        return false;
    }

    force_assist = state == STATE::FORCE_ENABLED;

    if (speed <= 0) {
        // all checks disabled via speed threshold, still allow force enabled
        speed_assist = false;
        alt_error.reset();
        angle_error.reset();
        return force_assist;
    }

    // assistance due to Q_ASSIST_SPEED
    // if option bit is enabled only allow assist with real airspeed sensor
    speed_assist = (have_airspeed && aspeed < speed) && 
       (!quadplane.option_is_set(QuadPlane::OPTION::DISABLE_SYNTHETIC_AIRSPEED_ASSIST) || plane.ahrs.using_airspeed_sensor());

    const uint32_t now_ms = AP_HAL::millis();
    const uint32_t tigger_delay_ms = delay * 1000;
    const uint32_t clear_delay_ms = tigger_delay_ms * 2;

    /*
      optional assistance when altitude is too close to the ground
     */
    if (alt <= 0) {
        // Alt assist disabled
        alt_error.reset();

    } else {
        const float height_above_ground = plane.relative_ground_altitude(plane.g.rangefinder_landing);
        if (alt_error.update(height_above_ground < alt, now_ms, tigger_delay_ms, clear_delay_ms)) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Alt assist %.1fm", height_above_ground);
        }
    }

    if (angle <= 0) {
        // Angle assist disabled
        angle_error.reset();

    } else {

        /*
        now check if we should provide assistance due to attitude error
        */
        const uint16_t allowed_envelope_error_cd = 500U;
        const bool inside_envelope = (labs(plane.ahrs.roll_sensor) <= (plane.aparm.roll_limit*100 + allowed_envelope_error_cd)) &&
                                     (plane.ahrs.pitch_sensor < (plane.aparm.pitch_limit_max*100 + allowed_envelope_error_cd)) &&
                                     (plane.ahrs.pitch_sensor > (plane.aparm.pitch_limit_min*100 - allowed_envelope_error_cd));

        const int32_t max_angle_cd = 100U*angle;
        const bool inside_angle_error = (labs(plane.ahrs.roll_sensor - plane.nav_roll_cd) < max_angle_cd) &&
                                        (labs(plane.ahrs.pitch_sensor - plane.nav_pitch_cd) < max_angle_cd);

        if (angle_error.update(!inside_envelope && !inside_angle_error, now_ms, tigger_delay_ms, clear_delay_ms)) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Angle assist r=%d p=%d",
                                         (int)(plane.ahrs.roll_sensor/100),
                                         (int)(plane.ahrs.pitch_sensor/100));
        }
    }

    return force_assist || speed_assist || alt_error.is_active() || angle_error.is_active();
}

#endif  // HAL_QUADPLANE_ENABLED
