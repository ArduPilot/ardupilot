#include "mode.h"
#include "Plane.h"

bool ModeGlide::_enter()
{
#if HAL_QUADPLANE_ENABLED
    assist_start_ms = 0;
#endif
    return true;
}

void ModeGlide::update()
{
#if HAL_QUADPLANE_ENABLED
    // with zero forward throttle the VTOL motors would hold a hover
    // waiting for airspeed until the battery is flat, so land if the
    // hover persists. A brief assist, such as from a glitch, ends
    // and the glide continues
    if (plane.quadplane.in_airspeed_wait()) {
        const uint32_t now_ms = AP_HAL::millis();
        if (assist_start_ms == 0) {
            assist_start_ms = now_ms;
        } else if (now_ms - assist_start_ms > 3000) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Glide: VTOL assist holding hover");
            IGNORE_RETURN(plane.set_mode(plane.mode_qland, ModeReason::VTOL_FAILED_TRANSITION));
            return;
        }
    } else {
        assist_start_ms = 0;
    }
#endif
    plane.nav_roll_cd = 0;
    plane.nav_pitch_cd = 0;
}

void ModeGlide::run()
{
    // Run base class function
    Mode::run();

    // Always zero throttle
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);
}

// true if throttle min/max limits should be applied
bool ModeGlide::use_throttle_limits() const
{
    // Never apply throttle limits in glide failsafe
    return false;
}
