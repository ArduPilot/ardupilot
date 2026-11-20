#include "mode.h"
#include "Plane.h"

void ModeGlide::update()
{
    plane.nav_roll_cd = 0;
    plane.nav_pitch_cd = 0;
    if (plane.flight_stage != AP_FixedWing::FlightStage::VTOL) {
        // throttle is always zero, so apply the full low throttle pitch down
        plane.nav_pitch_cd = -plane.g.stab_pitch_down * 100.0f;
    }
    plane.nav_pitch_cd = constrain_int32(plane.nav_pitch_cd, plane.pitch_limit_min*100, plane.aparm.pitch_limit_max.get()*100);
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
