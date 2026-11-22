#include "mode.h"
#include "Plane.h"

void ModeGlide::update()
{
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
