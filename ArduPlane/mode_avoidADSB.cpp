#include "mode.h"
#include "Plane.h"

#if HAL_ADSB_ENABLED

bool ModeAvoidADSB::_enter()
{
    return plane.mode_guided.enter();
}

void ModeAvoidADSB::update()
{
    plane.mode_guided.update();
}

void ModeAvoidADSB::navigate()
{
    // Zero indicates to use WP_LOITER_RAD
    plane.update_loiter(0);
}

// Return the long failsafe action that should be taken in this mode
failsafe_action_long ModeAvoidADSB::long_failsafe_action() const
{
    if ((plane.g.fs_action_long == (int8_t)failsafe_action_long::DEPLOY_PARACHUTE) ||
        (plane.g.fs_action_long == (int8_t)failsafe_action_long::GLIDE) ||
        (plane.g.fs_action_long == (int8_t)failsafe_action_long::AUTO) ||
        (plane.g.fs_action_long == (int8_t)failsafe_action_long::RTL)) {
        return failsafe_action_long(plane.g.fs_action_long.get());
    }

    return failsafe_action_long::CONTINUE;
}

#endif // HAL_ADSB_ENABLED

