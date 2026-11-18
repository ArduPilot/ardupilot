#include "mode.h"
#include "Plane.h"

bool ModeAutoRTL::_enter()
{
    if (!mission_joined) {
        // a plain mode request joins the return path if there is one, else the landing sequence
        if (!plane.have_position ||
            (!plane.mission.jump_to_closest_mission_leg(plane.current_loc) &&
             !plane.mission.jump_to_landing_sequence(plane.current_loc))) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Auto RTL: no return path or landing sequence found");
            return false;
        }
    }

    // carry on from where the jump put the mission rather than restarting it
    plane.mission.set_force_resume(true);
    if (!ModeAuto::_enter()) {
        plane.mission.set_force_resume(false);
        return false;
    }
    return true;
}

bool ModeAutoRTL::enter_landing_sequence(ModeReason reason)
{
    if (!plane.have_position || !plane.mission.jump_to_landing_sequence(plane.current_loc)) {
        return false;
    }
    return enter_joined(reason);
}

bool ModeAutoRTL::enter_return_path(ModeReason reason)
{
    if (!plane.have_position || !plane.mission.jump_to_closest_mission_leg(plane.current_loc)) {
        return false;
    }
    return enter_joined(reason);
}

bool ModeAutoRTL::enter_joined(ModeReason reason)
{
    mission_joined = true;
    // already in Auto RTL means the mission has moved and there is nothing more to do
    const bool entered = plane.set_mode(*this, reason);
    mission_joined = false;
    return entered;
}

// returns true if the vehicle can be armed in this mode
bool ModeAutoRTL::_pre_arm_checks(size_t buflen, char *buffer) const
{
    hal.util->snprintf(buffer, buflen, "in Auto RTL");
    return false;
}
