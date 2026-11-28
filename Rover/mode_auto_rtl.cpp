#include "Rover.h"

#if MODE_AUTORTL_ENABLED

// ModeAutoRTL::_enter - join the DO_RETURN_PATH_START corridor at the
// closest point and follow it home by running the AUTO mission from there.
//
// This is a full Rover flight mode (not a pseudo-mode redirect) that
// inherits all mission-navigation logic from ModeAuto while sharing
// rover.mission.  If no mission or no DO_RETURN_PATH_START return path
// exists, entry is refused cleanly without disturbing the current mode.
bool ModeAutoRTL::_enter()
{
    if (!rover.mission.present()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "No Mission. Can't set AUTO RTL.");
        return false;
    }

    // Find the closest leg after DO_RETURN_PATH_START before clobbering
    // wp_nav or guided state in ModeAuto::_enter().
    if (!rover.mission.jump_to_closest_mission_leg(rover.current_loc)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                      "Auto RTL: no return path in mission");
        return false;
    }

    // Force mission resume from the jumped index regardless of MIS_RESTART.
    rover.mission.set_force_resume(true);

    if (!ModeAuto::_enter()) {
        rover.mission.set_force_resume(false);
        return false;
    }

    // Auto RTL must begin driving immediately without waiting for
    // AUTO_KICKSTART or AUTO_TRIGGER_PIN.
    auto_triggered = true;

    return true;
}

#endif // MODE_AUTORTL_ENABLED
