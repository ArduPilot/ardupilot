/*
  Rover specific AP_AdvancedFailsafe class
 */

#include "Rover.h"

#if ADVANCED_FAILSAFE == ENABLED

/*
  Setup radio_out values for all channels to termination values
 */
void AP_AdvancedFailsafe_Rover::terminate_vehicle(void)
{
    // disarm as well
    AP::arming().disarm(AP_Arming::Method::AFS);

    // Set to HOLD mode
    rover.set_mode(rover.mode_hold, ModeReason::CRASH_FAILSAFE);
}

/*
  Return an AFS_MODE for current control mode
 */
AP_AdvancedFailsafe::control_mode AP_AdvancedFailsafe_Rover::afs_mode(void)
{
    if (rover.control_mode->is_autopilot_mode()) {
        return AP_AdvancedFailsafe::AFS_AUTO;
    }
    return AP_AdvancedFailsafe::AFS_STABILIZED;
}

#endif  // ADVANCED_FAILSAFE
