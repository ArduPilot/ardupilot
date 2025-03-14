#include "Copter.h"

//
// pre-takeoff checks
//

// detects if the vehicle should be allowed to takeoff or not and sets the motors.blocked flag
void Copter::takeoff_check()
{
#if HAL_WITH_ESC_TELEM && FRAME_CONFIG != HELI_FRAME
    // If vehicle is armed and flying then clear block and return
    if (motors->armed() && !ap.land_complete) {
        motors->set_spoolup_block(false);
        return;
    }

    // if motors have become unblocked return immediately
    // this ensures the motors can only be blocked immediate after arming
    if (motors->armed() && !motors->get_spoolup_block()) {
        return;
    }

    // Run the common takeoff checks
    motors->set_spoolup_block(!motors_takeoff_check(g2.takeoff_rpm_min, g2.takeoff_rpm_max));
#endif
}
