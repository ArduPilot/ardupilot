#include "Copter.h"

//
// pre-takeoff checks
//

// detects if the vehicle should be allowed to takeoff or not and sets the motors.blocked flag
void Copter::takeoff_check()
{
#if HAL_WITH_ESC_TELEM && FRAME_CONFIG != HELI_FRAME
    motors_takeoff_check(g2.takeoff_rpm_min, g2.takeoff_rpm_max, ap.land_complete);
#endif
}
