#include "mode.h"
#include "Plane.h"

void ModeStabilize::update()
{
    plane.nav_roll_cd = 0;
    plane.nav_pitch_cd = 0;
}

void ModeStabilize::run()
{
    // note this is actually in deg/s for some SID_AXIS values for yaw
    Vector3f offset_deg;
#if AP_PLANE_SYSTEMID_ENABLED
    auto &systemid = plane.g2.systemid;
    systemid.update();
    offset_deg = systemid.get_attitude_offset_deg();
#endif
    plane.nav_roll_cd += offset_deg.x * 100.0f;
    plane.nav_pitch_cd += offset_deg.y * 100.0f;
    
    plane.stabilize_roll();
    plane.stabilize_pitch();
    stabilize_stick_mixing_direct();
    plane.stabilize_yaw();

    output_pilot_throttle();
}
