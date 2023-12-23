#include "Blimp.h"
/*
 * Init and run calls for velocity flight mode
 */

#include <AP_Vehicle/AP_MultiCopter.h>

// Runs the main velocity controller
void ModeVelocity::run()
{
    Vector3f target_vel;
    float target_vel_yaw;
    get_pilot_input(target_vel, target_vel_yaw);
    target_vel.x *= g.max_vel_xy;
    target_vel.y *= g.max_vel_xy;
    if (g.simple_mode == 0) {
        //If simple mode is disabled, input is in body-frame, thus needs to be rotated.
        blimp.rotate_BF_to_NE(target_vel.xy());
    }
    target_vel.z *= g.max_vel_z;
    target_vel_yaw *= g.max_vel_yaw;

    blimp.loiter->run_vel(target_vel, target_vel_yaw, Vector4b{false,false,false,false});
}
