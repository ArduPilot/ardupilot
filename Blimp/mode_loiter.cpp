#include "Blimp.h"
/*
 * Init and run calls for loiter flight mode
 */

bool ModeLoiter::init(bool ignore_checks)
{
    target_pos = blimp.pos_ned;
    target_yaw = blimp.ahrs.get_yaw_rad();

    return true;
}

//Runs the main loiter controller
void ModeLoiter::run()
{
    const float dt = blimp.scheduler.get_last_loop_time_s();

    Vector3f pilot;
    float pilot_yaw;
    get_pilot_input(pilot, pilot_yaw);
    if (g.simple_mode == 0) {
        //If simple mode is disabled, input is in body-frame, thus needs to be rotated.
        blimp.rotate_BF_to_NE(pilot.xy());
    }
    pilot.x *= loiter->max_pos_x * dt;
    pilot.y *= loiter->max_pos_y * dt;
    pilot.z *= loiter->max_pos_z * dt;
    pilot_yaw *= loiter->max_pos_yaw * dt;

    // This keeps the target position from getting too far away from the blimp's actual position.
    Vector4b close;
    close.x = fabsf(target_pos.x-blimp.pos_ned.x) < (loiter->max_pos_x*blimp.loiter->pos_lag);
    close.y = fabsf(target_pos.y-blimp.pos_ned.y) < (loiter->max_pos_y*blimp.loiter->pos_lag);
    close.z = fabsf(target_pos.z-blimp.pos_ned.z) < (loiter->max_pos_z*blimp.loiter->pos_lag);
    close.yaw = fabsf(wrap_PI(target_yaw-ahrs.get_yaw())) < (loiter->max_pos_yaw*blimp.loiter->pos_lag);

    Vector4b targ_negative;
    targ_negative.x = (target_pos.x > blimp.pos_ned.x) && (pilot.x < 0);
    targ_negative.y = (target_pos.y > blimp.pos_ned.y) && (pilot.y < 0);

    Vector4b targ_positive;
    targ_positive.x = (target_pos.x < blimp.pos_ned.x) && (pilot.x > 0);
    targ_positive.y = (target_pos.y < blimp.pos_ned.y) && (pilot.y > 0);


    if ((close.x && close.y && close.z && close.yaw)) {
        target_pos.x += pilot.x;
        target_pos.y += pilot.y;
        target_pos.z += pilot.z;
        target_yaw = wrap_PI(target_yaw + pilot_yaw);
    } else if ((close.x && close.z && (targ_negative.y || targ_positive.y)) ||
               (close.y && close.z && (targ_negative.x || targ_positive.x))) {
        target_pos.x += pilot.x;
        target_pos.y += pilot.y;
    }

    if (!blimp.motors->armed()) {
        target_pos = blimp.pos_ned;
        target_yaw = blimp.ahrs.get_yaw();
    }

    blimp.loiter->run(target_pos, target_yaw, Vector4b{false,false,false,false});
}
