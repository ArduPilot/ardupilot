#include "Blimp.h"
/*
 * Init and run calls for velocity flight mode
 */

#include <AP_Vehicle/AP_MultiCopter.h>

// Runs the main velocity controller
void ModeVelocity::run()
{
    Vector3f target_vel;
    target_vel.x = channel_front->get_control_in() / float(RC_SCALE) * g.max_vel_xy;
    target_vel.y = channel_right->get_control_in() / float(RC_SCALE) * g.max_vel_xy;
    blimp.rotate_BF_to_NE(target_vel.xy());
    target_vel.z = channel_down->get_control_in()  / float(RC_SCALE) * g.max_vel_z;
    float target_vel_yaw = channel_yaw->get_control_in() / float(RC_SCALE) * g.max_vel_yaw;

    Vector2f actuator = blimp.pid_vel_xy.update_all(target_vel, blimp.vel_ned_filtd, {0,0,0});
    blimp.rotate_NE_to_BF(actuator);
    float act_down = blimp.pid_vel_z.update_all(target_vel.z, blimp.vel_ned_filtd.z);
    float act_yaw = blimp.pid_vel_yaw.update_all(target_vel_yaw, blimp.vel_yaw_filtd);

    if(!(blimp.g.dis_mask & (1<<(2-1)))){
        motors->front_out = actuator.x;
    }
    if(!(blimp.g.dis_mask & (1<<(1-1)))){
        motors->right_out = actuator.y;
    }
    if(!(blimp.g.dis_mask & (1<<(3-1)))){
        motors->down_out = act_down;
    }
    if(!(blimp.g.dis_mask & (1<<(4-1)))){
        motors->yaw_out = act_yaw;
    }

    AP::logger().Write_PSCN(0.0, blimp.pos_ned.x * 100.0, 0.0, 0.0, blimp.vel_ned_filtd.x * 100.0, 0.0, 0.0, 0.0);
    AP::logger().Write_PSCE(0.0, blimp.pos_ned.y * 100.0, 0.0, 0.0, blimp.vel_ned_filtd.y * 100.0, 0.0, 0.0, 0.0);
    AP::logger().Write_PSCD(0.0, -blimp.pos_ned.z * 100.0, 0.0, 0.0, -blimp.vel_ned_filtd.z * 100.0, 0.0, 0.0, 0.0);
}
