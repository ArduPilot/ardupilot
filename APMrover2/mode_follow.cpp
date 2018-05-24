#include "mode.h"
#include "Rover.h"

// initialize follow mode
bool ModeFollow::_enter()
{
    return ModeGuided::enter();
}

void ModeFollow::update()
{
    // variables to be sent to velocity controller
    Vector3f desired_velocity_neu_cms;
    float yaw_cd = 0.0f;

    Vector3f dist_vec;  //vector to lead vehicle
    Vector3f dist_vec_offs; // vector to lead vehicle + offset
    Vector3f vel_of_target; // velocity of lead vehicle

    if (g2.follow.get_target_dist_and_vel_ned(dist_vec, dist_vec_offs, vel_of_target)) {

        // convert dist_vec_offs to cm in NEU
        const Vector3f dist_vec_offs_neu(dist_vec_offs.x * 100.f, dist_vec_offs.y * 100.0f, -dist_vec_offs.z * 100.0f);

        // calculate desired velocity vector in cm/s in NEU
        const float kp = g2.follow.get_pos_p().kP();
        desired_velocity_neu_cms.x = (vel_of_target.x * 100.0f) + (dist_vec_offs_neu.x * kp);
        desired_velocity_neu_cms.y = (vel_of_target.y * 100.0f) + (dist_vec_offs_neu.y * kp);

        // scaled desired velocity to stay within horizontal speed limit
        float desired_speed = safe_sqrt(sq(desired_velocity_neu_cms.x) + sq(desired_velocity_neu_cms.y));
        if (!is_zero(desired_speed) && (desired_speed > _desired_speed)) {
            const float scalar_xy = _desired_speed / desired_speed;
            desired_velocity_neu_cms.x *= scalar_xy;
            desired_velocity_neu_cms.y *= scalar_xy;
            desired_speed = _desired_speed;
        }

        // unit vector towards target position (i.e. vector to lead vehicle + offset)
        Vector3f dir_to_target_neu = dist_vec_offs_neu;
        const float dir_to_target_neu_len = dir_to_target_neu.length();
        if (!is_zero(dir_to_target_neu_len)) {
            dir_to_target_neu /= dir_to_target_neu_len;
        }

        // calculate vehicle heading
        const Vector3f dist_vec_xy(dist_vec_offs.x, dist_vec_offs.y, 0.0f);
        if (dist_vec_xy.length() > 1.0f) {
            yaw_cd = get_bearing_cd(Vector3f(), dist_vec_xy);
        }
    }

    // re-use guided mode's heading and speed controller
    ModeGuided::set_desired_heading_and_speed(yaw_cd, safe_sqrt(sq(desired_velocity_neu_cms.x) + sq(desired_velocity_neu_cms.y)));

    ModeGuided::update();
}
