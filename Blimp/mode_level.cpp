#include "Blimp.h"
/*
 * Init and run calls for level flight mode
 */

bool ModeLevel::init(bool ignore_checks)
{
    if (is_zero(blimp.loiter->lvl_max)){
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "LOIT_LVLMAX is zero. Leveling is disabled.");
    }
    if (loiter->options & Loiter::LVL_EN_YAW_RATE){
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Yaw rate control enabled.");
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Yaw rate control disabled.");
    }
    if (loiter->options & Loiter::LVL_EN_YAW_POS){
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Yaw position control enabled.");
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Yaw position control disabled.");
    }
    if (loiter->options & Loiter::LVL_EN_Z_RATE){
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Z rate control enabled.");
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Z rate control disabled.");
    }
    target_yaw = blimp.ahrs.get_yaw();
    return true;
}

// Runs the main level controller
void ModeLevel::run()
{
    Vector3f pilot;
    float pilot_yaw;
    get_pilot_input(pilot, pilot_yaw);

    float out_right_com = pilot.y*g.max_man_thr;
    float out_front_com = pilot.x*g.max_man_thr;
    float out_down_com = pilot.z;
    float out_yaw_com = pilot_yaw;

    loiter->run_level_roll(out_right_com);
    loiter->run_level_pitch(out_front_com);
    loiter->run_down_stab(out_down_com);
    loiter->run_yaw_stab(out_yaw_com, target_yaw);
}
