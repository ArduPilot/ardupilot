#include "mode.h"
#include "Plane.h"

bool ModeTraining::_enter()
{
    plane.throttle_allows_nudging = false;
    plane.auto_throttle_mode = false;
    plane.auto_navigation_mode = false;

    return true;
}

void ModeTraining::update()
{
    plane.mode_training.manual_roll = false;
    plane.mode_training.manual_pitch = false;
    plane.update_load_factor();

    // if the roll is past the set roll limit, then
    // we set target roll to the limit
    if (plane.ahrs.roll_sensor >= plane.roll_limit_cd) {
        plane.nav_roll_cd = plane.roll_limit_cd;
    } else if (plane.ahrs.roll_sensor <= -plane.roll_limit_cd) {
        plane.nav_roll_cd = -plane.roll_limit_cd;
    } else {
        plane.mode_training.manual_roll = true;
        plane.nav_roll_cd = 0;
    }

    // if the pitch is past the set pitch limits, then
    // we set target pitch to the limit
    if (plane.ahrs.pitch_sensor >= plane.aparm.pitch_limit_max_cd) {
        plane.nav_pitch_cd = plane.aparm.pitch_limit_max_cd;
    } else if (plane.ahrs.pitch_sensor <= plane.pitch_limit_min_cd) {
        plane.nav_pitch_cd = plane.pitch_limit_min_cd;
    } else {
        plane.mode_training.manual_pitch = true;
        plane.nav_pitch_cd = 0;
    }
    if (plane.fly_inverted()) {
        plane.nav_pitch_cd = -plane.nav_pitch_cd;
    }
}

