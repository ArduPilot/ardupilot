#include "mode.h"
#include "Plane.h"

void ModeTraining::update()
{
    plane.training_manual_roll = false;
    plane.training_manual_pitch = false;
    plane.update_load_factor();

    // if the roll is past the set roll limit, then
    // we set target roll to the limit
    if (ahrs.roll_sensor >= plane.roll_limit_cd) {
        plane.nav_roll_cd = plane.roll_limit_cd;
    } else if (ahrs.roll_sensor <= -plane.roll_limit_cd) {
        plane.nav_roll_cd = -plane.roll_limit_cd;
    } else {
        plane.training_manual_roll = true;
        plane.nav_roll_cd = 0;
    }

    // if the pitch is past the set pitch limits, then
    // we set target pitch to the limit
    if (ahrs.pitch_sensor >= plane.aparm.pitch_limit_max*100) {
        plane.nav_pitch_cd = plane.aparm.pitch_limit_max*100;
    } else if (ahrs.pitch_sensor <= plane.pitch_limit_min*100) {
        plane.nav_pitch_cd = plane.pitch_limit_min*100;
    } else {
        plane.training_manual_pitch = true;
        plane.nav_pitch_cd = 0;
    }
    if (plane.fly_inverted()) {
        plane.nav_pitch_cd = -plane.nav_pitch_cd;
    }
}

/*
  a special stabilization function for training mode
 */
void ModeTraining::run()
{
    const float rexpo = plane.roll_in_expo(false);
    const float pexpo = plane.pitch_in_expo(false);
    if (plane.training_manual_roll) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, rexpo);
    } else {
        // calculate what is needed to hold
        plane.stabilize_roll();
        if ((plane.nav_roll_cd > 0 && rexpo < SRV_Channels::get_output_scaled(SRV_Channel::k_aileron)) ||
            (plane.nav_roll_cd < 0 && rexpo > SRV_Channels::get_output_scaled(SRV_Channel::k_aileron))) {
            // allow user to get out of the roll
            SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, rexpo);
        }
    }

    if (plane.training_manual_pitch) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, pexpo);
    } else {
        plane.stabilize_pitch();
        if ((plane.nav_pitch_cd > 0 && pexpo < SRV_Channels::get_output_scaled(SRV_Channel::k_elevator)) ||
            (plane.nav_pitch_cd < 0 && pexpo > SRV_Channels::get_output_scaled(SRV_Channel::k_elevator))) {
            // allow user to get back to level
            SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, pexpo);
        }
    }

    // Always manual rudder control
    output_rudder_and_steering(plane.rudder_in_expo(false));

    output_pilot_throttle();

}
