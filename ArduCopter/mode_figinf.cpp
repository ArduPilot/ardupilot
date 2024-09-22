#include "Copter.h"

#if MODE_FIGINF_ENABLED

#define CLIMB_SPEED 100 // cm/s

bool ModeFigInf::init(bool ignore_checks)
{
    if(!ignore_checks){
        if(!AP::ahrs().home_is_set()){
            return false;
        }
    }

    _state = SubMode::STARTING;
    _state_complete = true;

    pos_control->init_z_controller();

    return true;
}

void ModeFigInf::run()
{
    if(!motors->armed()){
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        return;
    }

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, 0);

    climb_to_alt();

    pos_control->update_z_controller();

}

void ModeFigInf::climb_to_alt()
{
    float current_alt = pos_control->get_pos_target_z_cm();

    float alt_change = _target_alt_cm - current_alt;
    float slew_rate;
    if(alt_change){
        slew_rate = CLIMB_SPEED;
    }else {
        slew_rate = -get_pilot_speed_dn();
    }

     float dt = 1/100;
     float max_change = fabsf(slew_rate) * dt;
     float clamped_change = (alt_change > 0) ? MIN(alt_change, max_change) : MAX(alt_change, -max_change);
    float slew_alt = current_alt + clamped_change;
    
    pos_control->set_alt_target_with_slew(_target_alt_cm);

    pos_control->set_vel_desired_z_cms(slew_rate);
}
#endif