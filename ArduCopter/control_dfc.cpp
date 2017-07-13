#include "Copter.h"

/*
 * Init and run calls for dfc-stabilize flight mode
 */

// dfc_init - initialise dfc-stabilize controller
bool Copter::dfc_init(bool ignore_checks)
{
    // if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
    if (motors->armed() && ap.land_complete && !mode_has_manual_throttle(control_mode) &&
            (get_pilot_desired_throttle(channel_throttle->get_control_in()) > get_non_takeoff_throttle())) {
        return false;
    }
    // set target altitude to zero for reporting
    pos_control->set_alt_target(0);

    return true;
}

// dfc_run - runs the main dfc-stabilize controller
// should be called at 100hz or more
void Copter::dfc_run()
{
    float target_roll, target_pitch;
    float target_yaw_rate;
    float pilot_throttle_scaled;
    float target_fx, target_fy;

    // if not armed set throttle to zero and exit immediately
    if (!motors->armed() || ap.throttle_zero || !motors->get_interlock()) {
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        attitude_control->set_throttle_out_unstabilized(0,true,g.throttle_filt);
        return;
    }

    // clear landing flag
    set_land_complete(false);

    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();
    
    // SEE if DFC is enabled
    if(g.frame_type >= AP_Motors::MOTOR_FRAME_TYPE_DFC_15)
    {

      target_roll  = 0.0f + 1500.0*(RC_Channels::rc_channel(CH_6)->norm_input()); //S1
      target_pitch = 0.0f + 1500.0*(RC_Channels::rc_channel(CH_7)->norm_input()); //S2
    
      target_fx = (float)(channel_pitch->get_control_in()*-1); // X is backwards -push forward on pitch stick is negative --to generate forward accel-need it to be positive
      target_fy = (float)channel_roll->get_control_in();	  

      motors->set_FX( target_fx/4500.0f ); //scale down to +-1
      motors->set_FY( target_fy/4500.0f );
      
    }else{
      
      get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);	
      //target roll/pitch range +- 4500
      target_fx = 0.0f;
      target_fy = 0.0f;
      
    }

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->get_control_in());

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

    // body-frame rate controller is run directly from 100hz loop

    // output pilot's throttle
    attitude_control->set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);
}
