// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
  plane specific AP_AdvancedFailsafe class
 */

#include "Plane.h"

// Constructor
AP_AdvancedFailsafe_Plane::AP_AdvancedFailsafe_Plane(AP_Mission &_mission, AP_Baro &_baro, const AP_GPS &_gps, const RCMapper &_rcmap) :
    AP_AdvancedFailsafe(_mission, _baro, _gps, _rcmap)
{}


/*
  setup radio_out values for all channels to termination values
 */
void AP_AdvancedFailsafe_Plane::terminate_vehicle(void)
{
    // we are terminating. Setup primary output channels radio_out values
    RC_Channel *ch_roll     = RC_Channel::rc_channel(rcmap.roll()-1);
    RC_Channel *ch_pitch    = RC_Channel::rc_channel(rcmap.pitch()-1);
    RC_Channel *ch_yaw      = RC_Channel::rc_channel(rcmap.yaw()-1);
    RC_Channel *ch_throttle = RC_Channel::rc_channel(rcmap.throttle()-1);

    ch_roll->set_radio_out(ch_roll->get_limit_pwm(RC_Channel::RC_CHANNEL_LIMIT_MIN));
    ch_pitch->set_radio_out(ch_pitch->get_limit_pwm(RC_Channel::RC_CHANNEL_LIMIT_MAX));
    ch_yaw->set_radio_out(ch_yaw->get_limit_pwm(RC_Channel::RC_CHANNEL_LIMIT_MAX));
    ch_throttle->set_radio_out(ch_throttle->get_limit_pwm(RC_Channel::RC_CHANNEL_LIMIT_MIN));

    // and all aux channels
    RC_Channel_aux::set_servo_limit(RC_Channel_aux::k_flap_auto, RC_Channel::RC_CHANNEL_LIMIT_MAX);
    RC_Channel_aux::set_servo_limit(RC_Channel_aux::k_flap, RC_Channel::RC_CHANNEL_LIMIT_MAX);
    RC_Channel_aux::set_servo_limit(RC_Channel_aux::k_aileron, RC_Channel::RC_CHANNEL_LIMIT_MIN);
    RC_Channel_aux::set_servo_limit(RC_Channel_aux::k_rudder, RC_Channel::RC_CHANNEL_LIMIT_MAX);
    RC_Channel_aux::set_servo_limit(RC_Channel_aux::k_elevator, RC_Channel::RC_CHANNEL_LIMIT_MAX);
    RC_Channel_aux::set_servo_limit(RC_Channel_aux::k_elevator_with_input, RC_Channel::RC_CHANNEL_LIMIT_MAX);
    RC_Channel_aux::set_servo_limit(RC_Channel_aux::k_manual, RC_Channel::RC_CHANNEL_LIMIT_TRIM);
    RC_Channel_aux::set_servo_limit(RC_Channel_aux::k_none, RC_Channel::RC_CHANNEL_LIMIT_TRIM);

    ch_roll->output();
    ch_pitch->output();
    ch_yaw->output();
    ch_throttle->output();
    RC_Channel_aux::output_ch_all();

    plane.quadplane.afs_terminate();
    
    // also disarm to ensure that ignition is cut
    plane.disarm_motors();
}

void AP_AdvancedFailsafe_Plane::setup_IO_failsafe(void)
{
    const RC_Channel *ch_roll     = RC_Channel::rc_channel(rcmap.roll()-1);
    const RC_Channel *ch_pitch    = RC_Channel::rc_channel(rcmap.pitch()-1);
    const RC_Channel *ch_yaw      = RC_Channel::rc_channel(rcmap.yaw()-1);
    const RC_Channel *ch_throttle = RC_Channel::rc_channel(rcmap.throttle()-1);

    // setup primary channel output values
    hal.rcout->set_failsafe_pwm(1U<<(rcmap.roll()-1),     ch_roll->get_limit_pwm(RC_Channel::RC_CHANNEL_LIMIT_MIN));
    hal.rcout->set_failsafe_pwm(1U<<(rcmap.pitch()-1),    ch_pitch->get_limit_pwm(RC_Channel::RC_CHANNEL_LIMIT_MAX));
    hal.rcout->set_failsafe_pwm(1U<<(rcmap.yaw()-1),      ch_yaw->get_limit_pwm(RC_Channel::RC_CHANNEL_LIMIT_MAX));
    hal.rcout->set_failsafe_pwm(1U<<(rcmap.throttle()-1), ch_throttle->get_limit_pwm(RC_Channel::RC_CHANNEL_LIMIT_MIN));

    // and all aux channels
    RC_Channel_aux::set_servo_failsafe(RC_Channel_aux::k_flap_auto, RC_Channel::RC_CHANNEL_LIMIT_MAX);
    RC_Channel_aux::set_servo_failsafe(RC_Channel_aux::k_flap, RC_Channel::RC_CHANNEL_LIMIT_MAX);
    RC_Channel_aux::set_servo_failsafe(RC_Channel_aux::k_aileron, RC_Channel::RC_CHANNEL_LIMIT_MIN);
    RC_Channel_aux::set_servo_failsafe(RC_Channel_aux::k_rudder, RC_Channel::RC_CHANNEL_LIMIT_MAX);
    RC_Channel_aux::set_servo_failsafe(RC_Channel_aux::k_elevator, RC_Channel::RC_CHANNEL_LIMIT_MAX);
    RC_Channel_aux::set_servo_failsafe(RC_Channel_aux::k_elevator_with_input, RC_Channel::RC_CHANNEL_LIMIT_MAX);
    RC_Channel_aux::set_servo_failsafe(RC_Channel_aux::k_manual, RC_Channel::RC_CHANNEL_LIMIT_TRIM);
    RC_Channel_aux::set_servo_failsafe(RC_Channel_aux::k_none, RC_Channel::RC_CHANNEL_LIMIT_TRIM);

    if (plane.quadplane.available()) {
        // setup AP_Motors outputs for failsafe
        uint16_t mask = plane.quadplane.motors->get_motor_mask();
        hal.rcout->set_failsafe_pwm(mask, plane.quadplane.thr_min_pwm);
    }
}

/*
  return an AFS_MODE for current control mode
 */
AP_AdvancedFailsafe::control_mode AP_AdvancedFailsafe_Plane::afs_mode(void)
{
    if (plane.auto_throttle_mode) {
        return AP_AdvancedFailsafe::AFS_AUTO;
    }
    if (plane.control_mode == MANUAL) {
        return AP_AdvancedFailsafe::AFS_MANUAL;
    }
    return AP_AdvancedFailsafe::AFS_STABILIZED;
}
