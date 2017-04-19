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
    plane.g2.servo_channels.disable_passthrough(true);
    
    plane.servos_output();

    // and all aux channels
    SRV_Channels::set_output_limit(SRV_Channel::k_flap_auto, SRV_Channel::SRV_CHANNEL_LIMIT_MAX);
    SRV_Channels::set_output_limit(SRV_Channel::k_flap, SRV_Channel::SRV_CHANNEL_LIMIT_MAX);
    SRV_Channels::set_output_limit(SRV_Channel::k_aileron, SRV_Channel::SRV_CHANNEL_LIMIT_MIN);
    SRV_Channels::set_output_limit(SRV_Channel::k_rudder, SRV_Channel::SRV_CHANNEL_LIMIT_MAX);
    SRV_Channels::set_output_limit(SRV_Channel::k_elevator, SRV_Channel::SRV_CHANNEL_LIMIT_MAX);
    SRV_Channels::set_output_limit(SRV_Channel::k_elevator_with_input, SRV_Channel::SRV_CHANNEL_LIMIT_MAX);
    SRV_Channels::set_output_limit(SRV_Channel::k_manual, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
    SRV_Channels::set_output_limit(SRV_Channel::k_none, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);

    plane.quadplane.afs_terminate();
    
    // also disarm to ensure that ignition is cut
    plane.disarm_motors();
}

void AP_AdvancedFailsafe_Plane::setup_IO_failsafe(void)
{
    // all aux channels
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_flap_auto, SRV_Channel::SRV_CHANNEL_LIMIT_MAX);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_flap, SRV_Channel::SRV_CHANNEL_LIMIT_MAX);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_aileron, SRV_Channel::SRV_CHANNEL_LIMIT_MIN);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_rudder, SRV_Channel::SRV_CHANNEL_LIMIT_MAX);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_elevator, SRV_Channel::SRV_CHANNEL_LIMIT_MAX);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_elevator_with_input, SRV_Channel::SRV_CHANNEL_LIMIT_MAX);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_manual, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_none, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);

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
