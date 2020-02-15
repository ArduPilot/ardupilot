/*
  plane specific AP_AdvancedFailsafe class
 */

#include "Plane.h"

#if ADVANCED_FAILSAFE == ENABLED
// Constructor
AP_AdvancedFailsafe_Plane::AP_AdvancedFailsafe_Plane(AP_Mission &_mission) :
    AP_AdvancedFailsafe(_mission)
{}


/*
  setup radio_out values for all channels to termination values
 */
void AP_AdvancedFailsafe_Plane::terminate_vehicle(void)
{
    if (plane.quadplane.available() && _terminate_action == TERMINATE_ACTION_LAND) {
        // perform a VTOL landing
        plane.set_mode(plane.mode_qland, ModeReason::FENCE_BREACHED);
        return;
    }

    plane.g2.servo_channels.disable_passthrough(true);
    
    if (_terminate_action == TERMINATE_ACTION_LAND) {
        plane.landing.terminate();
    } else {
        // aerodynamic termination is the default approach to termination
        SRV_Channels::set_output_scaled(SRV_Channel::k_flap_auto, 100);
        SRV_Channels::set_output_scaled(SRV_Channel::k_flap, 100);
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, SERVO_MAX);
        SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, SERVO_MAX);
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, SERVO_MAX);
        if (plane.have_reverse_thrust()) {
            // configured for reverse thrust, use TRIM
            SRV_Channels::set_output_limit(SRV_Channel::k_throttle, SRV_Channel::Limit::TRIM);
            SRV_Channels::set_output_limit(SRV_Channel::k_throttleLeft, SRV_Channel::Limit::TRIM);
            SRV_Channels::set_output_limit(SRV_Channel::k_throttleRight, SRV_Channel::Limit::TRIM);
        } else {
            // use MIN
            SRV_Channels::set_output_limit(SRV_Channel::k_throttle, SRV_Channel::Limit::MIN);
            SRV_Channels::set_output_limit(SRV_Channel::k_throttleLeft, SRV_Channel::Limit::MIN);
            SRV_Channels::set_output_limit(SRV_Channel::k_throttleRight, SRV_Channel::Limit::MIN);
        }
        SRV_Channels::set_output_limit(SRV_Channel::k_manual, SRV_Channel::Limit::TRIM);
        SRV_Channels::set_output_limit(SRV_Channel::k_none, SRV_Channel::Limit::TRIM);
    }

    plane.servos_output();

    plane.quadplane.afs_terminate();
    
    // also disarm to ensure that ignition is cut
    plane.arming.disarm();
}

void AP_AdvancedFailsafe_Plane::setup_IO_failsafe(void)
{
    // all aux channels
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_flap_auto, SRV_Channel::Limit::MAX);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_flap, SRV_Channel::Limit::MAX);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_aileron, SRV_Channel::Limit::MIN);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_rudder, SRV_Channel::Limit::MAX);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_elevator, SRV_Channel::Limit::MAX);
    if (plane.have_reverse_thrust()) {
        // configured for reverse thrust, use TRIM
        SRV_Channels::set_failsafe_limit(SRV_Channel::k_throttle, SRV_Channel::Limit::TRIM);
        SRV_Channels::set_failsafe_limit(SRV_Channel::k_throttleLeft, SRV_Channel::Limit::TRIM);
        SRV_Channels::set_failsafe_limit(SRV_Channel::k_throttleRight, SRV_Channel::Limit::TRIM);
    } else {
        // normal throttle, use MIN
        SRV_Channels::set_failsafe_limit(SRV_Channel::k_throttle, SRV_Channel::Limit::MIN);
        SRV_Channels::set_failsafe_limit(SRV_Channel::k_throttleLeft, SRV_Channel::Limit::MIN);
        SRV_Channels::set_failsafe_limit(SRV_Channel::k_throttleRight, SRV_Channel::Limit::MIN);
    }
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_manual, SRV_Channel::Limit::TRIM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_none, SRV_Channel::Limit::TRIM);

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
    if (plane.control_mode == &plane.mode_manual) {
        return AP_AdvancedFailsafe::AFS_MANUAL;
    }
    return AP_AdvancedFailsafe::AFS_STABILIZED;
}
#endif // ADVANCED_FAILSAFE
