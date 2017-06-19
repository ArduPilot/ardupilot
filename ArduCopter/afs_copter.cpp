/*
  copter specific AP_AdvancedFailsafe class
 */

#include "Copter.h"

#if ADVANCED_FAILSAFE == ENABLED

// Constructor
AP_AdvancedFailsafe_Copter::AP_AdvancedFailsafe_Copter(AP_Mission &_mission, AP_Baro &_baro, const AP_GPS &_gps, const RCMapper &_rcmap) :
    AP_AdvancedFailsafe(_mission, _baro, _gps, _rcmap)
{}


/*
  setup radio_out values for all channels to termination values
 */
void AP_AdvancedFailsafe_Copter::terminate_vehicle(void)
{
    // stop motors
    copter.motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
    copter.motors->output();

    // disarm as well
    copter.init_disarm_motors();
    
    // and set all aux channels
    SRV_Channels::set_output_limit(SRV_Channel::k_heli_rsc, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
    SRV_Channels::set_output_limit(SRV_Channel::k_heli_tail_rsc, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
    SRV_Channels::set_output_limit(SRV_Channel::k_engine_run_enable, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
    SRV_Channels::set_output_limit(SRV_Channel::k_ignition, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
    SRV_Channels::set_output_limit(SRV_Channel::k_none, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
    SRV_Channels::set_output_limit(SRV_Channel::k_manual, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);

    SRV_Channels::output_ch_all();
}

void AP_AdvancedFailsafe_Copter::setup_IO_failsafe(void)
{
    // setup failsafe for all aux channels
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_heli_rsc, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_heli_tail_rsc, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_engine_run_enable, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_ignition, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_none, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_manual, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);

#if FRAME_CONFIG != HELI_FRAME
    // setup AP_Motors outputs for failsafe
    uint16_t mask = copter.motors->get_motor_mask();
    hal.rcout->set_failsafe_pwm(mask, copter.motors->get_pwm_output_min());
#endif
}

/*
  return an AFS_MODE for current control mode
 */
AP_AdvancedFailsafe::control_mode AP_AdvancedFailsafe_Copter::afs_mode(void)
{
    switch (copter.control_mode) {
    case AUTO:
    case GUIDED:
    case RTL:
    case LAND:
        return AP_AdvancedFailsafe::AFS_AUTO;
    default:
        break;
    }
    return AP_AdvancedFailsafe::AFS_STABILIZED;
}

#endif // ADVANCED_FAILSAFE
