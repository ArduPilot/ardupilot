/*
  Rover specific AP_AdvancedFailsafe class
 */

#include "Rover.h"

#if ADVANCED_FAILSAFE == ENABLED

// Constructor
AP_AdvancedFailsafe_Rover::AP_AdvancedFailsafe_Rover(AP_Mission &_mission, AP_Baro &_baro, const AP_GPS &_gps, const RCMapper &_rcmap) :
    AP_AdvancedFailsafe(_mission, _baro, _gps, _rcmap)
{}


/*
  Setup radio_out values for all channels to termination values
 */
void AP_AdvancedFailsafe_Rover::terminate_vehicle(void)
{
    // stop motors
    SRV_Channels::set_output_pwm(SRV_Channel::k_throttle, 0);
    SRV_Channels::set_output_pwm(SRV_Channel::k_steering, 0);
    rover.lateral_acceleration = 0;

    // disarm as well
    rover.disarm_motors();

    // Set to HOLD mode
    rover.set_mode(HOLD);
    // and set all aux channels
    SRV_Channels::set_output_limit(SRV_Channel::k_manual, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
    SRV_Channels::set_output_limit(SRV_Channel::k_none, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);

    SRV_Channels::output_ch_all();
}

void AP_AdvancedFailsafe_Rover::setup_IO_failsafe(void)
{
    // setup failsafe for all aux channels
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_manual, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_none, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
}

/*
  Return an AFS_MODE for current control mode
 */
AP_AdvancedFailsafe::control_mode AP_AdvancedFailsafe_Rover::afs_mode(void)
{
    switch (rover.control_mode) {
    case AUTO:
    case GUIDED:
    case RTL:
        return AP_AdvancedFailsafe::AFS_AUTO;
    default:
        break;
    }
    return AP_AdvancedFailsafe::AFS_STABILIZED;
}

#endif  // ADVANCED_FAILSAFE
