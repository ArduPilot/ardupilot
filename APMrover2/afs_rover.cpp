// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
  rover specific AP_AdvancedFailsafe class
 */

#include "Rover.h"

#if ADVANCED_FAILSAFE == ENABLED

// Constructor
AP_AdvancedFailsafe_Rover::AP_AdvancedFailsafe_Rover(AP_Mission &_mission, AP_Baro &_baro, const AP_GPS &_gps, const RCMapper &_rcmap) :
    AP_AdvancedFailsafe(_mission, _baro, _gps, _rcmap)
{}


/*
  setup radio_out values for all channels to termination values
 */
void AP_AdvancedFailsafe_Rover::terminate_vehicle(void)
{
    // stop motors
    rover.channel_throttle->set_servo_out(0);
    rover.channel_steer->set_servo_out(0);
    rover.lateral_acceleration = 0;

    // disarm as well
    rover.disarm_motors();
    
    // Set to HOLD mode 
    rover.set_mode(HOLD);
    // and set all aux channels
    RC_Channel_aux::set_servo_limit(RC_Channel_aux::k_none, RC_Channel::RC_CHANNEL_LIMIT_TRIM);
    RC_Channel_aux::set_servo_limit(RC_Channel_aux::k_manual, RC_Channel::RC_CHANNEL_LIMIT_TRIM);

    RC_Channel_aux::output_ch_all();
}

void AP_AdvancedFailsafe_Rover::setup_IO_failsafe(void)
{
    // setup failsafe for all aux channels
    RC_Channel_aux::set_servo_failsafe(RC_Channel_aux::k_none, RC_Channel::RC_CHANNEL_LIMIT_TRIM);
    RC_Channel_aux::set_servo_failsafe(RC_Channel_aux::k_manual, RC_Channel::RC_CHANNEL_LIMIT_TRIM);

}

/*
  return an AFS_MODE for current control mode
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

#endif // ADVANCED_FAILSAFE
