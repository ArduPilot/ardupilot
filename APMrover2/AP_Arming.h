#pragma once

#include <AP_Arming/AP_Arming.h>
#include <AC_Fence/AC_Fence.h>

/*
  a rover-specific arming class
 */
class AP_Arming_Rover : public AP_Arming
{
public:

    AP_Arming_Rover() : AP_Arming() { }

    enum ArmingRudder {
        ARMING_RUDDER_DISABLED  = 0,
        ARMING_RUDDER_ARMONLY   = 1,
        ARMING_RUDDER_ARMDISARM = 2,
        ARMING_RUDDER_NO_DISARM_MANUAL = 3
    };
    
    /* Do not allow copies */
    AP_Arming_Rover(const AP_Arming_Rover &other) = delete;
    AP_Arming_Rover &operator=(const AP_Arming_Rover&) = delete;

    bool pre_arm_checks(bool report) override;
    bool pre_arm_rc_checks(const bool display_failure);
    bool gps_checks(bool display_failure) override;

    //ArmingRudder rudder_arming() const { return (ArmingRudder)rudder_arming_value.get(); }
    ArmingRudder rudder_arming() const { return (ArmingRudder)rudder_arming_value; }

protected:
    bool fence_checks(bool report);
    bool proximity_check(bool report);
    
    // parameters
    //AP_Int8                 rudder_arming_value;
    uint8_t rudder_arming_value = AP_Arming_Rover::ARMING_RUDDER_ARMDISARM;
    
private:


};
