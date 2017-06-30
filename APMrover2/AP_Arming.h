#pragma once

#include <AP_Arming/AP_Arming.h>

/*
  a rover-specific arming class
 */
class AP_Arming_Rover : public AP_Arming
{
public:

    AP_Arming_Rover(const AP_AHRS &ahrs_ref, const AP_Baro &baro, Compass &compass,
                    const AP_BattMonitor &battery) :
        AP_Arming(ahrs_ref, baro, compass, battery) {
    }

    bool pre_arm_rc_checks(const bool display_failure);

protected:

    enum HomeState home_status() const override;
};

