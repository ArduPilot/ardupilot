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

    /* Do not allow copies */
    AP_Arming_Rover(const AP_Arming_Rover &other) = delete;
    AP_Arming_Rover &operator=(const AP_Arming_Rover&) = delete;

    void _pre_arm_checks() override;
    void pre_arm_rc_checks();
    void gps_checks() override;

protected:
    void fence_checks();
    void proximity_check();

private:

};
