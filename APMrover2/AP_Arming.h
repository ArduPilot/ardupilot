#pragma once

#include <AP_Arming/AP_Arming.h>
#include <AC_Fence/AC_Fence.h>

/*
  a rover-specific arming class
 */
class AP_Arming_Rover : public AP_Arming
{
public:
    AP_Arming_Rover(const AP_AHRS &ahrs_ref, Compass &compass,
                    const AP_BattMonitor &battery, const AC_Fence &fence)
        : AP_Arming(ahrs_ref, compass, battery),
          _fence(fence)
    {
    }

    /* Do not allow copies */
    AP_Arming_Rover(const AP_Arming_Rover &other) = delete;
    AP_Arming_Rover &operator=(const AP_Arming_Rover&) = delete;

    bool pre_arm_checks(bool report) override;
    bool pre_arm_rc_checks(const bool display_failure);
    bool gps_checks(bool display_failure) override;

protected:
    bool fence_checks(bool report);
    bool proximity_check(bool report);

private:
    const AC_Fence& _fence;
};
