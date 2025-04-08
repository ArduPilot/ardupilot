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
    CLASS_NO_COPY(AP_Arming_Rover);

    bool pre_arm_checks(bool report) override;
    bool arm_checks(AP_Arming::Method method) override;
    bool rc_calibration_checks(const bool display_failure) override;
    bool gps_checks(bool display_failure) override;

    bool disarm(AP_Arming::Method method, bool do_disarm_checks=true) override;
    bool arm(AP_Arming::Method method, bool do_arming_checks=true) override;

    void update_soft_armed();

protected:
    // the following check functions do not call into AP_Arming
    bool oa_check(bool report);
    bool parameter_checks(bool report);
    bool mode_checks(bool report);
    bool motor_checks(bool report);

};
