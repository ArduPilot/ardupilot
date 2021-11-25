#pragma once

#include <AP_Arming/AP_Arming.h>
#include "mode.h"

/*
  a plane specific arming class
 */
class AP_Arming_Plane : public AP_Arming
{
public:
    AP_Arming_Plane()
        : AP_Arming()
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    /* Do not allow copies */
    AP_Arming_Plane(const AP_Arming_Plane &other) = delete;
    AP_Arming_Plane &operator=(const AP_Arming_Plane&) = delete;

    bool pre_arm_checks(bool report) override;
    bool arm_checks(AP_Arming::Method method) override;

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

    void disarm_if_requested();
    bool disarm(AP_Arming::Method method, bool do_disarm_checks=true) override;
    bool arm(AP_Arming::Method method, bool do_arming_checks=true) override;

    void update_soft_armed();
    bool get_delay_arming() const { return delay_arming; };
    bool get_throttle_cut() const { return throttle_cut; };

protected:
    bool ins_checks(bool report) override;

    bool quadplane_checks(bool display_failure);

private:
    void change_arm_state(void);

    // throttle cut when trying to disarm but the plane is still flying
    bool throttle_cut = false;

    // mode the plane was in when throttle cut was enabled
    Mode *throttle_cut_prev_mode;

    // oneshot with duration AP_ARMING_DELAY_MS used by quadplane to delay spoolup after arming:
    // ignored unless OPTION_DELAY_ARMING or OPTION_TILT_DISARMED is set
    bool delay_arming;
};
