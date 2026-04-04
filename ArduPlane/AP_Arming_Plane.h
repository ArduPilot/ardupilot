#pragma once

#include <AP_Arming/AP_Arming.h>

#ifndef AP_PLANE_BLACKBOX_LOGGING
#define AP_PLANE_BLACKBOX_LOGGING 0
#endif

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
    CLASS_NO_COPY(AP_Arming_Plane);

    bool pre_arm_checks(bool report) override;
    bool arm_checks(AP_Arming::Method method) override;

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

    bool disarm(AP_Arming::Method method, bool do_disarm_checks=true) override;
    bool arm(AP_Arming::Method method, bool do_arming_checks=true) override;

    void update_soft_armed();
    bool get_delay_arming() const { return delay_arming; };

    // mandatory checks that cannot be bypassed.  This function will only be called if ARMING_SKIPCHK skips all or arming forced
    bool mandatory_checks(bool display_failure) override;

protected:
    bool ins_checks(bool report) override;
    bool terrain_database_required() const override;

    bool quadplane_checks(bool display_failure);
    bool mission_checks(bool report) override;

    // Checks rc has been received if it is configured to be used
    bool rc_received_if_enabled_check(bool display_failure);

private:
    void change_arm_state(void);

    // oneshot with duration AP_ARMING_DELAY_MS used by quadplane to delay spoolup after arming:
    // ignored unless OPTION_DELAY_ARMING or OPTION_TILT_DISARMED is set
    bool delay_arming;

#if AP_PLANE_BLACKBOX_LOGGING
    AP_Float blackbox_speed;
    uint32_t last_over_3dspeed_ms;
#endif
};
