// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
  additional arming checks for copter
 */

#ifndef __ARMING_CHECKS_H__
#define __ARMING_CHECKS_H__

#include <AP_Arming/AP_Arming.h>
#include <AP_InertialNav/AP_InertialNav.h>
#include <RC_Channel/RC_Channel.h>

#define COPTER_ARMING_CHECK_ALT_DISPARITY_MAX_CM    100

/*
  copter specific arming class
 */

class AP_Arming_Copter : public AP_Arming
{
public:
    AP_Arming_Copter(const AP_AHRS &ahrs_ref, const AP_Baro &baro, Compass &compass, const AP_BattMonitor &battery, const enum HomeState &home_state) :
        AP_Arming(ahrs_ref, baro, compass, battery, home_state)
    {
            AP_Param::setup_object_defaults(this, var_info);
    }

    void update_arming_checks();

    bool all_arming_checks_passing(bool arming_from_gcs);

    bool pre_arm_checks(bool report);

    bool arm_checks(bool report, bool arming_from_gcs);

    ArmingCheckResult barometer_checks(bool report, uint64_t &enabled_checks, uint64_t &passed_checks);

    ArmingCheckResult ins_checks(bool report, uint64_t &enabled_checks, uint64_t &passed_checks);

    ArmingCheckResult parameter_checks(bool report, uint64_t &enabled_checks, uint64_t &passed_checks);

    ArmingCheckResult compass_checks(bool report, uint64_t &enabled_checks, uint64_t &passed_checks);

    ArmingCheckResult gps_checks(bool report, uint64_t &enabled_checks, uint64_t &passed_checks);

    ArmingCheckResult manual_transmitter_checks(bool report, uint64_t &enabled_checks, uint64_t &passed_checks);

    ArmingCheckResult battery_checks(bool report, uint64_t &enabled_checks, uint64_t &passed_checks);

    ArmingCheckResult rangefinder_optflow_checks(bool report, uint64_t &enabled_checks, uint64_t &passed_checks);
    
    ArmingCheckResult terrain_checks(bool report, uint64_t &enabled_checks, uint64_t &passed_checks);
    
    ArmingCheckResult rallypoint_checks(bool report, uint64_t &enabled_checks, uint64_t &passed_checks);

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];
};

#endif //  __ARMING_CHECKS_H__
