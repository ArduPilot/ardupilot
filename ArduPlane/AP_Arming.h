#pragma once

#include <AP_Arming/AP_Arming.h>

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

    enum ArmingRudder {
        ARMING_RUDDER_DISABLED  = 0,
        ARMING_RUDDER_ARMONLY   = 1,
        ARMING_RUDDER_ARMDISARM = 2
    };

    /* Do not allow copies */
    AP_Arming_Plane(const AP_Arming_Plane &other) = delete;
    AP_Arming_Plane &operator=(const AP_Arming_Plane&) = delete;

    bool pre_arm_checks(bool report);

    ArmingRudder rudder_arming() const { return (ArmingRudder)rudder_arming_value.get(); }

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

protected:
    bool ins_checks(bool report);

    // parameters
    AP_Int8                 rudder_arming_value;
};
