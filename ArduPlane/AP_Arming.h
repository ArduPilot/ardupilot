#pragma once

#include <AP_Arming/AP_Arming.h>

/*
  a plane specific arming class
 */
class AP_Arming_Plane : public AP_Arming
{
public:
    enum ArmingRudder {
        ARMING_RUDDER_DISABLED  = 0,
        ARMING_RUDDER_ARMONLY   = 1,
        ARMING_RUDDER_ARMDISARM = 2
    };

    static AP_Arming_Plane create(const AP_AHRS &ahrs_ref, const AP_Baro &baro, Compass &compass, const AP_BattMonitor &battery) {
        return AP_Arming_Plane{ahrs_ref, baro, compass, battery};
    }

    constexpr AP_Arming_Plane(AP_Arming_Plane &&other) = default;

    /* Do not allow copies */
    AP_Arming_Plane(const AP_Arming_Plane &other) = delete;
    AP_Arming_Plane &operator=(const AP_Baro&) = delete;

    bool pre_arm_checks(bool report);

    ArmingRudder rudder_arming() const { return (ArmingRudder)rudder_arming_value.get(); }

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

protected:
    AP_Arming_Plane(const AP_AHRS &ahrs_ref, const AP_Baro &baro, Compass &compass,
                    const AP_BattMonitor &battery)
        : AP_Arming(ahrs_ref, baro, compass, battery)
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    bool ins_checks(bool report);
    enum HomeState home_status() const override;

    // parameters
    AP_Int8                 rudder_arming_value;
};
