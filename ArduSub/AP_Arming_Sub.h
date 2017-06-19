#pragma once

#include <AP_Arming/AP_Arming.h>

class AP_Arming_Sub : public AP_Arming {
public:
    AP_Arming_Sub(const AP_AHRS &ahrs_ref, const AP_Baro &baro, Compass &compass,
                    const AP_BattMonitor &battery) :
        AP_Arming(ahrs_ref, baro, compass, battery) {
            AP_Param::setup_object_defaults(this, var_info);
    }

    bool rc_check(bool report=true);
    bool pre_arm_checks(bool report) override;

protected:
    enum HomeState home_status() const override;
};
