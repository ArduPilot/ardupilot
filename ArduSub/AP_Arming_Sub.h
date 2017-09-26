#pragma once

#include <AP_Arming/AP_Arming.h>

class AP_Arming_Sub : public AP_Arming {
public:
    static AP_Arming_Sub create(const AP_AHRS &ahrs_ref, const AP_Baro &baro, Compass &compass, const AP_BattMonitor &battery) {
        return AP_Arming_Sub{ahrs_ref, baro, compass, battery};
    }

    constexpr AP_Arming_Sub(AP_Arming_Sub &&other) = default;

    /* Do not allow copies */
    AP_Arming_Sub(const AP_Arming_Sub &other) = delete;
    AP_Arming_Sub &operator=(const AP_Baro&) = delete;

    bool rc_check(bool report=true);
    bool pre_arm_checks(bool report) override;

protected:
    AP_Arming_Sub(const AP_AHRS &ahrs_ref, const AP_Baro &baro, Compass &compass,
                  const AP_BattMonitor &battery)
        : AP_Arming(ahrs_ref, baro, compass, battery)
    {
    }

    bool ins_checks(bool report) override;
    enum HomeState home_status() const override;
};
