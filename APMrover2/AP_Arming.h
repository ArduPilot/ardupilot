#pragma once

#include <AP_Arming/AP_Arming.h>

/*
  a rover-specific arming class
 */
class AP_Arming_Rover : public AP_Arming
{
public:
    static AP_Arming_Rover create(const AP_AHRS &ahrs_ref, const AP_Baro &baro, Compass &compass, const AP_BattMonitor &battery) {
        return AP_Arming_Rover{ahrs_ref, baro, compass, battery};
    }

    constexpr AP_Arming_Rover(AP_Arming_Rover &&other) = default;

    /* Do not allow copies */
    AP_Arming_Rover(const AP_Arming_Rover &other) = delete;
    AP_Arming_Rover &operator=(const AP_Baro&) = delete;

    bool pre_arm_checks(bool report) override;
    bool pre_arm_rc_checks(const bool display_failure);
    bool gps_checks(bool display_failure) override;

protected:
    AP_Arming_Rover(const AP_AHRS &ahrs_ref, const AP_Baro &baro, Compass &compass,
                    const AP_BattMonitor &battery)
        : AP_Arming(ahrs_ref, baro, compass, battery)
    {
    }

    enum HomeState home_status() const override;
};
