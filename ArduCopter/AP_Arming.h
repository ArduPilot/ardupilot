#pragma once

#include <AP_Arming/AP_Arming.h>

class AP_Arming_Copter : public AP_Arming
{
public:
    friend class Copter;
    friend class ToyMode;
    AP_Arming_Copter(const AP_AHRS_NavEKF &ahrs_ref, Compass &compass,
                     const AP_BattMonitor &battery, const AP_InertialNav_NavEKF &inav)
        : AP_Arming(ahrs_ref, compass, battery)
        , _inav(inav)
        , _ahrs_navekf(ahrs_ref)
    {
        // default REQUIRE parameter to 1 (Copter does not have an
        // actual ARMING_REQUIRE parameter)
        require.set_default(YES_MIN_PWM);
    }

    /* Do not allow copies */
    AP_Arming_Copter(const AP_Arming_Copter &other) = delete;
    AP_Arming_Copter &operator=(const AP_Arming_Copter&) = delete;

    void update(void);
    bool all_checks_passing(bool arming_from_gcs);

    bool rc_calibration_checks(bool display_failure) override;

protected:

    bool pre_arm_checks(bool display_failure) override;
    bool pre_arm_ekf_attitude_check();
    bool pre_arm_terrain_check(bool display_failure);
    bool pre_arm_proximity_check(bool display_failure);
    bool arm_checks(bool display_failure, bool arming_from_gcs);

    // NOTE! the following check functions *DO* call into AP_Arming:
    bool ins_checks(bool display_failure) override;
    bool compass_checks(bool display_failure) override;
    bool gps_checks(bool display_failure) override;
    bool barometer_checks(bool display_failure) override;
    bool board_voltage_checks(bool display_failure) override;

    // NOTE! the following check functions *DO NOT* call into AP_Arming!
    bool fence_checks(bool display_failure);
    bool parameter_checks(bool display_failure);
    bool motor_checks(bool display_failure);
    bool pilot_throttle_checks(bool display_failure);

    void set_pre_arm_check(bool b);

private:

    const AP_InertialNav_NavEKF &_inav;
    const AP_AHRS_NavEKF &_ahrs_navekf;

    void parameter_checks_pid_warning_message(bool display_failure, const char *error_msg);
};
