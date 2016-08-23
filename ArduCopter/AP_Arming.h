#pragma once

#include <AP_Arming/AP_Arming.h>

class AP_Arming_Copter : public AP_Arming
{
public:
    friend class Copter;
    friend class ToyMode;

    AP_Arming_Copter() : AP_Arming()
    {
        // default REQUIRE parameter to 1 (Copter does not have an
        // actual ARMING_REQUIRE parameter)
        require.set_default(YES_MIN_PWM);
    }

    /* Do not allow copies */
    AP_Arming_Copter(const AP_Arming_Copter &other) = delete;
    AP_Arming_Copter &operator=(const AP_Arming_Copter&) = delete;

    void update(void);

    void rc_calibration_checks() override;

protected:

    void pre_arm_checks(bool report) override;
    void _pre_arm_checks() override;
    void pre_arm_ekf_attitude_check();
    void pre_arm_terrain_check();
    void pre_arm_proximity_check();
    void arm_checks(AP_Arming::ArmingMethod method) override;

    // NOTE! the following check functions *DO* call into AP_Arming:
    void ins_checks() override;
    void compass_checks() override;
    void gps_checks() override;
    void barometer_checks() override;
    void board_voltage_checks() override;

    // NOTE! the following check functions *DO NOT* call into AP_Arming!
    void fence_checks();
    void parameter_checks();
    void motor_checks();
    void pilot_throttle_checks();

    void set_pre_arm_check(bool b);

private:

    void parameter_checks_pid_warning_message(const char *error_msg);
};
