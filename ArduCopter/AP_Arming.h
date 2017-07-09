#pragma once

#include <AP_Arming/AP_Arming.h>

class AP_Arming_Copter : public AP_Arming
{
public:
    AP_Arming_Copter(const AP_AHRS_NavEKF &ahrs_ref, const AP_Baro &baro, Compass &compass,
                     const AP_BattMonitor &battery, const AP_InertialNav_NavEKF &inav,
                     const AP_InertialSensor &ins) :
        AP_Arming(ahrs_ref, baro, compass, battery),
        _inav(inav),
        _ins(ins),
        _ahrs_navekf(ahrs_ref)
        {
    }

    void update(void);
    bool all_checks_passing(bool arming_from_gcs);
    void pre_arm_rc_checks(bool display_failure);

protected:

    bool pre_arm_checks(bool display_failure) override;
    bool pre_arm_gps_checks(bool display_failure);
    bool pre_arm_ekf_attitude_check();
    bool pre_arm_terrain_check(bool display_failure);
    bool pre_arm_proximity_check(bool display_failure);
    bool arm_checks(bool display_failure, bool arming_from_gcs);

    // NOTE! the following check functions *DO* call into AP_Arming:
    bool ins_checks(bool display_failure) override;
    bool compass_checks(bool display_failure) override;
    bool gps_checks(bool display_failure) override;

    // NOTE! the following check functions *DO NOT* call into AP_Arming!
    bool fence_checks(bool display_failure);
    bool board_voltage_checks(bool display_failure);
    bool parameter_checks(bool display_failure);
    bool motor_checks(bool display_failure);
    bool pilot_throttle_checks(bool display_failure);
    bool barometer_checks(bool display_failure);
    bool rc_calibration_checks(bool display_failure);

    void set_pre_arm_check(bool b);
    void set_pre_arm_rc_check(bool b);

    enum HomeState home_status() const override;

private:

    const AP_InertialNav_NavEKF &_inav;
    const AP_InertialSensor &_ins;
    const AP_AHRS_NavEKF &_ahrs_navekf;

};
