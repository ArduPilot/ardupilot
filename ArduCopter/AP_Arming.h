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
        require.set_default((uint8_t)Required::YES_MIN_PWM);
    }

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Arming_Copter);

    bool rc_calibration_checks(bool display_failure) override;

    bool disarm(AP_Arming::Method method, bool do_disarm_checks=true) override;
    bool arm(AP_Arming::Method method, bool do_arming_checks=true) override;

protected:

    bool pre_arm_checks(bool display_failure) override;
    bool pre_arm_ekf_attitude_check();
    bool proximity_checks(bool display_failure) const override;
    bool arm_checks(AP_Arming::Method method) override;

    // mandatory checks that cannot be bypassed.  This function will only be called if ARMING_CHECK is zero or arming forced
    bool mandatory_checks(bool display_failure) override;

    // NOTE! the following check functions *DO* call into AP_Arming:
    bool ins_checks(bool display_failure) override;
    bool gps_checks(bool display_failure) override;
    bool barometer_checks(bool display_failure) override;
    bool board_voltage_checks(bool display_failure) override;

    // NOTE! the following check functions *DO NOT* call into AP_Arming!
    bool parameter_checks(bool display_failure);
    bool oa_checks(bool display_failure);
    bool mandatory_gps_checks(bool display_failure);
    bool gcs_failsafe_check(bool display_failure);
    bool winch_checks(bool display_failure) const;
    bool alt_checks(bool display_failure);
    bool rc_throttle_failsafe_checks(bool display_failure) const;

    void set_pre_arm_check(bool b);

    // expected to return true if the terrain database is required to have
    // all data loaded
    bool terrain_database_required() const override;

private:

    // actually contains the pre-arm checks.  This is wrapped so that
    // we can store away success/failure of the checks.
    bool run_pre_arm_checks(bool display_failure);

};
