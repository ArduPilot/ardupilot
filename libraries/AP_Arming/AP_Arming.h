#pragma once

#include <AP_AHRS/AP_AHRS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <RC_Channel/RC_Channel.h>

class AP_Arming {
public:

    AP_Arming();

    /* Do not allow copies */
    AP_Arming(const AP_Arming &other) = delete;
    AP_Arming &operator=(const AP_Arming&) = delete;

    static AP_Arming &get_singleton();

    enum ArmingChecks {
        ARMING_CHECK_NONE       = 0x0000,
        ARMING_CHECK_ALL        = 0x0001,
        ARMING_CHECK_BARO       = 0x0002,
        ARMING_CHECK_COMPASS    = 0x0004,
        ARMING_CHECK_GPS        = 0x0008,
        ARMING_CHECK_INS        = 0x0010,
        ARMING_CHECK_PARAMETERS = 0x0020,
        ARMING_CHECK_RC         = 0x0040,
        ARMING_CHECK_VOLTAGE    = 0x0080,
        ARMING_CHECK_BATTERY    = 0x0100,
        ARMING_CHECK_AIRSPEED   = 0x0200,
        ARMING_CHECK_LOGGING    = 0x0400,
        ARMING_CHECK_SWITCH     = 0x0800,
        ARMING_CHECK_GPS_CONFIG = 0x1000,
        ARMING_CHECK_SYSTEM     = 0x2000,
    };

    enum ArmingMethod {
        RUDDER,
        MAVLINK,
        AUXSWITCH,
        MOTORTEST,
    };

    enum ArmingRequired {
        NO           = 0,
        YES_MIN_PWM  = 1,
        YES_ZERO_PWM = 2
    };

    // these functions should not be used by Copter which holds the armed state in the motors library
    ArmingRequired arming_required();
    virtual bool arm(ArmingMethod method, bool do_arming_checks=true);
    bool disarm();
    bool is_armed();

    // get bitmask of enabled checks
    uint16_t get_enabled_checks();

    // pre_arm_checks() is virtual so it can be modified in a vehicle specific subclass
    virtual void pre_arm_checks(bool report);
    virtual void _pre_arm_checks();
    bool check_failing(const ArmingChecks check) const;
    bool all_enabled_checks_passing() const;
    bool ok_to_fly(ArmingMethod method);
    bool ok_to_use_rc();

    // some arming checks have side-effects, or require some form of state
    // change to have occurred, and thus should not be done as pre-arm
    // checks.  Those go here:
    virtual void arm_checks(ArmingMethod method);

    // get expected magnetic field strength
    uint16_t compass_magfield_expected() const;

    // rudder arming support
    enum ArmingRudder {
        ARMING_RUDDER_DISABLED  = 0,
        ARMING_RUDDER_ARMONLY   = 1,
        ARMING_RUDDER_ARMDISARM = 2
    };
    ArmingRudder get_rudder_arming_type() const { return (ArmingRudder)_rudder_arming.get(); }

    static const struct AP_Param::GroupInfo        var_info[];

    // handle the case where a check fails
    void check_failed(const enum AP_Arming::ArmingChecks check, const char *fmt, ...);

protected:

    static AP_Arming *_singleton;

    // Parameters
    AP_Int8                 require;
    AP_Int16                checks_to_perform;      // bitmask for which checks are required
    AP_Float                accel_error_threshold;
    AP_Int8                 _rudder_arming;

    // internal members
    bool                    armed:1;
    bool                    logging_available:1;
    uint32_t                last_accel_pass_ms[INS_MAX_INSTANCES];
    uint32_t                last_gyro_pass_ms[INS_MAX_INSTANCES];

    virtual void barometer_checks();

    void airspeed_checks();

    void logging_checks();

    virtual void ins_checks();

    virtual void compass_checks();

    virtual void gps_checks();

    void battery_checks();

    void hardware_safety_check();

    virtual void board_voltage_checks();

    virtual void rc_calibration_checks();

    void manual_transmitter_checks();

    virtual void system_checks();

    void servo_checks();
    void rc_checks_copter_sub(const RC_Channel *channels[4],
                              const bool check_min_max = true);

    // returns true if a particular check is enabled
    bool check_enabled(const enum AP_Arming::ArmingChecks check) const;
    // returns a mavlink severity which should be used if a specific check fails
    MAV_SEVERITY check_severity(const enum AP_Arming::ArmingChecks check) const;
    uint32_t failing_checks; // bitmask of failing checks
    bool report_failing_checks : 1; // set to true to report failures to GCS
    bool arming_check_none_failed : 1; // true if a check unassociated with a bit fails

private:

    bool ins_accels_consistent(const AP_InertialSensor &ins);
    bool ins_gyros_consistent(const AP_InertialSensor &ins);

};

namespace AP {
    AP_Arming &arming();
};
