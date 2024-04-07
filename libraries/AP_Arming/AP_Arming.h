#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/Semaphores.h>
#include <AP_Param/AP_Param.h>
#include <AP_BoardConfig/AP_BoardConfig_config.h>

#include "AP_Arming_config.h"
#include "AP_InertialSensor/AP_InertialSensor_config.h"
#include "AP_Proximity/AP_Proximity_config.h"

class AP_Arming {
public:

    AP_Arming();

    CLASS_NO_COPY(AP_Arming);  /* Do not allow copies */

    static AP_Arming *get_singleton();

    void update();

    enum ArmingChecks {
        ARMING_CHECK_ALL         = (1U << 0),
        ARMING_CHECK_BARO        = (1U << 1),
        ARMING_CHECK_COMPASS     = (1U << 2),
        ARMING_CHECK_GPS         = (1U << 3),
        ARMING_CHECK_INS         = (1U << 4),
        ARMING_CHECK_PARAMETERS  = (1U << 5),
        ARMING_CHECK_RC          = (1U << 6),
        ARMING_CHECK_VOLTAGE     = (1U << 7),
        ARMING_CHECK_BATTERY     = (1U << 8),
        ARMING_CHECK_AIRSPEED    = (1U << 9),
        ARMING_CHECK_LOGGING     = (1U << 10),
        ARMING_CHECK_SWITCH      = (1U << 11),
        ARMING_CHECK_GPS_CONFIG  = (1U << 12),
        ARMING_CHECK_SYSTEM      = (1U << 13),
        ARMING_CHECK_MISSION     = (1U << 14),
        ARMING_CHECK_RANGEFINDER = (1U << 15),
        ARMING_CHECK_CAMERA      = (1U << 16),
        ARMING_CHECK_AUX_AUTH    = (1U << 17),
        ARMING_CHECK_VISION      = (1U << 18),
        ARMING_CHECK_FFT         = (1U << 19),
        ARMING_CHECK_OSD         = (1U << 20),
    };

    enum class Method {
        RUDDER = 0,
        MAVLINK = 1,
        AUXSWITCH = 2,
        MOTORTEST = 3,
        SCRIPTING = 4,
        TERMINATION = 5, // only disarm uses this...
        CPUFAILSAFE = 6, // only disarm uses this...
        BATTERYFAILSAFE = 7, // only disarm uses this...
        SOLOPAUSEWHENLANDED = 8, // only disarm uses this...
        AFS = 9, // only disarm uses this...
        ADSBCOLLISIONACTION = 10, // only disarm uses this...
        PARACHUTE_RELEASE = 11, // only disarm uses this...
        CRASH = 12, // only disarm uses this...
        LANDED = 13, // only disarm uses this...
        MISSIONEXIT = 14, // only disarm uses this...
        FENCEBREACH = 15, // only disarm uses this...
        RADIOFAILSAFE = 16, // only disarm uses this...
        DISARMDELAY = 17, // only disarm uses this...
        GCSFAILSAFE = 18, // only disarm uses this...
        TERRRAINFAILSAFE = 19, // only disarm uses this...
        FAILSAFE_ACTION_TERMINATE = 20, // only disarm uses this...
        TERRAINFAILSAFE = 21, // only disarm uses this...
        MOTORDETECTDONE = 22, // only disarm uses this...
        BADFLOWOFCONTROL = 23, // only disarm uses this...
        EKFFAILSAFE = 24, // only disarm uses this...
        GCS_FAILSAFE_SURFACEFAILED = 25, // only disarm uses this...
        GCS_FAILSAFE_HOLDFAILED = 26, // only disarm uses this...
        TAKEOFFTIMEOUT = 27, // only disarm uses this...
        AUTOLANDED = 28, // only disarm uses this...
        PILOT_INPUT_FAILSAFE = 29, // only disarm uses this...
        TOYMODELANDTHROTTLE = 30, // only disarm uses this...
        TOYMODELANDFORCE = 31, // only disarm uses this...
        LANDING = 32, // only disarm uses this...
        DEADRECKON_FAILSAFE = 33, // only disarm uses this...
        BLACKBOX = 34,
        DDS = 35,
        UNKNOWN = 100,
    };

    enum class Required {
        NO           = 0,
        YES_MIN_PWM  = 1,
        YES_ZERO_PWM = 2
    };

    void init(void);

    // these functions should not be used by Copter which holds the armed state in the motors library
    Required arming_required() const;
    virtual bool arm(AP_Arming::Method method, bool do_arming_checks=true);
    virtual bool disarm(AP_Arming::Method method, bool do_disarm_checks=true);
    bool is_armed() const;
    bool is_armed_and_safety_off() const;

    // get bitmask of enabled checks
    uint32_t get_enabled_checks() const;

    // pre_arm_checks() is virtual so it can be modified in a vehicle specific subclass
    virtual bool pre_arm_checks(bool report);

    // some arming checks have side-effects, or require some form of state
    // change to have occurred, and thus should not be done as pre-arm
    // checks.  Those go here:
    virtual bool arm_checks(AP_Arming::Method method);

    // get expected magnetic field strength
    uint16_t compass_magfield_expected() const;

    // rudder arming support
    enum class RudderArming {
        IS_DISABLED  = 0, // DISABLED leaks in from vehicle defines.h
        ARMONLY   = 1,
        ARMDISARM = 2
    };

    RudderArming get_rudder_arming_type() const { return (RudderArming)_rudder_arming.get(); }

#if AP_ARMING_AUX_AUTH_ENABLED
    // auxiliary authorisation methods
    bool get_aux_auth_id(uint8_t& auth_id);
    void set_aux_auth_passed(uint8_t auth_id);
    void set_aux_auth_failed(uint8_t auth_id, const char* fail_msg);
#endif

    static const struct AP_Param::GroupInfo        var_info[];

    // method that was last used for disarm; invalid unless the
    // vehicle has been disarmed at least once.
    Method last_disarm_method() const { return _last_disarm_method; }

    // method that was last used for arm; invalid unless the
    // vehicle has been disarmed at least once.
    Method last_arm_method() const { return _last_arm_method; }
    
    // enum for ARMING_OPTIONS parameter
    enum class Option : int32_t {
        DISABLE_PREARM_DISPLAY             = (1U << 0),
        DISABLE_STATUSTEXT_ON_STATE_CHANGE = (1U << 1),
    };
    bool option_enabled(Option option) const {
        return (_arming_options & uint32_t(option)) != 0;
    }

    void send_arm_disarm_statustext(const char *string) const;

    static bool method_is_GCS(Method method) {
        return (method == Method::MAVLINK || method == Method::DDS);
    }
protected:

    // Parameters
    AP_Enum<Required>       require;
    AP_Int32                checks_to_perform;      // bitmask for which checks are required
    AP_Float                accel_error_threshold;
    AP_Int8                 _rudder_arming;
    AP_Int32                _required_mission_items;
    AP_Int32                _arming_options;
    AP_Int16                magfield_error_threshold;

    // internal members
    bool                    armed;
    uint32_t                last_accel_pass_ms;
    uint32_t                last_gyro_pass_ms;

    virtual bool barometer_checks(bool report);

    bool airspeed_checks(bool report);

    bool logging_checks(bool report);

#if AP_INERTIALSENSOR_ENABLED
    virtual bool ins_checks(bool report);
#endif

    bool compass_checks(bool report);

    virtual bool gps_checks(bool report);

    bool battery_checks(bool report);

    bool hardware_safety_check(bool report);

    virtual bool board_voltage_checks(bool report);

    virtual bool rc_calibration_checks(bool report);

    bool rc_in_calibration_check(bool report);

    bool rc_arm_checks(AP_Arming::Method method);

    bool manual_transmitter_checks(bool report);

    virtual bool mission_checks(bool report);

    bool terrain_checks(bool report) const;

    // expected to return true if the terrain database is required to have
    // all data loaded
    virtual bool terrain_database_required() const;

    bool rangefinder_checks(bool report);

    bool fence_checks(bool report);

#if HAL_HAVE_IMU_HEATER
    bool heater_min_temperature_checks(bool report);
#endif

    bool camera_checks(bool display_failure);

    bool osd_checks(bool display_failure) const;

    bool mount_checks(bool display_failure) const;

#if AP_ARMING_AUX_AUTH_ENABLED
    bool aux_auth_checks(bool display_failure);
#endif

    bool generator_checks(bool report) const;

    bool opendroneid_checks(bool display_failure);
    
    bool serial_protocol_checks(bool display_failure);
    
    bool estop_checks(bool display_failure);

#if AP_ARMING_CRASHDUMP_ACK_ENABLED
    bool crashdump_checks(bool report);
#endif

    virtual bool system_checks(bool report);

    bool can_checks(bool report);

    bool fettec_checks(bool display_failure) const;

#if HAL_PROXIMITY_ENABLED
    virtual bool proximity_checks(bool report) const;
#endif

    bool servo_checks(bool report) const;
    bool rc_checks_copter_sub(bool display_failure, const class RC_Channel *channels[4]) const;

    bool visodom_checks(bool report) const;
    bool disarm_switch_checks(bool report) const;

    // mandatory checks that cannot be bypassed.  This function will only be called if ARMING_CHECK is zero or arming forced
    virtual bool mandatory_checks(bool report);

    // returns true if a particular check is enabled
    bool check_enabled(const enum AP_Arming::ArmingChecks check) const;
    // handle the case where a check fails
    void check_failed(const enum AP_Arming::ArmingChecks check, bool report, const char *fmt, ...) const FMT_PRINTF(4, 5);
    void check_failed(bool report, const char *fmt, ...) const FMT_PRINTF(3, 4);

    void Log_Write_Arm(bool forced, AP_Arming::Method method);
    void Log_Write_Disarm(bool forced, AP_Arming::Method method);

private:

    static AP_Arming *_singleton;

#if AP_INERTIALSENSOR_ENABLED
    bool ins_accels_consistent(const class AP_InertialSensor &ins);
    bool ins_gyros_consistent(const class AP_InertialSensor &ins);
#endif

    // check if we should keep logging after disarming
    void check_forced_logging(const AP_Arming::Method method);

    enum MIS_ITEM_CHECK {
        MIS_ITEM_CHECK_LAND          = (1 << 0),
        MIS_ITEM_CHECK_VTOL_LAND     = (1 << 1),
        MIS_ITEM_CHECK_DO_LAND_START = (1 << 2),
        MIS_ITEM_CHECK_TAKEOFF       = (1 << 3),
        MIS_ITEM_CHECK_VTOL_TAKEOFF  = (1 << 4),
        MIS_ITEM_CHECK_RALLY         = (1 << 5),
        MIS_ITEM_CHECK_RETURN_TO_LAUNCH = (1 << 6),
        MIS_ITEM_CHECK_MAX
    };

#if AP_ARMING_AUX_AUTH_ENABLED
    // auxiliary authorisation
    static const uint8_t aux_auth_count_max = 3;    // maximum number of auxiliary authorisers
    static const uint8_t aux_auth_str_len = 42;     // maximum length of failure message (50-8 for "PreArm: ")
    enum class AuxAuthStates : uint8_t {
        NO_RESPONSE = 0,
        AUTH_FAILED,
        AUTH_PASSED
    } aux_auth_state[aux_auth_count_max] = {};  // state of each auxiliary authorisation
    uint8_t aux_auth_count;     // number of auxiliary authorisers
    uint8_t aux_auth_fail_msg_source;   // authorisation id who set aux_auth_fail_msg
    char* aux_auth_fail_msg;    // buffer for holding failure messages
    bool aux_auth_error;        // true if too many auxiliary authorisers
    HAL_Semaphore aux_auth_sem; // semaphore for accessing the aux_auth_state and aux_auth_fail_msg
#endif

    // method that was last used for arm/disarm; invalid unless the
    // vehicle has been disarmed at least once.
    Method _last_disarm_method = Method::UNKNOWN;
    Method _last_arm_method = Method::UNKNOWN;

    uint32_t last_prearm_display_ms;  // last time we send statustexts for prearm failures
    bool running_arming_checks;  // true if the arming checks currently being performed are being done because the vehicle is trying to arm the vehicle
    
    bool last_prearm_checks_result; // result of last prearm check
    bool report_immediately; // set to true when check goes from true to false, to trigger immediate report

    void update_arm_gpio();

#if AP_ARMING_CRASHDUMP_ACK_ENABLED
    struct CrashDump {
        void check_reset();
        AP_Int8  acked;
    } crashdump_ack;
#endif  // AP_ARMING_CRASHDUMP_ACK_ENABLED

};

namespace AP {
    AP_Arming &arming();
};
