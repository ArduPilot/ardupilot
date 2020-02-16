#pragma once

#include "Copter.h"
class Parameters;
class ParametersG2;

class GCS_Copter;

class Mode {

public:

    // Auto Pilot Modes enumeration
    enum class Number : uint8_t {
        STABILIZE =     0,  // manual airframe angle with manual throttle
        ACRO =          1,  // manual body-frame angular rate with manual throttle
        ALT_HOLD =      2,  // manual airframe angle with automatic throttle
        AUTO =          3,  // fully automatic waypoint control using mission commands
        GUIDED =        4,  // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
        LOITER =        5,  // automatic horizontal acceleration with automatic throttle
        RTL =           6,  // automatic return to launching point
        CIRCLE =        7,  // automatic circular flight with automatic throttle
        LAND =          9,  // automatic landing with horizontal position control
        DRIFT =        11,  // semi-automous position, yaw and throttle control
        SPORT =        13,  // manual earth-frame angular rate control with manual throttle
        FLIP =         14,  // automatically flip the vehicle on the roll axis
        AUTOTUNE =     15,  // automatically tune the vehicle's roll and pitch gains
        POSHOLD =      16,  // automatic position hold with manual override, with automatic throttle
        BRAKE =        17,  // full-brake using inertial/GPS system, no pilot input
        THROW =        18,  // throw to launch mode using inertial/GPS system, no pilot input
        AVOID_ADSB =   19,  // automatic avoidance of obstacles in the macro scale - e.g. full-sized aircraft
        GUIDED_NOGPS = 20,  // guided mode but only accepts attitude and altitude
        SMART_RTL =    21,  // SMART_RTL returns to home by retracing its steps
        FLOWHOLD  =    22,  // FLOWHOLD holds position with optical flow without rangefinder
        FOLLOW    =    23,  // follow attempts to follow another vehicle or ground station
        ZIGZAG    =    24,  // ZIGZAG mode is able to fly in a zigzag manner with predefined point A and point B
        SYSTEMID  =    25,  // System ID mode produces automated system identification signals in the controllers
        AUTOROTATE =   26,  // Autonomous autorotation
    };

    // constructor
    Mode(void);

    // do not allow copying
    Mode(const Mode &other) = delete;
    Mode &operator=(const Mode&) = delete;

    // child classes should override these methods
    virtual bool init(bool ignore_checks) {
        return true;
    }
    virtual void run() = 0;
    virtual bool requires_GPS() const = 0;
    virtual bool has_manual_throttle() const = 0;
    virtual bool allows_arming(bool from_gcs) const = 0;
    virtual bool is_autopilot() const { return false; }
    virtual bool has_user_takeoff(bool must_navigate) const { return false; }
    virtual bool in_guided_mode() const { return false; }
    virtual bool logs_attitude() const { return false; }

    // return a string for this flightmode
    virtual const char *name() const = 0;
    virtual const char *name4() const = 0;

    bool do_user_takeoff(float takeoff_alt_cm, bool must_navigate);
    virtual bool is_taking_off() const;
    static void takeoff_stop() { takeoff.stop(); }

    virtual bool landing_gear_should_be_deployed() const { return false; }
    virtual bool is_landing() const { return false; }

    // mode requires terrain to be present to be functional
    virtual bool requires_terrain_failsafe() const { return false; }

    // functions for reporting to GCS
    virtual bool get_wp(Location &loc) { return false; };
    virtual int32_t wp_bearing() const { return 0; }
    virtual uint32_t wp_distance() const { return 0; }
    virtual float crosstrack_error() const { return 0.0f;}

    void update_navigation();

    int32_t get_alt_above_ground_cm(void);

    // pilot input processing
    void get_pilot_desired_lean_angles(float &roll_out, float &pitch_out, float angle_max, float angle_limit) const;
    float get_pilot_desired_yaw_rate(int16_t stick_angle);
    float get_pilot_desired_throttle() const;

    const Vector3f& get_desired_velocity() {
        // note that position control isn't used in every mode, so
        // this may return bogus data:
        return pos_control->get_desired_velocity();
    }

protected:

    // navigation support functions
    virtual void run_autopilot() {}

    // helper functions
    bool is_disarmed_or_landed() const;
    void zero_throttle_and_relax_ac(bool spool_up = false);
    void zero_throttle_and_hold_attitude();
    void make_safe_spool_down();

    // functions to control landing
    // in modes that support landing
    void land_run_horizontal_control();
    void land_run_vertical_control(bool pause_descent = false);

    // return expected input throttle setting to hover:
    virtual float throttle_hover() const;

    // Alt_Hold based flight mode states used in Alt_Hold, Loiter, and Sport
    enum AltHoldModeState {
        AltHold_MotorStopped,
        AltHold_Takeoff,
        AltHold_Landed_Ground_Idle,
        AltHold_Landed_Pre_Takeoff,
        AltHold_Flying
    };
    AltHoldModeState get_alt_hold_state(float target_climb_rate_cms);

    // convenience references to avoid code churn in conversion:
    Parameters &g;
    ParametersG2 &g2;
    AC_WPNav *&wp_nav;
    AC_Loiter *&loiter_nav;
    AC_PosControl *&pos_control;
    AP_InertialNav &inertial_nav;
    AP_AHRS &ahrs;
    AC_AttitudeControl_t *&attitude_control;
    MOTOR_CLASS *&motors;
    RC_Channel *&channel_roll;
    RC_Channel *&channel_pitch;
    RC_Channel *&channel_throttle;
    RC_Channel *&channel_yaw;
    float &G_Dt;

    // note that we support two entirely different automatic takeoffs:

    // "user-takeoff", which is available in modes such as ALT_HOLD
    // (see has_user_takeoff method).  "user-takeoff" is a simple
    // reach-altitude-based-on-pilot-input-or-parameter routine.

    // "auto-takeoff" is used by both Guided and Auto, and is
    // basically waypoint navigation with pilot yaw permitted.

    // user-takeoff support; takeoff state is shared across all mode instances
    class _TakeOff {
    public:
        void start(float alt_cm);
        void stop();
        void get_climb_rates(float& pilot_climb_rate,
                             float& takeoff_climb_rate);
        bool triggered(float target_climb_rate) const;

        bool running() const { return _running; }
    private:
        bool _running;
        float max_speed;
        float alt_delta;
        uint32_t start_ms;
    };

    static _TakeOff takeoff;

    virtual bool do_user_takeoff_start(float takeoff_alt_cm);

    // method shared by both Guided and Auto for takeoff.  This is
    // waypoint navigation but the user can control the yaw.
    void auto_takeoff_run();
    void auto_takeoff_set_start_alt(void);
    void auto_takeoff_attitude_run(float target_yaw_rate);
    // altitude below which we do no navigation in auto takeoff
    static float auto_takeoff_no_nav_alt_cm;

public:
    // Navigation Yaw control
    class AutoYaw {

    public:

        // yaw(): main product of AutoYaw; the heading:
        float yaw();

        // mode(): current method of determining desired yaw:
        autopilot_yaw_mode mode() const { return (autopilot_yaw_mode)_mode; }
        void set_mode_to_default(bool rtl);
        void set_mode(autopilot_yaw_mode new_mode);
        autopilot_yaw_mode default_mode(bool rtl) const;

        // rate_cds(): desired yaw rate in centidegrees/second:
        float rate_cds() const;
        void set_rate(float new_rate_cds);

        // set_roi(...): set a "look at" location:
        void set_roi(const Location &roi_location);

        void set_fixed_yaw(float angle_deg,
                           float turn_rate_dps,
                           int8_t direction,
                           bool relative_angle);

    private:

        float look_ahead_yaw();
        float roi_yaw();

        // auto flight mode's yaw mode
        uint8_t _mode = AUTO_YAW_LOOK_AT_NEXT_WP;

        // Yaw will point at this location if mode is set to AUTO_YAW_ROI
        Vector3f roi;

        // bearing from current location to the ROI
        float _roi_yaw;

        // yaw used for YAW_FIXED yaw_mode
        int32_t _fixed_yaw;

        // Deg/s we should turn
        int16_t _fixed_yaw_slewrate;

        // heading when in yaw_look_ahead_yaw
        float _look_ahead_yaw;

        // turn rate (in cds) when auto_yaw_mode is set to AUTO_YAW_RATE
        float _rate_cds;

        // used to reduce update rate to 100hz:
        uint8_t roi_yaw_counter;

    };
    static AutoYaw auto_yaw;

    // pass-through functions to reduce code churn on conversion;
    // these are candidates for moving into the Mode base
    // class.
    float get_pilot_desired_climb_rate(float throttle_control);
    float get_non_takeoff_throttle(void);
    void update_simple_mode(void);
    bool set_mode(Mode::Number mode, ModeReason reason);
    void set_land_complete(bool b);
    GCS_Copter &gcs();
    void set_throttle_takeoff(void);
    float get_avoidance_adjusted_climbrate(float target_rate);
    uint16_t get_pilot_speed_dn(void);

    // end pass-through functions
};


#if MODE_ACRO_ENABLED == ENABLED
class ModeAcro : public Mode {

public:
    // inherit constructor
    using Mode::Mode;

    enum class Trainer {
        OFF = 0,
        LEVELING = 1,
        LIMITED = 2,
    };

    virtual void run() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return true; }
    bool allows_arming(bool from_gcs) const override { return true; };
    bool is_autopilot() const override { return false; }

protected:

    const char *name() const override { return "ACRO"; }
    const char *name4() const override { return "ACRO"; }

    void get_pilot_desired_angle_rates(int16_t roll_in, int16_t pitch_in, int16_t yaw_in, float &roll_out, float &pitch_out, float &yaw_out);

    float throttle_hover() const override;

private:

};
#endif

#if FRAME_CONFIG == HELI_FRAME
class ModeAcro_Heli : public ModeAcro {

public:
    // inherit constructor
    using ModeAcro::Mode;

    bool init(bool ignore_checks) override;
    void run() override;
    void virtual_flybar( float &roll_out, float &pitch_out, float &yaw_out, float pitch_leak, float roll_leak);

protected:
private:
};
#endif


class ModeAltHold : public Mode {

public:
    // inherit constructor
    using Mode::Mode;

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; };
    bool is_autopilot() const override { return false; }
    bool has_user_takeoff(bool must_navigate) const override {
        return !must_navigate;
    }

protected:

    const char *name() const override { return "ALT_HOLD"; }
    const char *name4() const override { return "ALTH"; }

private:

};


class ModeAuto : public Mode {

public:
    // inherit constructor
    using Mode::Mode;

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return false; };
    bool is_autopilot() const override { return true; }
    bool in_guided_mode() const override { return mode() == Auto_NavGuided; }

    // Auto
    AutoMode mode() const { return _mode; }

    bool loiter_start();
    void rtl_start();
    void takeoff_start(const Location& dest_loc);
    void wp_start(const Location& dest_loc);
    void land_start();
    void land_start(const Vector3f& destination);
    void circle_movetoedge_start(const Location &circle_center, float radius_m);
    void circle_start();
    void spline_start(const Vector3f& destination, bool stopped_at_start, AC_WPNav::spline_segment_end_type seg_end_type, const Vector3f& next_spline_destination);
    void spline_start(const Location& destination, bool stopped_at_start, AC_WPNav::spline_segment_end_type seg_end_type, const Location& next_destination);
    void nav_guided_start();

    bool is_landing() const override;
    bool landing_gear_should_be_deployed() const override;

    bool is_taking_off() const override;

    bool requires_terrain_failsafe() const override { return true; }

    // return true if this flight mode supports user takeoff
    //  must_nagivate is true if mode must also control horizontal position
    virtual bool has_user_takeoff(bool must_navigate) const override { return false; }

    void payload_place_start();

    // for GCS_MAVLink to call:
    bool do_guided(const AP_Mission::Mission_Command& cmd);

    AP_Mission mission{
        FUNCTOR_BIND_MEMBER(&ModeAuto::start_command, bool, const AP_Mission::Mission_Command &),
        FUNCTOR_BIND_MEMBER(&ModeAuto::verify_command, bool, const AP_Mission::Mission_Command &),
        FUNCTOR_BIND_MEMBER(&ModeAuto::exit_mission, void)};

protected:

    const char *name() const override { return "AUTO"; }
    const char *name4() const override { return "AUTO"; }

    uint32_t wp_distance() const override;
    int32_t wp_bearing() const override;
    float crosstrack_error() const override { return wp_nav->crosstrack_error();}
    bool get_wp(Location &loc) override;
    void run_autopilot() override;

private:

    bool start_command(const AP_Mission::Mission_Command& cmd);
    bool verify_command(const AP_Mission::Mission_Command& cmd);
    void exit_mission();

    void takeoff_run();
    void wp_run();
    void spline_run();
    void land_run();
    void rtl_run();
    void circle_run();
    void nav_guided_run();
    void loiter_run();
    void loiter_to_alt_run();

    Location loc_from_cmd(const AP_Mission::Mission_Command& cmd) const;

    void payload_place_start(const Vector3f& destination);
    void payload_place_run();
    bool payload_place_run_should_run();
    void payload_place_run_loiter();
    void payload_place_run_descend();
    void payload_place_run_release();

    AutoMode _mode = Auto_TakeOff;   // controls which auto controller is run

    Location terrain_adjusted_location(const AP_Mission::Mission_Command& cmd) const;

    void do_takeoff(const AP_Mission::Mission_Command& cmd);
    void do_nav_wp(const AP_Mission::Mission_Command& cmd);
    void do_land(const AP_Mission::Mission_Command& cmd);
    void do_loiter_unlimited(const AP_Mission::Mission_Command& cmd);
    void do_circle(const AP_Mission::Mission_Command& cmd);
    void do_loiter_time(const AP_Mission::Mission_Command& cmd);
    void do_loiter_to_alt(const AP_Mission::Mission_Command& cmd);
    void do_spline_wp(const AP_Mission::Mission_Command& cmd);
#if NAV_GUIDED == ENABLED
    void do_nav_guided_enable(const AP_Mission::Mission_Command& cmd);
    void do_guided_limits(const AP_Mission::Mission_Command& cmd);
#endif
    void do_nav_delay(const AP_Mission::Mission_Command& cmd);
    void do_wait_delay(const AP_Mission::Mission_Command& cmd);
    void do_within_distance(const AP_Mission::Mission_Command& cmd);
    void do_yaw(const AP_Mission::Mission_Command& cmd);
    void do_change_speed(const AP_Mission::Mission_Command& cmd);
    void do_set_home(const AP_Mission::Mission_Command& cmd);
    void do_roi(const AP_Mission::Mission_Command& cmd);
    void do_mount_control(const AP_Mission::Mission_Command& cmd);
#if PARACHUTE == ENABLED
    void do_parachute(const AP_Mission::Mission_Command& cmd);
#endif
#if WINCH_ENABLED == ENABLED
    void do_winch(const AP_Mission::Mission_Command& cmd);
#endif
    void do_payload_place(const AP_Mission::Mission_Command& cmd);
    void do_RTL(void);

    bool verify_takeoff();
    bool verify_land();
    bool verify_payload_place();
    bool verify_loiter_unlimited();
    bool verify_loiter_time(const AP_Mission::Mission_Command& cmd);
    bool verify_loiter_to_alt();
    bool verify_RTL();
    bool verify_wait_delay();
    bool verify_within_distance();
    bool verify_yaw();
    bool verify_nav_wp(const AP_Mission::Mission_Command& cmd);
    bool verify_circle(const AP_Mission::Mission_Command& cmd);
    bool verify_spline_wp(const AP_Mission::Mission_Command& cmd);
#if NAV_GUIDED == ENABLED
    bool verify_nav_guided_enable(const AP_Mission::Mission_Command& cmd);
#endif
    bool verify_nav_delay(const AP_Mission::Mission_Command& cmd);

    // Loiter control
    uint16_t loiter_time_max;                // How long we should stay in Loiter Mode for mission scripting (time in seconds)
    uint32_t loiter_time;                    // How long have we been loitering - The start time in millis

    struct {
        bool reached_destination_xy : 1;
        bool loiter_start_done : 1;
        bool reached_alt : 1;
        float alt_error_cm;
        int32_t alt;
    } loiter_to_alt;

    // Delay the next navigation command
    uint32_t nav_delay_time_max_ms;  // used for delaying the navigation commands (eg land,takeoff etc.)
    uint32_t nav_delay_time_start_ms;

    // Delay Mission Scripting Command
    int32_t condition_value;  // used in condition commands (eg delay, change alt, etc.)
    uint32_t condition_start;

    enum class State {
        FlyToLocation = 0,
        Descending = 1
    };
    State state = State::FlyToLocation;

    struct {
        PayloadPlaceStateType state = PayloadPlaceStateType_Calibrating_Hover_Start; // records state of place (descending, releasing, released, ...)
        uint32_t hover_start_timestamp; // milliseconds
        float hover_throttle_level;
        uint32_t descend_start_timestamp; // milliseconds
        uint32_t place_start_timestamp; // milliseconds
        float descend_throttle_level;
        float descend_start_altitude;
        float descend_max; // centimetres
    } nav_payload_place;
};

#if AUTOTUNE_ENABLED == ENABLED
/*
  wrapper class for AC_AutoTune
 */
class AutoTune : public AC_AutoTune
{
public:
    bool init() override;
    void run() override;

protected:
    bool start(void) override;
    bool position_ok() override;
    float get_pilot_desired_climb_rate_cms(void) const override;
    void get_pilot_desired_rp_yrate_cd(float &roll_cd, float &pitch_cd, float &yaw_rate_cds) override;
    void init_z_limits() override;
    void log_pids() override;
};

class ModeAutoTune : public Mode {

public:
    // inherit constructor
    using Mode::Mode;

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return false; }
    bool is_autopilot() const override { return false; }

    void save_tuning_gains();
    void stop();
    void reset();

protected:

    const char *name() const override { return "AUTOTUNE"; }
    const char *name4() const override { return "ATUN"; }
};
#endif


class ModeBrake : public Mode {

public:
    // inherit constructor
    using Mode::Mode;

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return false; };
    bool is_autopilot() const override { return false; }

    void timeout_to_loiter_ms(uint32_t timeout_ms);

protected:

    const char *name() const override { return "BRAKE"; }
    const char *name4() const override { return "BRAK"; }

private:

    void init_target();

    uint32_t _timeout_start;
    uint32_t _timeout_ms;

};


class ModeCircle : public Mode {

public:
    // inherit constructor
    using Mode::Mode;

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return false; };
    bool is_autopilot() const override { return true; }

protected:

    const char *name() const override { return "CIRCLE"; }
    const char *name4() const override { return "CIRC"; }

    uint32_t wp_distance() const override;
    int32_t wp_bearing() const override;

private:

    // Circle
    bool pilot_yaw_override = false; // true if pilot is overriding yaw
    bool speed_changing = false;     // true when the roll stick is being held to facilitate stopping at 0 rate
};


class ModeDrift : public Mode {

public:
    // inherit constructor
    using Mode::Mode;

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; };
    bool is_autopilot() const override { return false; }

protected:

    const char *name() const override { return "DRIFT"; }
    const char *name4() const override { return "DRIF"; }

private:

    float get_throttle_assist(float velz, float pilot_throttle_scaled);

};


class ModeFlip : public Mode {

public:
    // inherit constructor
    using Mode::Mode;

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return false; };
    bool is_autopilot() const override { return false; }

protected:

    const char *name() const override { return "FLIP"; }
    const char *name4() const override { return "FLIP"; }

private:

    // Flip
    Vector3f orig_attitude;         // original vehicle attitude before flip

    enum class FlipState : uint8_t {
        Start,
        Roll,
        Pitch_A,
        Pitch_B,
        Recover,
        Abandon
    };
    FlipState _state;               // current state of flip
    Mode::Number   orig_control_mode;   // flight mode when flip was initated
    uint32_t  start_time_ms;          // time since flip began
    int8_t    roll_dir;            // roll direction (-1 = roll left, 1 = roll right)
    int8_t    pitch_dir;           // pitch direction (-1 = pitch forward, 1 = pitch back)
};


#if !HAL_MINIMIZE_FEATURES && OPTFLOW == ENABLED
/*
  class to support FLOWHOLD mode, which is a position hold mode using
  optical flow directly, avoiding the need for a rangefinder
 */

class ModeFlowHold : public Mode {
public:
    // need a constructor for parameters
    ModeFlowHold(void);

    bool init(bool ignore_checks) override;
    void run(void) override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; };
    bool is_autopilot() const override { return false; }
    bool has_user_takeoff(bool must_navigate) const override {
        return !must_navigate;
    }

    static const struct AP_Param::GroupInfo var_info[];

protected:
    const char *name() const override { return "FLOWHOLD"; }
    const char *name4() const override { return "FHLD"; }

private:

    // FlowHold states
    enum FlowHoldModeState {
        FlowHold_MotorStopped,
        FlowHold_Takeoff,
        FlowHold_Flying,
        FlowHold_Landed
    };

    // calculate attitude from flow data
    void flow_to_angle(Vector2f &bf_angle);

    LowPassFilterVector2f flow_filter;

    bool flowhold_init(bool ignore_checks);
    void flowhold_run();
    void flowhold_flow_to_angle(Vector2f &angle, bool stick_input);
    void update_height_estimate(void);

    // minimum assumed height
    const float height_min = 0.1f;

    // maximum scaling height
    const float height_max = 3.0f;

    AP_Float flow_max;
    AC_PI_2D flow_pi_xy{0.2f, 0.3f, 3000, 5, 0.0025f};
    AP_Float flow_filter_hz;
    AP_Int8  flow_min_quality;
    AP_Int8  brake_rate_dps;

    float quality_filtered;

    uint8_t log_counter;
    bool limited;
    Vector2f xy_I;

    // accumulated INS delta velocity in north-east form since last flow update
    Vector2f delta_velocity_ne;

    // last flow rate in radians/sec in north-east axis
    Vector2f last_flow_rate_rps;

    // timestamp of last flow data
    uint32_t last_flow_ms;

    float last_ins_height;
    float height_offset;

    // are we braking after pilot input?
    bool braking;

    // last time there was significant stick input
    uint32_t last_stick_input_ms;
};
#endif // OPTFLOW


class ModeGuided : public Mode {

public:
    // inherit constructor
    using Mode::Mode;

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return from_gcs; }
    bool is_autopilot() const override { return true; }
    bool has_user_takeoff(bool must_navigate) const override { return true; }
    bool in_guided_mode() const override { return true; }

    bool requires_terrain_failsafe() const override { return true; }

    void set_angle(const Quaternion &q, float climb_rate_cms, bool use_yaw_rate, float yaw_rate_rads);
    bool set_destination(const Vector3f& destination, bool use_yaw = false, float yaw_cd = 0.0, bool use_yaw_rate = false, float yaw_rate_cds = 0.0, bool yaw_relative = false, bool terrain_alt = false);
    bool set_destination(const Location& dest_loc, bool use_yaw = false, float yaw_cd = 0.0, bool use_yaw_rate = false, float yaw_rate_cds = 0.0, bool yaw_relative = false);
    bool get_wp(Location &loc) override;
    void set_velocity(const Vector3f& velocity, bool use_yaw = false, float yaw_cd = 0.0, bool use_yaw_rate = false, float yaw_rate_cds = 0.0, bool yaw_relative = false, bool log_request = true);
    bool set_destination_posvel(const Vector3f& destination, const Vector3f& velocity, bool use_yaw = false, float yaw_cd = 0.0, bool use_yaw_rate = false, float yaw_rate_cds = 0.0, bool yaw_relative = false);

    void limit_clear();
    void limit_init_time_and_pos();
    void limit_set(uint32_t timeout_ms, float alt_min_cm, float alt_max_cm, float horiz_max_cm);
    bool limit_check();

    bool is_taking_off() const override;

    bool do_user_takeoff_start(float final_alt_above_home) override;

    GuidedMode mode() const { return guided_mode; }

    void angle_control_start();
    void angle_control_run();

protected:

    const char *name() const override { return "GUIDED"; }
    const char *name4() const override { return "GUID"; }

    uint32_t wp_distance() const override;
    int32_t wp_bearing() const override;
    float crosstrack_error() const override;

private:

    void pos_control_start();
    void vel_control_start();
    void posvel_control_start();
    void takeoff_run();
    void pos_control_run();
    void vel_control_run();
    void posvel_control_run();
    void set_desired_velocity_with_accel_and_fence_limits(const Vector3f& vel_des);
    void set_yaw_state(bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_angle);

    // controls which controller is run (pos or vel):
    GuidedMode guided_mode = Guided_TakeOff;

};


class ModeGuidedNoGPS : public ModeGuided {

public:
    // inherit constructor
    using ModeGuided::Mode;

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return from_gcs; }
    bool is_autopilot() const override { return true; }

protected:

    const char *name() const override { return "GUIDED_NOGPS"; }
    const char *name4() const override { return "GNGP"; }

private:

};


class ModeLand : public Mode {

public:
    // inherit constructor
    using Mode::Mode;

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return false; };
    bool is_autopilot() const override { return true; }

    bool is_landing() const override { return true; };
    bool landing_gear_should_be_deployed() const override { return true; };

    void do_not_use_GPS();

protected:

    const char *name() const override { return "LAND"; }
    const char *name4() const override { return "LAND"; }

private:

    void gps_run();
    void nogps_run();
};


class ModeLoiter : public Mode {

public:
    // inherit constructor
    using Mode::Mode;

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; };
    bool is_autopilot() const override { return false; }
    bool has_user_takeoff(bool must_navigate) const override { return true; }

#if PRECISION_LANDING == ENABLED
    void set_precision_loiter_enabled(bool value) { _precision_loiter_enabled = value; }
#endif

protected:

    const char *name() const override { return "LOITER"; }
    const char *name4() const override { return "LOIT"; }

    uint32_t wp_distance() const override;
    int32_t wp_bearing() const override;

#if PRECISION_LANDING == ENABLED
    bool do_precision_loiter();
    void precision_loiter_xy();
#endif

private:

#if PRECISION_LANDING == ENABLED
    bool _precision_loiter_enabled;
#endif

};


class ModePosHold : public Mode {

public:
    // inherit constructor
    using Mode::Mode;

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; };
    bool is_autopilot() const override { return false; }
    bool has_user_takeoff(bool must_navigate) const override { return true; }

protected:

    const char *name() const override { return "POSHOLD"; }
    const char *name4() const override { return "PHLD"; }

private:

    void update_pilot_lean_angle(float &lean_angle_filtered, float &lean_angle_raw);
    float mix_controls(float mix_ratio, float first_control, float second_control);
    void update_brake_angle_from_velocity(float &brake_angle, float velocity);
    void update_wind_comp_estimate();
    void get_wind_comp_lean_angles(float &roll_angle, float &pitch_angle);
    void roll_controller_to_pilot_override();
    void pitch_controller_to_pilot_override();

    enum class RPMode {
        PILOT_OVERRIDE=0,            // pilot is controlling this axis (i.e. roll or pitch)
        BRAKE,                       // this axis is braking towards zero
        BRAKE_READY_TO_LOITER,       // this axis has completed braking and is ready to enter loiter mode (both modes must be this value before moving to next stage)
        BRAKE_TO_LOITER,             // both vehicle's axis (roll and pitch) are transitioning from braking to loiter mode (braking and loiter controls are mixed)
        LOITER,                      // both vehicle axis are holding position
        CONTROLLER_TO_PILOT_OVERRIDE // pilot has input controls on this axis and this axis is transitioning to pilot override (other axis will transition to brake if no pilot input)
    };

    RPMode roll_mode;
    RPMode pitch_mode;

    uint8_t braking_time_updated_roll   : 1;    // true once we have re-estimated the braking time.  This is done once as the vehicle begins to flatten out after braking
    uint8_t braking_time_updated_pitch  : 1;    // true once we have re-estimated the braking time.  This is done once as the vehicle begins to flatten out after braking

    // pilot input related variables
    float pilot_roll;                         // pilot requested roll angle (filtered to slow returns to zero)
    float pilot_pitch;                        // pilot requested roll angle (filtered to slow returns to zero)

    // braking related variables
    float brake_gain;                           // gain used during conversion of vehicle's velocity to lean angle during braking (calculated from brake_rate)
    float brake_roll;                           // target roll angle during braking periods
    float brake_pitch;                          // target pitch angle during braking periods
    int16_t brake_timeout_roll;                 // number of cycles allowed for the braking to complete, this timeout will be updated at half-braking
    int16_t brake_timeout_pitch;                // number of cycles allowed for the braking to complete, this timeout will be updated at half-braking
    float brake_angle_max_roll;                 // maximum lean angle achieved during braking.  Used to determine when the vehicle has begun to flatten out so that we can re-estimate the braking time
    float brake_angle_max_pitch;                // maximum lean angle achieved during braking  Used to determine when the vehicle has begun to flatten out so that we can re-estimate the braking time
    int16_t brake_to_loiter_timer;              // cycles to mix brake and loiter controls in POSHOLD_BRAKE_TO_LOITER

    // loiter related variables
    int16_t controller_to_pilot_timer_roll;     // cycles to mix controller and pilot controls in POSHOLD_CONTROLLER_TO_PILOT
    int16_t controller_to_pilot_timer_pitch;    // cycles to mix controller and pilot controls in POSHOLD_CONTROLLER_TO_PILOT
    float controller_final_roll;                // final roll angle from controller as we exit brake or loiter mode (used for mixing with pilot input)
    float controller_final_pitch;               // final pitch angle from controller as we exit brake or loiter mode (used for mixing with pilot input)

    // wind compensation related variables
    Vector2f wind_comp_ef;                      // wind compensation in earth frame, filtered lean angles from position controller
    float wind_comp_roll;                       // roll angle to compensate for wind
    float wind_comp_pitch;                      // pitch angle to compensate for wind
    uint16_t wind_comp_start_timer;             // counter to delay start of wind compensation for a short time after loiter is engaged
    int8_t  wind_comp_timer;                    // counter to reduce wind comp roll/pitch lean angle calcs to 10hz

    // final output
    float roll;   // final roll angle sent to attitude controller
    float pitch;  // final pitch angle sent to attitude controller

};


class ModeRTL : public Mode {

public:
    // inherit constructor
    using Mode::Mode;

    bool init(bool ignore_checks) override;
    void run() override {
        return run(true);
    }
    void run(bool disarm_on_land);

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return false; };
    bool is_autopilot() const override { return true; }

    bool requires_terrain_failsafe() const override { return true; }

    // for reporting to GCS
    bool get_wp(Location &loc) override;

    // RTL states
    enum RTLState {
        RTL_Starting,
        RTL_InitialClimb,
        RTL_ReturnHome,
        RTL_LoiterAtHome,
        RTL_FinalDescent,
        RTL_Land
    };
    RTLState state() { return _state; }

    // this should probably not be exposed
    bool state_complete() { return _state_complete; }

    bool is_landing() const override;
    bool landing_gear_should_be_deployed() const override;

    void restart_without_terrain();

    // enum for RTL_ALT_TYPE parameter
    enum class RTLAltType {
        RTL_ALTTYPE_RELATIVE = 0,
        RTL_ALTTYPE_TERRAIN = 1
    };
    ModeRTL::RTLAltType get_alt_type() const;

protected:

    const char *name() const override { return "RTL"; }
    const char *name4() const override { return "RTL "; }

    // for reporting to GCS
    uint32_t wp_distance() const override;
    int32_t wp_bearing() const override;
    float crosstrack_error() const override { return wp_nav->crosstrack_error();}

    void descent_start();
    void descent_run();
    void land_start();
    void land_run(bool disarm_on_land);

    void set_descent_target_alt(uint32_t alt) { rtl_path.descent_target.alt = alt; }

private:

    void climb_start();
    void return_start();
    void climb_return_run();
    void loiterathome_start();
    void loiterathome_run();
    void build_path();
    void compute_return_target();

    RTLState _state = RTL_InitialClimb;  // records state of rtl (initial climb, returning home, etc)
    bool _state_complete = false; // set to true if the current state is completed

    struct {
        // NEU w/ Z element alt-above-ekf-origin unless use_terrain is true in which case Z element is alt-above-terrain
        Location origin_point;
        Location climb_target;
        Location return_target;
        Location descent_target;
        bool land;
    } rtl_path;

    // return target alt type
    enum class ReturnTargetAltType {
        RETURN_TARGET_ALTTYPE_RELATIVE = 0,
        RETURN_TARGET_ALTTYPE_RANGEFINDER = 1,
        RETURN_TARGET_ALTTYPE_TERRAINDATABASE = 2
    };

    // Loiter timer - Records how long we have been in loiter
    uint32_t _loiter_start_time;

    bool terrain_following_allowed;
};


class ModeSmartRTL : public ModeRTL {

public:
    // inherit constructor
    using ModeRTL::Mode;

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return false; }
    bool is_autopilot() const override { return true; }

    void save_position();
    void exit();

protected:

    const char *name() const override { return "SMARTRTL"; }
    const char *name4() const override { return "SRTL"; }

    // for reporting to GCS
    bool get_wp(Location &loc) override;
    uint32_t wp_distance() const override;
    int32_t wp_bearing() const override;
    float crosstrack_error() const override { return wp_nav->crosstrack_error();}

private:

    void wait_cleanup_run();
    void path_follow_run();
    void pre_land_position_run();
    void land();
    SmartRTLState smart_rtl_state = SmartRTL_PathFollow;

};


class ModeSport : public Mode {

public:
    // inherit constructor
    using Mode::Mode;

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; };
    bool is_autopilot() const override { return false; }
    bool has_user_takeoff(bool must_navigate) const override {
        return !must_navigate;
    }

protected:

    const char *name() const override { return "SPORT"; }
    const char *name4() const override { return "SPRT"; }

private:

};


class ModeStabilize : public Mode {

public:
    // inherit constructor
    using Mode::Mode;

    virtual void run() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return true; }
    bool allows_arming(bool from_gcs) const override { return true; };
    bool is_autopilot() const override { return false; }

protected:

    const char *name() const override { return "STABILIZE"; }
    const char *name4() const override { return "STAB"; }

private:

};

#if FRAME_CONFIG == HELI_FRAME
class ModeStabilize_Heli : public ModeStabilize {

public:
    // inherit constructor
    using ModeStabilize::Mode;

    bool init(bool ignore_checks) override;
    void run() override;

protected:

private:

};
#endif

class ModeSystemId : public Mode {

public:
    ModeSystemId(void);

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; };
    bool is_autopilot() const override { return false; }
    bool logs_attitude() const override { return true; }
    void set_magnitude(float input) { waveform_magnitude = input; }

    static const struct AP_Param::GroupInfo var_info[];

protected:

    const char *name() const override { return "SYSTEMID"; }
    const char *name4() const override { return "SYSI"; }

private:

    void log_data();
    float waveform(float time);

    enum class AxisType {
        NONE = 0,           // none
        INPUT_ROLL = 1,     // angle input roll axis is being excited
        INPUT_PITCH = 2,    // angle pitch axis is being excited
        INPUT_YAW = 3,      // angle yaw axis is being excited
        RECOVER_ROLL = 4,   // angle roll axis is being excited
        RECOVER_PITCH = 5,  // angle pitch axis is being excited
        RECOVER_YAW = 6,    // angle yaw axis is being excited
        RATE_ROLL = 7,      // rate roll axis is being excited
        RATE_PITCH = 8,     // rate pitch axis is being excited
        RATE_YAW = 9,       // rate yaw axis is being excited
        MIX_ROLL = 10,      // mixer roll axis is being excited
        MIX_PITCH = 11,     // mixer pitch axis is being excited
        MIX_YAW = 12,       // mixer pitch axis is being excited
        MIX_THROTTLE = 13,  // mixer throttle axis is being excited
    };

    AP_Int8 axis;               // Controls which axis are being excited. Set to non-zero to display other parameters
    AP_Float waveform_magnitude;// Magnitude of chirp waveform
    AP_Float frequency_start;   // Frequency at the start of the chirp
    AP_Float frequency_stop;    // Frequency at the end of the chirp
    AP_Float time_fade_in;      // Time to reach maximum amplitude of chirp
    AP_Float time_record;       // Time taken to complete the chirp waveform
    AP_Float time_fade_out;     // Time to reach zero amplitude after chirp finishes

    bool att_bf_feedforward;    // Setting of attitude_control->get_bf_feedforward
    float waveform_time;        // Time reference for waveform
    float waveform_sample;      // Current waveform sample
    float waveform_freq_rads;   // Instantaneous waveform frequency
    float time_const_freq;      // Time at constant frequency before chirp starts
    int8_t log_subsample;       // Subsample multiple for logging.

    // System ID states
    enum class SystemIDModeState {
        SYSTEMID_STATE_STOPPED,
        SYSTEMID_STATE_TESTING
    } systemid_state;
};

class ModeThrow : public Mode {

public:
    // inherit constructor
    using Mode::Mode;

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; };
    bool is_autopilot() const override { return false; }

    // Throw types
    enum ThrowModeType {
        ThrowType_Upward = 0,
        ThrowType_Drop = 1
    };

protected:

    const char *name() const override { return "THROW"; }
    const char *name4() const override { return "THRW"; }

private:

    bool throw_detected();
    bool throw_position_good();
    bool throw_height_good();
    bool throw_attitude_good();

    // Throw stages
    enum ThrowModeStage {
        Throw_Disarmed,
        Throw_Detecting,
        Throw_Uprighting,
        Throw_HgtStabilise,
        Throw_PosHold
    };

    ThrowModeStage stage = Throw_Disarmed;
    ThrowModeStage prev_stage = Throw_Disarmed;
    uint32_t last_log_ms;
    bool nextmode_attempted;
    uint32_t free_fall_start_ms;    // system time free fall was detected
    float free_fall_start_velz;     // vertical velocity when free fall was detected
};

// modes below rely on Guided mode so must be declared at the end (instead of in alphabetical order)

class ModeAvoidADSB : public ModeGuided {

public:
    // inherit constructor
    using ModeGuided::Mode;

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return false; }
    bool is_autopilot() const override { return true; }

    bool set_velocity(const Vector3f& velocity_neu);

protected:

    const char *name() const override { return "AVOID_ADSB"; }
    const char *name4() const override { return "AVOI"; }

private:

};

class ModeFollow : public ModeGuided {

public:

    // inherit constructor
    using ModeGuided::Mode;

    bool init(bool ignore_checks) override;
    void exit();
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return false; }
    bool is_autopilot() const override { return true; }

protected:

    const char *name() const override { return "FOLLOW"; }
    const char *name4() const override { return "FOLL"; }

    // for reporting to GCS
    bool get_wp(Location &loc) override;
    uint32_t wp_distance() const override;
    int32_t wp_bearing() const override;

    uint32_t last_log_ms;   // system time of last time desired velocity was logging
};

class ModeZigZag : public Mode {        

public:

    // Inherit constructor
    using Mode::Mode;

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return true; }

    // save current position as A (dest_num = 0) or B (dest_num = 1).  If both A and B have been saved move to the one specified
    void save_or_move_to_destination(uint8_t dest_num);

    // return manual control to the pilot
    void return_to_manual_control(bool maintain_target);

protected:

    const char *name() const override { return "ZIGZAG"; }
    const char *name4() const override { return "ZIGZ"; }

private:

    void auto_control();
    void manual_control();
    bool reached_destination();
    bool calculate_next_dest(uint8_t position_num, bool use_wpnav_alt, Vector3f& next_dest, bool& terrain_alt) const;

    Vector2f dest_A;    // in NEU frame in cm relative to ekf origin
    Vector2f dest_B;    // in NEU frame in cm relative to ekf origin

    enum zigzag_state {
        STORING_POINTS, // storing points A and B, pilot has manual control
        AUTO,           // after A and B defined, pilot toggle the switch from one side to the other, vehicle flies autonomously
        MANUAL_REGAIN   // pilot toggle the switch to middle position, has manual control
    } stage;

    uint32_t reach_wp_time_ms = 0;  // time since vehicle reached destination (or zero if not yet reached)
};

#if MODE_AUTOROTATE_ENABLED == ENABLED
class ModeAutorotate : public Mode {

public:

    // inherit constructor
    using Mode::Mode;

    bool init(bool ignore_checks) override;
    void run() override;

    bool is_autopilot() const override { return true; }
    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return false; };

    static const struct AP_Param::GroupInfo  var_info[];

protected:

    const char *name() const override { return "AUTOROTATE"; }
    const char *name4() const override { return "AROT"; }

private:

    // --- Internal variables ---
    float _initial_rpm;             // Head speed recorded at initiation of flight mode (RPM)
    float _target_head_speed;       // The terget head main rotor head speed.  Normalised by main rotor set point
    float _fwd_speed_target;        // Target forward speed (cm/s)
    float _desired_v_z;             // Desired vertical
    int32_t _pitch_target;          // Target pitch attitude to pass to attitude controller
    float _collective_aggression;   // The 'aggresiveness' of collective appliction
    float _z_touch_down_start;      // The height in cm that the touch down phase began
    float _t_touch_down_initiate;   // The time in ms that the touch down phase began
    float now;                      // Current time in millis
    float _entry_time_start;        // Time remaining until entry phase moves on to glide phase
    float _hs_decay;                // The head accerleration during the entry phase
    float _bail_time;               // Timer for exiting the bail out phase (s)
    float _bail_time_start;         // Time at start of bail out
    float _des_z;                   // Desired vertical position
    float _target_climb_rate_adjust;// Target vertical acceleration used during bail out phase
    float _target_pitch_adjust;     // Target pitch rate used during bail out phase
    uint16_t log_counter;           // Used to reduce the data flash logging rate

    enum class Autorotation_Phase {
        ENTRY,
        SS_GLIDE,
        FLARE,
        TOUCH_DOWN,
        BAIL_OUT } phase_switch;
        
    enum class Navigation_Decision {
        USER_CONTROL_STABILISED,
        STRAIGHT_AHEAD,
        INTO_WIND,
        NEAREST_RALLY} nav_pos_switch;

    // --- Internal flags ---
    struct controller_flags {
            bool entry_initial             : 1;
            bool ss_glide_initial          : 1;
            bool flare_initial             : 1;
            bool touch_down_initial        : 1;
            bool straight_ahead_initial    : 1;
            bool level_initial             : 1;
            bool break_initial             : 1;
            bool bail_out_initial          : 1;
            bool bad_rpm                   : 1;
    } _flags;

    struct message_flags {
            bool bad_rpm                   : 1;
    } _msg_flags;

    //--- Internal functions ---
    void warning_message(uint8_t message_n);    //Handles output messages to the terminal

};
#endif
