#pragma once

// this class is #included into the Copter:: namespace

class Mode {
    friend class Copter;
    friend class AP_Arming_Copter;

public:

    Mode(Copter &copter) :
        _copter(copter),
        g(copter.g),
        g2(copter.g2),
        wp_nav(_copter.wp_nav),
        pos_control(_copter.pos_control),
        inertial_nav(_copter.inertial_nav),
        ahrs(_copter.ahrs),
        attitude_control(_copter.attitude_control),
        motors(_copter.motors),
        channel_roll(_copter.channel_roll),
        channel_pitch(_copter.channel_pitch),
        channel_throttle(_copter.channel_throttle),
        channel_yaw(_copter.channel_yaw),
        G_Dt(_copter.G_Dt),
        ap(_copter.ap),
        takeoff_state(_copter.takeoff_state),
        ekfGndSpdLimit(_copter.ekfGndSpdLimit),
        ekfNavVelGainScaler(_copter.ekfNavVelGainScaler),
#if FRAME_CONFIG == HELI_FRAME
        heli_flags(_copter.heli_flags),
#endif
        auto_yaw_mode(_copter.auto_yaw_mode)
        { };

protected:

    virtual bool init(bool ignore_checks) = 0;
    virtual void run() = 0;

    virtual bool is_autopilot() const { return false; }
    virtual bool requires_GPS() const = 0;
    virtual bool has_manual_throttle() const = 0;
    virtual bool allows_arming(bool from_gcs) const = 0;

    virtual bool landing_gear_should_be_deployed() const { return false; }

    virtual const char *name() const = 0;

    // returns a string for this flightmode, exactly 4 bytes
    virtual const char *name4() const = 0;

    // navigation support functions:
    void update_navigation();
    virtual void run_autopilot() {}
    virtual uint32_t wp_distance() const { return 0; }
    virtual int32_t wp_bearing() const { return 0; }

    Copter &_copter;

    // convenience references to avoid code churn in conversion:
    Parameters &g;
    ParametersG2 &g2;
    AC_WPNav *&wp_nav;
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
    ap_t &ap;
    takeoff_state_t &takeoff_state;

    // takeoff support
    bool takeoff_triggered(float target_climb_rate) const;

    // gnd speed limit required to observe optical flow sensor limits
    float &ekfGndSpdLimit;

    // scale factor applied to velocity controller gain to prevent optical flow noise causing excessive angle demand noise
    float &ekfNavVelGainScaler;

    // Navigation Yaw control
    // auto flight mode's yaw mode
    uint8_t &auto_yaw_mode;

#if FRAME_CONFIG == HELI_FRAME
    heli_flags_t &heli_flags;
#endif

    // pass-through functions to reduce code churn on conversion;
    // these are candidates for moving into the Mode base
    // class.
    void get_pilot_desired_lean_angles(float roll_in, float pitch_in, float &roll_out, float &pitch_out, float angle_max, float angle_limit) {
        _copter.get_pilot_desired_lean_angles(roll_in, pitch_in, roll_out, pitch_out, angle_max, angle_limit);
    }
    float get_surface_tracking_climb_rate(int16_t target_rate, float current_alt_target, float dt) {
        return _copter.get_surface_tracking_climb_rate(target_rate, current_alt_target, dt);
    }
    float get_pilot_desired_yaw_rate(int16_t stick_angle) {
        return _copter.get_pilot_desired_yaw_rate(stick_angle);
    }
    float get_pilot_desired_climb_rate(float throttle_control) {
        return _copter.get_pilot_desired_climb_rate(throttle_control);
    }
    float get_pilot_desired_throttle(int16_t throttle_control, float thr_mid = 0.0f) {
        return _copter.get_pilot_desired_throttle(throttle_control, thr_mid);
    }
    float get_non_takeoff_throttle() {
        return _copter.get_non_takeoff_throttle();
    }
    void update_simple_mode(void) {
        _copter.update_simple_mode();
    }
    float get_smoothing_gain() {
        return _copter.get_smoothing_gain();
    }
    bool set_mode(control_mode_t mode, mode_reason_t reason) {
        return _copter.set_mode(mode, reason);
    }
    void set_land_complete(bool b) {
        return _copter.set_land_complete(b);
    }
    GCS_Copter &gcs() {
        return _copter.gcs();
    }
    void Log_Write_Event(uint8_t id) {
        return _copter.Log_Write_Event(id);
    }
    void set_throttle_takeoff() {
        return _copter.set_throttle_takeoff();
    }
    void set_auto_yaw_mode(uint8_t yaw_mode) {
        return _copter.set_auto_yaw_mode(yaw_mode);
    }
    void set_auto_yaw_rate(float turn_rate_cds) {
        return _copter.set_auto_yaw_rate(turn_rate_cds);
    }
    void set_auto_yaw_look_at_heading(float angle_deg, float turn_rate_dps, int8_t direction, bool relative_angle) {
        return _copter.set_auto_yaw_look_at_heading(angle_deg, turn_rate_dps, direction, relative_angle);
    }
    void takeoff_timer_start(float alt_cm) {
        return _copter.takeoff_timer_start(alt_cm);
    }
    void takeoff_stop() {
        return _copter.takeoff_stop();
    }
    void takeoff_get_climb_rates(float& pilot_climb_rate, float& takeoff_climb_rate) {
        return _copter.takeoff_get_climb_rates(pilot_climb_rate, takeoff_climb_rate);
    }
    float get_auto_heading() {
        return _copter.get_auto_heading();
    }
    float get_auto_yaw_rate_cds() {
        return _copter.get_auto_yaw_rate_cds();
    }
    float get_avoidance_adjusted_climbrate(float target_rate) {
        return _copter.get_avoidance_adjusted_climbrate(target_rate);
    }
    uint16_t get_pilot_speed_dn() {
        return _copter.get_pilot_speed_dn();
    }

    // end pass-through functions

    void zero_throttle_and_relax_ac();
};


class ModeAcro : public Mode {

public:

    ModeAcro(Copter &copter) :
        Copter::Mode(copter)
        { }
    virtual bool init(bool ignore_checks) override;
    virtual void run() override;

    virtual bool is_autopilot() const override { return false; }
    virtual bool requires_GPS() const override { return false; }
    virtual bool has_manual_throttle() const override { return true; }
    virtual bool allows_arming(bool from_gcs) const override { return true; };

protected:

    const char *name() const override { return "ACRO"; }
    const char *name4() const override { return "ACRO"; }

    void get_pilot_desired_angle_rates(int16_t roll_in, int16_t pitch_in, int16_t yaw_in, float &roll_out, float &pitch_out, float &yaw_out);

private:

};

#if FRAME_CONFIG == HELI_FRAME
class ModeAcro_Heli : public ModeAcro {

public:

    ModeAcro_Heli(Copter &copter) :
        Copter::ModeAcro(copter)
        { }

    bool init(bool ignore_checks) override;
    void run() override;

protected:
private:
};
#endif


class ModeAltHold : public Mode {

public:

    ModeAltHold(Copter &copter) :
        Copter::Mode(copter)
        { }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; };
    bool is_autopilot() const override { return false; }

protected:

    const char *name() const override { return "ALT_HOLD"; }
    const char *name4() const override { return "ALTH"; }

private:

};


class ModeAuto : public Mode {

public:

    ModeAuto(Copter &copter, AP_Mission &_mission, AC_Circle *& _circle_nav) :
        Copter::Mode(copter),
        mission(_mission),
        circle_nav(_circle_nav)
        { }

    virtual bool init(bool ignore_checks) override;
    virtual void run() override;

    virtual bool is_autopilot() const override { return true; }
    virtual bool requires_GPS() const override { return true; }
    virtual bool has_manual_throttle() const override { return false; }
    virtual bool allows_arming(bool from_gcs) const override { return false; };

    // Auto
    AutoMode mode() const { return _mode; }

    bool loiter_start();
    void rtl_start();
    void takeoff_start(const Location& dest_loc);
    void wp_start(const Vector3f& destination);
    void wp_start(const Location_Class& dest_loc);
    void land_start();
    void land_start(const Vector3f& destination);
    void circle_movetoedge_start(const Location_Class &circle_center, float radius_m);
    void circle_start();
    void spline_start(const Vector3f& destination, bool stopped_at_start, AC_WPNav::spline_segment_end_type seg_end_type, const Vector3f& next_spline_destination);
    void spline_start(const Location_Class& destination, bool stopped_at_start, AC_WPNav::spline_segment_end_type seg_end_type, const Location_Class& next_destination);
    void nav_guided_start();

    bool landing_gear_should_be_deployed() const override;

    void payload_place_start();

    // only out here temporarily
    bool start_command(const AP_Mission::Mission_Command& cmd);
    bool verify_command_callback(const AP_Mission::Mission_Command& cmd);
    void exit_mission();

    // for GCS_MAVLink to call:
    bool do_guided(const AP_Mission::Mission_Command& cmd);

protected:

    const char *name() const override { return "AUTO"; }
    const char *name4() const override { return "AUTO"; }

    uint32_t wp_distance() const override {
        return wp_nav->get_wp_distance_to_destination();
    }
    int32_t wp_bearing() const override {
        return wp_nav->get_wp_bearing_to_destination();
    }
    void run_autopilot() override { mission.update(); }

private:

    bool verify_command(const AP_Mission::Mission_Command& cmd);

    void takeoff_run();
    void wp_run();
    void spline_run();
    void land_run();
    void rtl_run();
    void circle_run();
    void nav_guided_run();
    void loiter_run();

    void payload_place_start(const Vector3f& destination);
    void payload_place_run();
    bool payload_place_run_should_run();
    void payload_place_run_loiter();
    void payload_place_run_descend();
    void payload_place_run_release();

    AutoMode _mode = Auto_TakeOff;   // controls which auto controller is run

    AP_Mission &mission;
    AC_Circle *&circle_nav;

    Location_Class terrain_adjusted_location(const AP_Mission::Mission_Command& cmd) const;

    void do_takeoff(const AP_Mission::Mission_Command& cmd);
    void do_nav_wp(const AP_Mission::Mission_Command& cmd);
    void do_land(const AP_Mission::Mission_Command& cmd);
    void do_loiter_unlimited(const AP_Mission::Mission_Command& cmd);
    void do_circle(const AP_Mission::Mission_Command& cmd);
    void do_loiter_time(const AP_Mission::Mission_Command& cmd);
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
#if CAMERA == ENABLED
    void do_digicam_configure(const AP_Mission::Mission_Command& cmd);
    void do_digicam_control(const AP_Mission::Mission_Command& cmd);
#endif
#if PARACHUTE == ENABLED
    void do_parachute(const AP_Mission::Mission_Command& cmd);
#endif
#if GRIPPER_ENABLED == ENABLED
    void do_gripper(const AP_Mission::Mission_Command& cmd);
#endif
    void do_winch(const AP_Mission::Mission_Command& cmd);
    void do_payload_place(const AP_Mission::Mission_Command& cmd);
    void do_RTL(void);

    bool verify_takeoff();
    bool verify_land();
    bool verify_payload_place();
    bool verify_loiter_unlimited();
    bool verify_loiter_time();
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

    void auto_spline_start(const Location_Class& destination, bool stopped_at_start, AC_WPNav::spline_segment_end_type seg_end_type, const Location_Class& next_destination);

    // Loiter control
    uint16_t loiter_time_max;                // How long we should stay in Loiter Mode for mission scripting (time in seconds)
    uint32_t loiter_time;                    // How long have we been loitering - The start time in millis

    // Delay the next navigation command
    int32_t nav_delay_time_max;  // used for delaying the navigation commands (eg land,takeoff etc.)
    uint32_t nav_delay_time_start;

    // Delay Mission Scripting Command
    int32_t condition_value;  // used in condition commands (eg delay, change alt, etc.)
    uint32_t condition_start;

    LandStateType land_state = LandStateType_FlyToLocation; // records state of land (flying to location, descending)

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
class ModeAutoTune : public Mode {

public:

    ModeAutoTune(Copter &copter) :
        Copter::Mode(copter)
        { }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return false; }
    bool is_autopilot() const override { return false; }

    void save_tuning_gains();

    void stop();

protected:

    const char *name() const override { return "AUTOTUNE"; }
    const char *name4() const override { return "ATUN"; }

private:

    bool start(bool ignore_checks);

    void autotune_attitude_control();
    void backup_gains_and_initialise();
    void load_orig_gains();
    void load_tuned_gains();
    void load_intra_test_gains();
    void load_twitch_gains();
    void update_gcs(uint8_t message_id);
    bool roll_enabled();
    bool pitch_enabled();
    bool yaw_enabled();
    void twitching_test(float measurement, float target, float &measurement_min, float &measurement_max);
    void updating_d_up(float &tune_d, float tune_d_min, float tune_d_max, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float target, float measurement_min, float measurement_max);
    void updating_d_down(float &tune_d, float tune_d_min, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float target, float measurement_min, float measurement_max);
    void updating_p_down(float &tune_p, float tune_p_min, float tune_p_step_ratio, float target, float measurement_max);
    void updating_p_up(float &tune_p, float tune_p_max, float tune_p_step_ratio, float target, float measurement_max);
    void updating_p_up_d_down(float &tune_d, float tune_d_min, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float target, float measurement_min, float measurement_max);
    void twitching_measure_acceleration(float &rate_of_change, float rate_measurement, float &rate_measurement_max);
    void get_poshold_attitude(float &roll_cd, float &pitch_cd, float &yaw_cd);

    void Log_Write_AutoTune(uint8_t axis, uint8_t tune_step, float meas_target, float meas_min, float meas_max, float new_gain_rp, float new_gain_rd, float new_gain_sp, float new_ddt);
    void Log_Write_AutoTuneDetails(float angle_cd, float rate_cds);

    void send_step_string();
    const char *level_issue_string() const;
    const char * type_string() const;
    void announce_state_to_gcs();
    void do_gcs_announcements();

    enum LEVEL_ISSUE {
        LEVEL_ISSUE_NONE,
        LEVEL_ISSUE_ANGLE_ROLL,
        LEVEL_ISSUE_ANGLE_PITCH,
        LEVEL_ISSUE_ANGLE_YAW,
        LEVEL_ISSUE_RATE_ROLL,
        LEVEL_ISSUE_RATE_PITCH,
        LEVEL_ISSUE_RATE_YAW,
    };
    bool check_level(const enum LEVEL_ISSUE issue, const float current, const float maximum);
    bool currently_level();

    // autotune modes (high level states)
    enum TuneMode {
        UNINITIALISED = 0,        // autotune has never been run
        TUNING = 1,               // autotune is testing gains
        SUCCESS = 2,              // tuning has completed, user is flight testing the new gains
        FAILED = 3,               // tuning has failed, user is flying on original gains
    };

    // steps performed while in the tuning mode
    enum StepType {
        WAITING_FOR_LEVEL = 0,    // autotune is waiting for vehicle to return to level before beginning the next twitch
        TWITCHING = 1,            // autotune has begun a twitch and is watching the resulting vehicle movement
        UPDATE_GAINS = 2          // autotune has completed a twitch and is updating the gains based on the results
    };

    // things that can be tuned
    enum AxisType {
        ROLL = 0,                 // roll axis is being tuned (either angle or rate)
        PITCH = 1,                // pitch axis is being tuned (either angle or rate)
        YAW = 2,                  // pitch axis is being tuned (either angle or rate)
    };

    // mini steps performed while in Tuning mode, Testing step
    enum TuneType {
        RD_UP = 0,                // rate D is being tuned up
        RD_DOWN = 1,              // rate D is being tuned down
        RP_UP = 2,                // rate P is being tuned up
        SP_DOWN = 3,              // angle P is being tuned down
        SP_UP = 4                 // angle P is being tuned up
    };

    TuneMode mode                : 2;    // see TuneMode for what modes are allowed
    bool     pilot_override      : 1;    // true = pilot is overriding controls so we suspend tuning temporarily
    AxisType axis                : 2;    // see AxisType for which things can be tuned
    bool     positive_direction  : 1;    // false = tuning in negative direction (i.e. left for roll), true = positive direction (i.e. right for roll)
    StepType step                : 2;    // see StepType for what steps are performed
    TuneType tune_type           : 3;    // see TuneType
    bool     ignore_next         : 1;    // true = ignore the next test
    bool     twitch_first_iter   : 1;    // true on first iteration of a twitch (used to signal we must step the attitude or rate target)
    bool     use_poshold         : 1;    // true = enable position hold
    bool     have_position       : 1;    // true = start_position is value
    Vector3f start_position;

// variables
    uint32_t override_time;                         // the last time the pilot overrode the controls
    float    test_min;                              // the minimum angular rate achieved during TESTING_RATE step
    float    test_max;                              // the maximum angular rate achieved during TESTING_RATE step
    uint32_t step_start_time;                       // start time of current tuning step (used for timeout checks)
    uint32_t step_stop_time;                        // start time of current tuning step (used for timeout checks)
    int8_t   counter;                               // counter for tuning gains
    float    target_rate, start_rate;      // target and start rate
    float    target_angle, start_angle;    // target and start angles
    float    desired_yaw;                           // yaw heading during tune
    float    rate_max, test_accel_max;              // maximum acceleration variables

    LowPassFilterFloat  rotation_rate_filt;                         // filtered rotation rate in radians/second

// backup of currently being tuned parameter values
    float    orig_roll_rp = 0, orig_roll_ri, orig_roll_rd, orig_roll_sp, orig_roll_accel;
    float    orig_pitch_rp = 0, orig_pitch_ri, orig_pitch_rd, orig_pitch_sp, orig_pitch_accel;
    float    orig_yaw_rp = 0, orig_yaw_ri, orig_yaw_rd, orig_yaw_rLPF, orig_yaw_sp, orig_yaw_accel;
    bool     orig_bf_feedforward;

// currently being tuned parameter values
    float    tune_roll_rp, tune_roll_rd, tune_roll_sp, tune_roll_accel;
    float    tune_pitch_rp, tune_pitch_rd, tune_pitch_sp, tune_pitch_accel;
    float    tune_yaw_rp, tune_yaw_rLPF, tune_yaw_sp, tune_yaw_accel;

    uint32_t announce_time;
    float lean_angle;
    float rotation_rate;
    float roll_cd, pitch_cd;

    struct {
        LEVEL_ISSUE issue{LEVEL_ISSUE_NONE};
        float maximum;
        float current;
    } level_problem;

};
#endif


class ModeBrake : public Mode {

public:

    ModeBrake(Copter &copter) :
        Copter::Mode(copter)
        { }

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

    uint32_t _timeout_start;
    uint32_t _timeout_ms;

};


class ModeCircle : public Mode {

public:

    ModeCircle(Copter &copter, AC_Circle *& _circle_nav) :
        Copter::Mode(copter),
        circle_nav(_circle_nav)
        { }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return false; };
    bool is_autopilot() const override { return true; }

protected:

    const char *name() const override { return "CIRCLE"; }
    const char *name4() const override { return "CIRC"; }

    uint32_t wp_distance() const override {
        return wp_nav->get_loiter_distance_to_target();
    }
    int32_t wp_bearing() const override {
        return wp_nav->get_loiter_bearing_to_target();
    }

private:

    // Circle
    bool pilot_yaw_override = false; // true if pilot is overriding yaw
    AC_Circle *&circle_nav;

};


class ModeDrift : public Mode {

public:

    ModeDrift(Copter &copter) :
        Copter::Mode(copter)
        { }

    virtual bool init(bool ignore_checks) override;
    virtual void run() override;

    virtual bool requires_GPS() const override { return true; }
    virtual bool has_manual_throttle() const override { return false; }
    virtual bool allows_arming(bool from_gcs) const override { return true; };
    virtual bool is_autopilot() const override { return false; }

protected:

    const char *name() const override { return "DRIFT"; }
    const char *name4() const override { return "DRIF"; }

private:

    float get_throttle_assist(float velz, float pilot_throttle_scaled);

};


class ModeFlip : public Mode {

public:

    ModeFlip(Copter &copter) :
        Copter::Mode(copter)
        { }

    virtual bool init(bool ignore_checks) override;
    virtual void run() override;

    virtual bool requires_GPS() const override { return false; }
    virtual bool has_manual_throttle() const override { return false; }
    virtual bool allows_arming(bool from_gcs) const override { return false; };
    virtual bool is_autopilot() const override { return false; }

protected:

    const char *name() const override { return "FLIP"; }
    const char *name4() const override { return "FLIP"; }

private:

    // Flip
    Vector3f flip_orig_attitude;         // original vehicle attitude before flip

};


class ModeGuided : public Mode {

public:

    ModeGuided(Copter &copter) :
        Copter::Mode(copter)        { }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return from_gcs; }
    bool is_autopilot() const override { return true; }

    void set_angle(const Quaternion &q, float climb_rate_cms, bool use_yaw_rate, float yaw_rate_rads);
    bool set_destination(const Vector3f& destination, bool use_yaw = false, float yaw_cd = 0.0, bool use_yaw_rate = false, float yaw_rate_cds = 0.0, bool yaw_relative = false);
    bool set_destination(const Location_Class& dest_loc, bool use_yaw = false, float yaw_cd = 0.0, bool use_yaw_rate = false, float yaw_rate_cds = 0.0, bool yaw_relative = false);
    void set_velocity(const Vector3f& velocity, bool use_yaw = false, float yaw_cd = 0.0, bool use_yaw_rate = false, float yaw_rate_cds = 0.0, bool yaw_relative = false);
    bool set_destination_posvel(const Vector3f& destination, const Vector3f& velocity, bool use_yaw = false, float yaw_cd = 0.0, bool use_yaw_rate = false, float yaw_rate_cds = 0.0, bool yaw_relative = false);

    void limit_clear();
    void limit_init_time_and_pos();
    void limit_set(uint32_t timeout_ms, float alt_min_cm, float alt_max_cm, float horiz_max_cm);
    bool limit_check();

    bool takeoff_start(float final_alt_above_home);

    GuidedMode mode() const { return guided_mode; }

    void angle_control_start();
    void angle_control_run();

protected:

    const char *name() const override { return "GUIDED"; }
    const char *name4() const override { return "GUID"; }

    uint32_t wp_distance() const override;
    int32_t wp_bearing() const override;

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

    ModeGuidedNoGPS(Copter &copter) :
        Copter::ModeGuided(copter)        { }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
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

    ModeLand(Copter &copter) :
        Copter::Mode(copter)
        { }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return false; };
    bool is_autopilot() const override { return true; }
    bool landing_gear_should_be_deployed() const override { return true; };

    float get_land_descent_speed();
    bool landing_with_GPS();
    void do_not_use_GPS();

    int32_t get_alt_above_ground(void);

protected:

    const char *name() const override { return "LAND"; }
    const char *name4() const override { return "LAND"; }

private:

    void gps_run();
    void nogps_run();
};


class ModeLoiter : public Mode {

public:

    ModeLoiter(Copter &copter) :
        Copter::Mode(copter)
        { }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; };
    bool is_autopilot() const override { return false; }

#if PRECISION_LANDING == ENABLED
    void set_precision_loiter_enabled(bool value) { _precision_loiter_enabled = value; }
#endif

protected:

    const char *name() const override { return "LOITER"; }
    const char *name4() const override { return "LOIT"; }

    uint32_t wp_distance() const override {
        return wp_nav->get_loiter_distance_to_target();
    }
    int32_t wp_bearing() const override {
        return wp_nav->get_loiter_bearing_to_target();
    }

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

    ModePosHold(Copter &copter) :
        Copter::Mode(copter)
        { }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; };
    bool is_autopilot() const override { return false; }

protected:

    const char *name() const override { return "POSHOLD"; }
    const char *name4() const override { return "PHLD"; }

private:

    void poshold_update_pilot_lean_angle(float &lean_angle_filtered, float &lean_angle_raw);
    int16_t poshold_mix_controls(float mix_ratio, int16_t first_control, int16_t second_control);
    void poshold_update_brake_angle_from_velocity(int16_t &brake_angle, float velocity);
    void poshold_update_wind_comp_estimate();
    void poshold_get_wind_comp_lean_angles(int16_t &roll_angle, int16_t &pitch_angle);
    void poshold_roll_controller_to_pilot_override();
    void poshold_pitch_controller_to_pilot_override();

};


class ModeRTL : public Mode {

public:

    ModeRTL(Copter &copter) :
        Copter::Mode(copter)
        { }

    bool init(bool ignore_checks) override;
    void run() override {
        return run(true);
    }
    void run(bool disarm_on_land);

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return false; };
    bool is_autopilot() const override { return true; }

    RTLState state() { return _state; }

    // this should probably not be exposed
    bool state_complete() { return _state_complete; }

    bool landing_gear_should_be_deployed() const override;

    void restart_without_terrain();

protected:

    const char *name() const override { return "RTL"; }
    const char *name4() const override { return "RTL "; }

    uint32_t wp_distance() const override {
        return wp_nav->get_wp_distance_to_destination();
    }
    int32_t wp_bearing() const override {
        return wp_nav->get_wp_bearing_to_destination();
    }

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
    void build_path(bool terrain_following_allowed);
    void compute_return_target(bool terrain_following_allowed);

    // RTL
    RTLState _state = RTL_InitialClimb;  // records state of rtl (initial climb, returning home, etc)
    bool _state_complete = false; // set to true if the current state is completed

    struct {
        // NEU w/ Z element alt-above-ekf-origin unless use_terrain is true in which case Z element is alt-above-terrain
        Location_Class origin_point;
        Location_Class climb_target;
        Location_Class return_target;
        Location_Class descent_target;
        bool land;
        bool terrain_used;
    } rtl_path;

    // Loiter timer - Records how long we have been in loiter
    uint32_t _loiter_start_time = 0;
};


class ModeSmartRTL : public ModeRTL {

public:

    ModeSmartRTL(Copter &copter) :
        ModeSmartRTL::ModeRTL(copter)
        { }

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

    uint32_t wp_distance() const override {
        return wp_nav->get_wp_distance_to_destination();
    }
    int32_t wp_bearing() const override {
        return wp_nav->get_wp_bearing_to_destination();
    }

private:

    void wait_cleanup_run();
    void path_follow_run();
    void pre_land_position_run();
    void land();
    SmartRTLState smart_rtl_state = SmartRTL_PathFollow;

};


class ModeSport : public Mode {

public:

    ModeSport(Copter &copter) :
        Copter::Mode(copter)
        { }

    virtual bool init(bool ignore_checks) override;
    virtual void run() override;

    virtual bool requires_GPS() const override { return false; }
    virtual bool has_manual_throttle() const override { return false; }
    virtual bool allows_arming(bool from_gcs) const override { return true; };
    virtual bool is_autopilot() const override { return false; }

protected:

    const char *name() const override { return "SPORT"; }
    const char *name4() const override { return "SPRT"; }

private:

};


class ModeStabilize : public Mode {

public:

    ModeStabilize(Copter &copter) :
        Copter::Mode(copter)
        { }

    virtual bool init(bool ignore_checks) override;
    virtual void run() override;

    virtual bool requires_GPS() const override { return false; }
    virtual bool has_manual_throttle() const override { return true; }
    virtual bool allows_arming(bool from_gcs) const override { return true; };
    virtual bool is_autopilot() const override { return false; }

protected:

    const char *name() const override { return "STABILIZE"; }
    const char *name4() const override { return "STAB"; }

private:

};

#if FRAME_CONFIG == HELI_FRAME
class ModeStabilize_Heli : public ModeStabilize {

public:

    ModeStabilize_Heli(Copter &copter) :
        Copter::ModeStabilize(copter)
        { }

    bool init(bool ignore_checks) override;
    void run() override;

protected:

private:

};
#endif


class ModeThrow : public Mode {

public:

    ModeThrow(Copter &copter) :
        Copter::Mode(copter)
        { }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; };
    bool is_autopilot() const override { return false; }


protected:

    const char *name() const override { return "THROW"; }
    const char *name4() const override { return "THRW"; }

private:

    bool throw_detected();
    bool throw_position_good();
    bool throw_height_good();
    bool throw_attitude_good();

    ThrowModeStage stage = Throw_Disarmed;
    ThrowModeStage prev_stage = Throw_Disarmed;
    uint32_t last_log_ms;
    bool nextmode_attempted;
    uint32_t free_fall_start_ms;    // system time free fall was detected
    float free_fall_start_velz;     // vertical velocity when free fall was detected

};


class ModeAvoidADSB : public ModeGuided {

public:

    ModeAvoidADSB(Copter &copter) :
        Copter::ModeGuided(copter) { }

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
