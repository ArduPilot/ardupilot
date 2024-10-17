#pragma once

#include "Rover.h"

// pre-define ModeRTL so Auto can appear higher in this file
class ModeRTL;

class Mode
{
public:

    // Auto Pilot modes
    // ----------------
    enum class Number : uint8_t {
        MANUAL       = 0,
        ACRO         = 1,
        STEERING     = 3,
        HOLD         = 4,
        LOITER       = 5,
        FOLLOW       = 6,
        SIMPLE       = 7,
#if MODE_DOCK_ENABLED
        DOCK         = 8,
#endif
        CIRCLE       = 9,
        AUTO         = 10,
        RTL          = 11,
        SMART_RTL    = 12,
        GUIDED       = 15,
        INITIALISING = 16,
    };

    // Constructor
    Mode();

    // do not allow copying
    CLASS_NO_COPY(Mode);

    // enter this mode, returns false if we failed to enter
    bool enter();

    // perform any cleanups required:
    void exit();

    // returns a unique number specific to this mode
    virtual Number mode_number() const = 0;

    // returns short text name (up to 4 bytes)
    virtual const char *name4() const = 0;

    //
    // methods that sub classes should override to affect movement of the vehicle in this mode
    //

    // convert user input to targets, implement high level control for this mode
    virtual void update() = 0;

    //
    // attributes of the mode
    //

    // return if in non-manual mode : Auto, Guided, RTL, SmartRTL
    virtual bool is_autopilot_mode() const { return false; }

    // return if external control is allowed in this mode (Guided or Guided-within-Auto)
    virtual bool in_guided_mode() const { return false; }

    // returns true if vehicle can be armed or disarmed from the transmitter in this mode
    virtual bool allows_arming_from_transmitter() { return !is_autopilot_mode(); }

    // returns false if vehicle cannot be armed in this mode
    virtual bool allows_arming() const { return true; }

    bool allows_stick_mixing() const { return is_autopilot_mode(); }

    //
    // attributes for mavlink system status reporting
    //

    // returns true if any RC input is used
    virtual bool has_manual_input() const { return false; }

    // true if heading is controlled
    virtual bool attitude_stabilized() const { return true; }

    // true if mode requires position and/or velocity estimate
    virtual bool requires_position() const { return true; }
    virtual bool requires_velocity() const { return true; }

    // return heading (in degrees) and cross track error (in meters) for reporting to ground station (NAV_CONTROLLER_OUTPUT message)
    virtual float wp_bearing() const;
    virtual float nav_bearing() const;
    virtual float crosstrack_error() const;
    virtual float get_desired_lat_accel() const;

    // get speed error in m/s, not currently supported
    float speed_error() const { return 0.0f; }

    //
    // navigation methods
    //

    // return distance (in meters) to destination
    virtual float get_distance_to_destination() const { return 0.0f; }

    // return desired location (used in Guided, Auto, RTL, etc)
    // return true on success, false if there is no valid destination
    virtual bool get_desired_location(Location& destination) const WARN_IF_UNUSED { return false; }

    // set desired location (used in Guided, Auto)
    // set next_destination (if known).  If not provided vehicle stops at destination
    virtual bool set_desired_location(const Location &destination, Location next_destination = Location()) WARN_IF_UNUSED;

    // true if vehicle has reached desired location. defaults to true because this is normally used by missions and we do not want the mission to become stuck
    virtual bool reached_destination() const { return true; }

    // get default speed for this mode (held in CRUISE_SPEED, WP_SPEED or RTL_SPEED)
    // rtl argument should be true if called from RTL or SmartRTL modes (handled here to avoid duplication)
    float get_speed_default(bool rtl = false) const;

    // set desired speed in m/s
    virtual bool set_desired_speed(float speed) { return false; }

    // execute the mission in reverse (i.e. backing up)
    void set_reversed(bool value);

    // init reversed flag for autopilot mode
    virtual void init_reversed_flag() { if (is_autopilot_mode()) { set_reversed(false); } }

    // handle tacking request (from auxiliary switch) in sailboats
    virtual void handle_tack_request();

protected:

    // subclasses override this to perform checks before entering the mode
    virtual bool _enter() { return true; }

    // subclasses override this to perform any required cleanup when exiting the mode
    virtual void _exit() { return; }

    // decode pilot steering and throttle inputs and return in steer_out and throttle_out arguments
    // steering_out is in the range -4500 ~ +4500 with positive numbers meaning rotate clockwise
    // throttle_out is in the range -100 ~ +100
    void get_pilot_desired_steering_and_throttle(float &steering_out, float &throttle_out) const;

    // decode pilot input steering and return steering_out and speed_out (in m/s)
    void get_pilot_desired_steering_and_speed(float &steering_out, float &speed_out) const;

    // decode pilot lateral movement input and return in lateral_out argument
    void get_pilot_desired_lateral(float &lateral_out) const;

    // decode pilot's input and return heading_out (in cd) and speed_out (in m/s)
    void get_pilot_desired_heading_and_speed(float &heading_out, float &speed_out) const;

    // decode pilot roll and pitch inputs and return in roll_out and pitch_out arguments
    // outputs are in the range -1 to +1
    void get_pilot_desired_roll_and_pitch(float &roll_out, float &pitch_out) const;

    // decode pilot height inputs and return in height_out arguments
    // outputs are in the range -1 to +1
    void get_pilot_desired_walking_height(float &walking_height_out) const;

    // high level call to navigate to waypoint
    void navigate_to_waypoint();

    // calculate steering output given a turn rate
    // desired turn rate in radians/sec. Positive to the right.
    void calc_steering_from_turn_rate(float turn_rate);

    // calculate steering angle given a desired lateral acceleration
    void calc_steering_from_lateral_acceleration(float lat_accel, bool reversed = false);

    // calculate steering output to drive towards desired heading
    // rate_max is a maximum turn rate in deg/s.  set to zero to use default turn rate limits
    void calc_steering_to_heading(float desired_heading_cd, float rate_max_degs = 0.0f);

    // calculates the amount of throttle that should be output based
    // on things like proximity to corners and current speed
    virtual void calc_throttle(float target_speed, bool avoidance_enabled);

    // performs a controlled stop. returns true once vehicle has stopped
    bool stop_vehicle();

    // estimate maximum vehicle speed (in m/s)
    // cruise_speed is in m/s, cruise_throttle should be in the range -1 to +1
    float calc_speed_max(float cruise_speed, float cruise_throttle) const;

    // calculate pilot input to nudge speed up or down
    //  target_speed should be in meters/sec
    //  reversed should be true if the vehicle is intentionally backing up which allows the pilot to increase the backing up speed by pulling the throttle stick down
    float calc_speed_nudge(float target_speed, bool reversed);

protected:

    // decode pilot steering and throttle inputs and return in steer_out and throttle_out arguments
    // steering_out is in the range -4500 ~ +4500 with positive numbers meaning rotate clockwise
    // throttle_out is in the range -100 ~ +100
    void get_pilot_input(float &steering_out, float &throttle_out) const;
    void set_steering(float steering_value);

    // references to avoid code churn:
    class AP_AHRS &ahrs;
    class Parameters &g;
    class ParametersG2 &g2;
    class RC_Channel *&channel_steer;
    class RC_Channel *&channel_throttle;
    class RC_Channel *&channel_lateral;
    class RC_Channel *&channel_roll;
    class RC_Channel *&channel_pitch;
    class RC_Channel *&channel_walking_height;
    class AR_AttitudeControl &attitude_control;

    // private members for waypoint navigation
    float _distance_to_destination; // distance from vehicle to final destination in meters
    bool _reached_destination;  // true once the vehicle has reached the destination
    float _desired_yaw_cd;      // desired yaw in centi-degrees.  used in Auto, Guided and Loiter
};


class ModeAcro : public Mode
{
public:

    Number mode_number() const override { return Number::ACRO; }
    const char *name4() const override { return "ACRO"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // attributes for mavlink system status reporting
    bool has_manual_input() const override { return true; }

    // acro mode requires a velocity estimate for non skid-steer rovers
    bool requires_position() const override { return false; }
    bool requires_velocity() const override;

    // sailboats in acro mode support user manually initiating tacking from transmitter
    void handle_tack_request() override;
};


class ModeAuto : public Mode
{
public:

    Number mode_number() const override { return Number::AUTO; }
    const char *name4() const override { return "AUTO"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;
    void calc_throttle(float target_speed, bool avoidance_enabled) override;

    // attributes of the mode
    bool is_autopilot_mode() const override { return true; }

    // return if external control is allowed in this mode (Guided or Guided-within-Auto)
    bool in_guided_mode() const override { return _submode == SubMode::Guided || _submode == SubMode::NavScriptTime; }

    // return heading (in degrees) and cross track error (in meters) for reporting to ground station (NAV_CONTROLLER_OUTPUT message)
    float wp_bearing() const override;
    float nav_bearing() const override;
    float crosstrack_error() const override;
    float get_desired_lat_accel() const override;

    // return distance (in meters) to destination
    float get_distance_to_destination() const override;

    // get or set desired location
    bool get_desired_location(Location& destination) const override WARN_IF_UNUSED;
    bool set_desired_location(const Location &destination, Location next_destination = Location()) override WARN_IF_UNUSED;
    bool reached_destination() const override;

    // set desired speed in m/s
    bool set_desired_speed(float speed) override;

    // start RTL (within auto)
    void start_RTL();

    // lua accessors for nav script time support
    bool nav_script_time(uint16_t &id, uint8_t &cmd, float &arg1, float &arg2, int16_t &arg3, int16_t &arg4);
    void nav_script_time_done(uint16_t id);

    // 
    void init_reversed_flag() override {
        if (!mission.is_resume()) {
            set_reversed(false);
        }
    }

    AP_Mission mission{
        FUNCTOR_BIND_MEMBER(&ModeAuto::start_command, bool, const AP_Mission::Mission_Command&),
        FUNCTOR_BIND_MEMBER(&ModeAuto::verify_command_callback, bool, const AP_Mission::Mission_Command&),
        FUNCTOR_BIND_MEMBER(&ModeAuto::exit_mission, void)};

    enum class DoneBehaviour : uint8_t {
        HOLD      = 0,
        LOITER    = 1,
        ACRO      = 2,
        MANUAL    = 3,
    };

protected:

    bool _enter() override;
    void _exit() override;

    enum SubMode: uint8_t {
        WP,                // drive to a given location
        HeadingAndSpeed,   // turn to a given heading
        RTL,               // perform RTL within auto mode
        Loiter,            // perform Loiter within auto mode
        Guided,            // handover control to external navigation system from within auto mode
        Stop,              // stop the vehicle as quickly as possible
        NavScriptTime,     // accept targets from lua scripts while NAV_SCRIPT_TIME commands are executing
        Circle,            // circle a given location
    } _submode;

private:

    bool check_trigger(void);
    bool start_loiter();
    void start_guided(const Location& target_loc);
    void start_stop();
    void send_guided_position_target();

    bool start_command(const AP_Mission::Mission_Command& cmd);
    void exit_mission();
    bool verify_command_callback(const AP_Mission::Mission_Command& cmd);

    bool verify_command(const AP_Mission::Mission_Command& cmd);
    void do_RTL(void);
    bool do_nav_wp(const AP_Mission::Mission_Command& cmd, bool always_stop_at_destination);
    void do_nav_guided_enable(const AP_Mission::Mission_Command& cmd);
    void do_nav_set_yaw_speed(const AP_Mission::Mission_Command& cmd);
    void do_nav_delay(const AP_Mission::Mission_Command& cmd);
    bool verify_nav_delay(const AP_Mission::Mission_Command& cmd);
    bool verify_nav_wp(const AP_Mission::Mission_Command& cmd);
    bool verify_RTL() const;
    bool verify_loiter_unlimited(const AP_Mission::Mission_Command& cmd);
    bool verify_loiter_time(const AP_Mission::Mission_Command& cmd);
    bool verify_nav_guided_enable(const AP_Mission::Mission_Command& cmd);
    bool verify_nav_set_yaw_speed();
    bool do_circle(const AP_Mission::Mission_Command& cmd);
    bool verify_circle(const AP_Mission::Mission_Command& cmd);
    void do_wait_delay(const AP_Mission::Mission_Command& cmd);
    void do_within_distance(const AP_Mission::Mission_Command& cmd);
    bool verify_wait_delay();
    bool verify_within_distance();
    void do_change_speed(const AP_Mission::Mission_Command& cmd);
    void do_set_home(const AP_Mission::Mission_Command& cmd);
    void do_set_reverse(const AP_Mission::Mission_Command& cmd);
    void do_guided_limits(const AP_Mission::Mission_Command& cmd);
#if AP_SCRIPTING_ENABLED
    void do_nav_script_time(const AP_Mission::Mission_Command& cmd);
    bool verify_nav_script_time();
#endif

    bool waiting_to_start;  // true if waiting for EKF origin before starting mission
    bool auto_triggered;        // true when auto has been triggered to start

    // HeadingAndSpeed sub mode variables
    float _desired_speed;   // desired speed in HeadingAndSpeed submode
    bool _reached_heading;  // true when vehicle has reached desired heading in TurnToHeading sub mode

    // Loiter control
    uint16_t loiter_duration;       // How long we should loiter at the nav_waypoint (time in seconds)
    uint32_t loiter_start_time;     // How long have we been loitering - The start time in millis
    bool previously_reached_wp;     // set to true if we have EVER reached the waypoint

    // Guided-within-Auto variables
    struct {
        Location loc;           // location target sent to external navigation
        bool valid;             // true if loc is valid
        uint32_t last_sent_ms;  // system time that target was last sent to offboard navigation
    } guided_target;

    // Conditional command
    // A value used in condition commands (eg delay, change alt, etc.)
    // For example in a change altitude command, it is the altitude to change to.
    int32_t condition_value;
    // A starting value used to check the status of a conditional command.
    // For example in a delay command the condition_start records that start time for the delay
    int32_t condition_start;

    // Delay the next navigation command
    uint32_t nav_delay_time_max_ms;  // used for delaying the navigation commands
    uint32_t nav_delay_time_start_ms;

#if AP_SCRIPTING_ENABLED
    // nav_script_time command variables
    struct {
        bool done;          // true once lua script indicates it has completed
        uint16_t id;        // unique id to avoid race conditions between commands and lua scripts
        uint32_t start_ms;  // system time nav_script_time command was received (used for timeout)
        uint8_t command;    // command number provided by mission command
        uint8_t timeout_s;  // timeout (in seconds) provided by mission command
        float arg1;         // 1st argument provided by mission command
        float arg2;         // 2nd argument provided by mission command
        int16_t arg3;       // 3rd argument provided by mission command
        int16_t arg4;       // 4th argument provided by mission command
    } nav_scripting;
#endif

    // Mission change detector
    AP_Mission_ChangeDetector mis_change_detector;
};

class ModeCircle : public Mode
{
public:

    // need a constructor for parameters
    ModeCircle();

    // Does not allow copies
    CLASS_NO_COPY(ModeCircle);

    Number mode_number() const override { return Number::CIRCLE; }
    const char *name4() const override { return "CIRC"; }

    // initialise with specific center location, radius (in meters) and direction
    // replaces use of _enter when initialised from within Auto mode
    bool set_center(const Location& center_loc, float radius_m, bool dir_ccw);

    // methods that affect movement of the vehicle in this mode
    void update() override;

    bool is_autopilot_mode() const override { return true; }

    // return desired heading (in degrees) and cross track error (in meters) for reporting to ground station (NAV_CONTROLLER_OUTPUT message)
    float wp_bearing() const override;
    float nav_bearing() const override;
    float crosstrack_error() const override { return dist_to_edge_m; }
    float get_desired_lat_accel() const override;

    // set desired speed in m/s
    bool set_desired_speed(float speed_ms) override;

    // return distance (in meters) to destination
    float get_distance_to_destination() const override { return _distance_to_destination; }

    // get or set desired location
    bool get_desired_location(Location& destination) const override WARN_IF_UNUSED;

    // return total angle in radians that vehicle has circled
    // fabsf is used so that full rotations in either direction are counted
    float get_angle_total_rad() const { return fabsf(angle_total_rad); }

    static const struct AP_Param::GroupInfo var_info[];

protected:

    AP_Float radius;        // circle radius in meters
    AP_Float speed;         // vehicle speed in m/s.  If zero uses WP_SPEED
    AP_Int8 direction;      // direction 0:clockwise, 1:counter-clockwise

    // initialise mode
    bool _enter() override;

    // Update position controller targets driving to the circle edge
    void update_drive_to_radius();

    // Update position controller targets while circling
    void update_circling();

    // initialise target_yaw_rad using the vehicle's position and yaw
    // if there is no current position estimate target_yaw_rad is set to vehicle yaw
    void init_target_yaw_rad();

    // limit config speed so that lateral acceleration is within limits
    // outputs warning to user if speed is reduced
    void check_config_speed();

    // ensure config radius is no smaller then vehicle's TURN_RADIUS
    // radius is increased if necessary and warning is output to the user
    void check_config_radius();

    // enum for Direction parameter
    enum class Direction {
        CW = 0,
        CCW = 1
    };

    // local members
    struct {
        Location center_loc;    // circle center as a Location
        Vector2f center_pos;    // circle center as an offset (in meters) from the EKF origin
        float radius;   // circle radius
        float speed;    // desired speed around circle in m/s
        Direction dir;  // direction, 0:clockwise, 1:counter-clockwise
    } config;
    struct {
        float speed;    // vehicle's target speed around circle in m/s
        float yaw_rad;  // earth-frame angle of tarrget point on the circle
        Vector2p pos;   // latest position target sent to position controller
        Vector2f vel;   // latest velocity target sent to position controller
        Vector2f accel; // latest accel target sent to position controller
    } target;
    float angle_total_rad;  // total angle in radians that vehicle has circled
    bool reached_edge;      // true once vehicle has reached edge of circle
    float dist_to_edge_m;   // distance to edge of circle in meters (equivalent to crosstrack error)
    bool tracking_back;     // true if the vehicle is trying to track back onto the circle
};

class ModeGuided : public Mode
{
public:
#if AP_EXTERNAL_CONTROL_ENABLED
    friend class AP_ExternalControl_Rover;
#endif

    Number mode_number() const override { return Number::GUIDED; }
    const char *name4() const override { return "GUID"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // attributes of the mode
    bool is_autopilot_mode() const override { return true; }

    // return if external control is allowed in this mode (Guided or Guided-within-Auto)
    bool in_guided_mode() const override { return true; }

    // return heading (in degrees) and cross track error (in meters) for reporting to ground station (NAV_CONTROLLER_OUTPUT message)
    float wp_bearing() const override;
    float nav_bearing() const override;
    float crosstrack_error() const override;
    float get_desired_lat_accel() const override;

    // return distance (in meters) to destination
    float get_distance_to_destination() const override;

    // return true if vehicle has reached destination
    bool reached_destination() const override;

    // set desired speed in m/s
    bool set_desired_speed(float speed) override;

    // get or set desired location
    bool get_desired_location(Location& destination) const override WARN_IF_UNUSED;
    bool set_desired_location(const Location &destination, Location next_destination = Location()) override WARN_IF_UNUSED;

    // set desired heading and speed
    void set_desired_heading_and_speed(float yaw_angle_cd, float target_speed);

    // set desired heading-delta, turn-rate and speed
    void set_desired_heading_delta_and_speed(float yaw_delta_cd, float target_speed);
    void set_desired_turn_rate_and_speed(float turn_rate_cds, float target_speed);

    // set steering and throttle (-1 to +1).  Only called from scripts
    void set_steering_and_throttle(float steering, float throttle);

    // vehicle start loiter
    bool start_loiter();

    // start stopping
    void start_stop();

    // guided limits
    void limit_set(uint32_t timeout_ms, float horiz_max);
    void limit_clear();
    void limit_init_time_and_location();
    bool limit_breached() const;

protected:

    enum class SubMode: uint8_t {
        WP,
        HeadingAndSpeed,
        TurnRateAndSpeed,
        Loiter,
        SteeringAndThrottle,
        Stop
    };

    // enum for GUID_OPTIONS parameter
    enum class Options : int32_t {
        SCurvesUsedForNavigation = (1U << 6)
    };

    bool _enter() override;

    // returns true if GUID_OPTIONS bit set to use scurve navigation instead of position controller input shaping
    // scurves provide path planning and object avoidance but cannot handle fast updates to the destination (for fast updates use position controller input shaping)
    bool use_scurves_for_navigation() const;

    SubMode _guided_mode;    // stores which GUIDED mode the vehicle is in

    // attitude control
    bool have_attitude_target;  // true if we have a valid attitude target
    uint32_t _des_att_time_ms;  // system time last call to set_desired_attitude was made (used for timeout)
    float _desired_yaw_rate_cds;// target turn rate centi-degrees per second
    bool send_notification;     // used to send one time notification to ground station
    float _desired_speed;       // desired speed used only in HeadingAndSpeed submode

    // direct steering and throttle control
    bool _have_strthr;          // true if we have a valid direct steering and throttle inputs
    uint32_t _strthr_time_ms;   // system time last call to set_steering_and_throttle was made (used for timeout)
    float _strthr_steering;     // direct steering input in the range -1 to +1
    float _strthr_throttle;     // direct throttle input in the range -1 to +1

    // limits
    struct {
        uint32_t timeout_ms;// timeout from the time that guided is invoked
        float horiz_max;    // horizontal position limit in meters from where guided mode was initiated (0 = no limit)
        uint32_t start_time_ms; // system time in milliseconds that control was handed to the external computer
        Location start_loc; // starting location for checking horiz_max limit
    } limit;
};


class ModeHold : public Mode
{
public:

    Number mode_number() const override { return Number::HOLD; }
    const char *name4() const override { return "HOLD"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // attributes for mavlink system status reporting
    bool attitude_stabilized() const override { return false; }

    // hold mode does not require position or velocity estimate
    bool requires_position() const override { return false; }
    bool requires_velocity() const override { return false; }
};

class ModeLoiter : public Mode
{
public:

    Number mode_number() const override { return Number::LOITER; }
    const char *name4() const override { return "LOIT"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // attributes of the mode
    bool is_autopilot_mode() const override { return true; }

    // return desired heading (in degrees) and cross track error (in meters) for reporting to ground station (NAV_CONTROLLER_OUTPUT message)
    float wp_bearing() const override { return _desired_yaw_cd * 0.01f; }
    float nav_bearing() const override { return _desired_yaw_cd * 0.01f; }
    float crosstrack_error() const override { return 0.0f; }

    // return desired location
    bool get_desired_location(Location& destination) const override WARN_IF_UNUSED;

    // return distance (in meters) to destination
    float get_distance_to_destination() const override { return _distance_to_destination; }

protected:

    bool _enter() override;

    Location _destination;      // target location to hold position around
    float _desired_speed;       // desired speed (ramped down from initial speed to zero)
};

class ModeManual : public Mode
{
public:

    Number mode_number() const override { return Number::MANUAL; }
    const char *name4() const override { return "MANU"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // attributes for mavlink system status reporting
    bool has_manual_input() const override { return true; }
    bool attitude_stabilized() const override { return false; }

    // manual mode does not require position or velocity estimate
    bool requires_position() const override { return false; }
    bool requires_velocity() const override { return false; }

protected:

    void _exit() override;
};


class ModeRTL : public Mode
{
public:

    Number mode_number() const override { return Number::RTL; }
    const char *name4() const override { return "RTL"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // attributes of the mode
    bool is_autopilot_mode() const override { return true; }

    // do not allow arming from this mode
    bool allows_arming() const override { return false; }

    // return desired location
    bool get_desired_location(Location& destination) const override WARN_IF_UNUSED;

    // return distance (in meters) to destination
    float get_distance_to_destination() const override { return _distance_to_destination; }
    bool reached_destination() const override;

    // set desired speed in m/s
    bool set_desired_speed(float speed) override;

protected:

    bool _enter() override;

    bool send_notification; // used to send one time notification to ground station
    bool _loitering;        // true if loitering at end of RTL

};

class ModeSmartRTL : public Mode
{
public:

    Number mode_number() const override { return Number::SMART_RTL; }
    const char *name4() const override { return "SRTL"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // attributes of the mode
    bool is_autopilot_mode() const override { return true; }

    // do not allow arming from this mode
    bool allows_arming() const override { return false; }

    // return desired location
    bool get_desired_location(Location& destination) const override WARN_IF_UNUSED;

    // return distance (in meters) to destination
    float get_distance_to_destination() const override { return _distance_to_destination; }
    bool reached_destination() const override { return smart_rtl_state == SmartRTLState::StopAtHome; }

    // set desired speed in m/s
    bool set_desired_speed(float speed) override;

    // save current position for use by the smart_rtl flight mode
    void save_position();

protected:

    // Safe RTL states
    enum class SmartRTLState: uint8_t {
        WaitForPathCleanup,
        PathFollow,
        StopAtHome,
        Failure
    } smart_rtl_state;

    bool _enter() override;
    bool _load_point;
    bool _loitering;        // true if loitering at end of SRTL
};



class ModeSteering : public Mode
{
public:

    Number mode_number() const override { return Number::STEERING; }
    const char *name4() const override { return "STER"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // attributes for mavlink system status reporting
    bool has_manual_input() const override { return true; }

    // steering requires velocity but not position
    bool requires_position() const override { return false; }
    bool requires_velocity() const override { return true; }

    // return desired lateral acceleration
    float get_desired_lat_accel() const override { return _desired_lat_accel; }

private:

    float _desired_lat_accel;   // desired lateral acceleration calculated from pilot steering input
};

class ModeInitializing : public Mode
{
public:

    Number mode_number() const override { return Number::INITIALISING; }
    const char *name4() const override { return "INIT"; }

    // methods that affect movement of the vehicle in this mode
    void update() override { }

    // do not allow arming from this mode
    bool allows_arming() const override { return false; }

    // attributes for mavlink system status reporting
    bool has_manual_input() const override { return true; }
    bool attitude_stabilized() const override { return false; }
protected:
    bool _enter() override { return false; };
};

#if MODE_FOLLOW_ENABLED
class ModeFollow : public Mode
{
public:

    Number mode_number() const override { return Number::FOLLOW; }
    const char *name4() const override { return "FOLL"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // attributes of the mode
    bool is_autopilot_mode() const override { return true; }

    // return desired heading (in degrees) and cross track error (in meters) for reporting to ground station (NAV_CONTROLLER_OUTPUT message)
    float wp_bearing() const override;
    float nav_bearing() const override { return wp_bearing(); }
    float crosstrack_error() const override { return 0.0f; }

    // return desired location
    bool get_desired_location(Location& destination) const override WARN_IF_UNUSED { return false; }

    // return distance (in meters) to destination
    float get_distance_to_destination() const override;

    // set desired speed in m/s
    bool set_desired_speed(float speed) override;

protected:

    bool _enter() override;
    void _exit() override;

    float _desired_speed;       // desired speed in m/s
};
#endif

class ModeSimple : public Mode
{
public:

    Number mode_number() const override { return Number::SIMPLE; }
    const char *name4() const override { return "SMPL"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;
    void init_heading();

    // simple type enum used for SIMPLE_TYPE parameter
    enum simple_type {
        Simple_InitialHeading = 0,
        Simple_CardinalDirections = 1,
    };

private:

    float _initial_heading_cd;  // vehicle heading (in centi-degrees) at moment vehicle was armed
    float _desired_heading_cd;  // latest desired heading (in centi-degrees) from pilot
};

#if MODE_DOCK_ENABLED
class ModeDock : public Mode
{
public:

    // need a constructor for parameters
    ModeDock(void);

    // Does not allow copies
    CLASS_NO_COPY(ModeDock);

    Number mode_number() const override { return Number::DOCK; }
    const char *name4() const override { return "DOCK"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    bool is_autopilot_mode() const override { return true; }

    // return distance (in meters) to destination
    float get_distance_to_destination() const override { return _distance_to_destination; }

    static const struct AP_Param::GroupInfo var_info[];

protected:

    AP_Float speed; // dock mode speed
    AP_Float desired_dir; // desired direction of approach
    AP_Int8 hdg_corr_enable; // enable heading correction
    AP_Float hdg_corr_weight; // heading correction weight
    AP_Float stopping_dist; // how far away from the docking target should we start stopping
    AP_Float dock_slow_dist_max_m; // Maximum distance to start slowdown
    AP_Float dock_slow_dist_min_m; // Minimum distance to start slowdown

    bool _enter() override;

    // return reduced speed of vehicle based on error in position and current distance from the dock
    float apply_slowdown(float desired_speed);

    // calculate position of dock relative to the vehicle
    bool calc_dock_pos_rel_vehicle_NE(Vector2f &dock_pos_rel_vehicle) const;

    // we force the vehicle to use real dock target vector when this much close to the docking station
    const float _force_real_target_limit_cm = 300.0f;
    // acceptable lateral error in vehicle's position with respect to dock. This is used while slowing down the vehicle
    const float _acceptable_pos_error_cm = 20.0f;

    Vector2f _dock_pos_rel_origin_cm;   // position vector towards docking target relative to ekf origin
    Vector2f _desired_heading_NE;       // unit vector in desired direction of docking
    bool _docking_complete = false;     // flag to mark docking complete when we are close enough to the dock
    bool _loitering = false; // true if we are loitering after mission completion
};
#endif
