#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/Location.h>
#include <AP_InertialNav/AP_InertialNav.h>     // Inertial Navigation library
#include <AC_AttitudeControl/AC_PosControl.h>      // Position control library
#include <AC_AttitudeControl/AC_AttitudeControl.h> // Attitude control library
#include <AP_Terrain/AP_Terrain.h>
#include <AC_Avoidance/AC_Avoid.h>                 // Stop at fence library

// loiter maximum velocities and accelerations
#define WPNAV_ACCELERATION              100.0f      // defines the default velocity vs distant curve.  maximum acceleration in cm/s/s that position controller asks for from acceleration controller
#define WPNAV_ACCELERATION_MIN           50.0f      // minimum acceleration in cm/s/s - used for sanity checking _wp_accel parameter

#define WPNAV_LOITER_SPEED             1250.0f      // default loiter speed in cm/s
#define WPNAV_LOITER_SPEED_MIN           20.0f      // minimum loiter speed in cm/s
#define WPNAV_LOITER_ACCEL_MAX          500.0f      // default acceleration in loiter mode
#define WPNAV_LOITER_BRAKE_ACCEL        250.0f      // minimum acceleration in loiter mode
#define WPNAV_LOITER_BRAKE_JERK         500.0f      // maximum jerk in cm/s/s/s in loiter mode
#define WPNAV_LOITER_BRAKE_START_DELAY    1.0f      // delay (in seconds) before loiter braking begins after sticks are released
#define WPNAV_LOITER_VEL_CORRECTION_MAX 200.0f      // max speed used to correct position errors in loiter
#define WPNAV_LOITER_POS_CORRECTION_MAX 200.0f      // max position error in loiter

#define WPNAV_WP_SPEED                  500.0f      // default horizontal speed between waypoints in cm/s
#define WPNAV_WP_SPEED_MIN               20.0f      // minimum horizontal speed between waypoints in cm/s
#define WPNAV_WP_TRACK_SPEED_MIN         50.0f      // minimum speed along track of the target point the vehicle is chasing in cm/s (used as target slows down before reaching destination)
#define WPNAV_WP_RADIUS                 200.0f      // default waypoint radius in cm
#define WPNAV_WP_RADIUS_MIN              10.0f      // minimum waypoint radius in cm

#define WPNAV_WP_SPEED_UP               250.0f      // default maximum climb velocity
#define WPNAV_WP_SPEED_DOWN             150.0f      // default maximum descent velocity

#define WPNAV_WP_ACCEL_Z_DEFAULT        100.0f      // default vertical acceleration between waypoints in cm/s/s

#define WPNAV_LEASH_LENGTH_MIN          100.0f      // minimum leash lengths in cm

#define WPNAV_WP_FAST_OVERSHOOT_MAX     200.0f      // 2m overshoot is allowed during fast waypoints to allow for smooth transitions to next waypoint

#define WPNAV_LOITER_ACTIVE_TIMEOUT_MS     200      // loiter controller is considered active if it has been called within the past 200ms (0.2 seconds)

#define WPNAV_YAW_DIST_MIN                 200      // minimum track length which will lead to target yaw being updated to point at next waypoint.  Under this distance the yaw target will be frozen at the current heading
#define WPNAV_YAW_LEASH_PCT_MIN         0.134f      // target point must be at least this distance from the vehicle (expressed as a percentage of the maximum distance it can be from the vehicle - i.e. the leash length)

#define WPNAV_RANGEFINDER_FILT_Z         0.25f      // range finder distance filtered at 0.25hz

class AC_WPNav
{
public:

    // spline segment end types enum
    enum spline_segment_end_type {
        SEGMENT_END_STOP = 0,
        SEGMENT_END_STRAIGHT,
        SEGMENT_END_SPLINE
    };

    /// Constructor
    AC_WPNav(const AP_InertialNav& inav, const AP_AHRS_View& ahrs, AC_PosControl& pos_control, const AC_AttitudeControl& attitude_control);

    /// provide pointer to terrain database
    void set_terrain(AP_Terrain* terrain_ptr) { _terrain = terrain_ptr; }

    /// provide pointer to avoidance library
    void set_avoidance(AC_Avoid* avoid_ptr) { _avoid = avoid_ptr; }

    /// provide rangefinder altitude
    void set_rangefinder_alt(bool use, bool healthy, float alt_cm) { _rangefinder_available = use; _rangefinder_healthy = healthy; _rangefinder_alt_cm = alt_cm; }

    ///
    /// loiter controller
    ///

    /// init_loiter_target to a position in cm from ekf origin
    ///     caller can set reset_I to false to preserve I term since previous time loiter controller ran.  Should only be false when caller is sure that not too much time has passed to invalidate the I terms
    void init_loiter_target(const Vector3f& position, bool reset_I=true);

    /// init_loiter_target - initialize's loiter position and feed-forward velocity from current pos and velocity
    void init_loiter_target();

    /// loiter_soften_for_landing - reduce response for landing
    void loiter_soften_for_landing();

    /// calculate_loiter_leash_length - calculates the maximum distance in cm that the target position may be from the current location
    void calculate_loiter_leash_length();

    /// set_pilot_desired_acceleration - sets pilot desired acceleration from roll and pitch stick input
    void set_pilot_desired_acceleration(float control_roll, float control_pitch);
    /// get_pilot_desired_acceleration - gets pilot desired
    /// acceleration, body frame, [forward,right]
    Vector2f get_pilot_desired_acceleration() const { return Vector2f(_loiter_desired_accel.x, _loiter_desired_accel.y); }
    /// clear_pilot_desired_acceleration - clear pilot desired acceleration
    void clear_pilot_desired_acceleration() { _loiter_desired_accel.x = 0.0f; _loiter_desired_accel.y = 0.0f; }

    /// get_stopping_point - returns vector to stopping point based on a horizontal position and velocity
    void get_loiter_stopping_point_xy(Vector3f& stopping_point) const;

    /// get_loiter_distance_to_target - get horizontal distance to loiter target in cm
    float get_loiter_distance_to_target() const { return _pos_control.get_distance_to_target(); }

    /// get_loiter_bearing_to_target - get bearing to loiter target in centi-degrees
    int32_t get_loiter_bearing_to_target() const;

    /// get_loiter_target - returns loiter target position
    const Vector3f& get_loiter_target() const { return _pos_control.get_pos_target(); }

    /// get_loiter_angle_max - returns the maximum pilot commanded angle in degrees
    float get_loiter_angle_max_cd() const;

    /// update_loiter - run the loiter controller - should be called at 10hz
    void update_loiter(float ekfGndSpdLimit, float ekfNavVelGainScaler);

    ///
    /// brake controller
    ///
    /// init_brake_target - initialize's position and feed-forward velocity from current pos and velocity
    void init_brake_target(float accel_cmss);
    ///
    /// update_brake - run the brake controller - should be called at 400hz
    void update_brake(float ekfGndSpdLimit, float ekfNavVelGainScaler);

    ///
    /// waypoint controller
    ///

    /// wp_and_spline_init - initialise straight line and spline waypoint controllers
    ///     updates target roll, pitch targets and I terms based on vehicle lean angles
    ///     should be called once before the waypoint controller is used but does not need to be called before subsequent updates to destination
    void wp_and_spline_init();

    /// set_speed_xy - allows main code to pass target horizontal velocity for wp navigation
    void set_speed_xy(float speed_cms);

    /// get_speed_xy - allows main code to retrieve target horizontal velocity for wp navigation
    float get_speed_xy() const { return _wp_speed_cms; }

    /// get_speed_up - returns target climb speed in cm/s during missions
    float get_speed_up() const { return _wp_speed_up_cms; }

    /// get_speed_down - returns target descent speed in cm/s during missions.  Note: always positive
    float get_speed_down() const { return _wp_speed_down_cms; }

    /// get_speed_z - returns target descent speed in cm/s during missions.  Note: always positive
    float get_accel_z() const { return _wp_accel_z_cms; }

    /// get_wp_acceleration - returns acceleration in cm/s/s during missions
    float get_wp_acceleration() const { return _wp_accel_cms.get(); }

    /// get_wp_destination waypoint using position vector (distance from ekf origin in cm)
    const Vector3f &get_wp_destination() const { return _destination; }

    /// get origin using position vector (distance from ekf origin in cm)
    const Vector3f &get_wp_origin() const { return _origin; }

    /// set_wp_destination waypoint using location class
    ///     returns false if conversion from location to vector from ekf origin cannot be calculated
    bool set_wp_destination(const Location_Class& destination);

    /// set_wp_destination waypoint using position vector (distance from ekf origin in cm)
    ///     terrain_alt should be true if destination.z is a desired altitude above terrain
    bool set_wp_destination(const Vector3f& destination, bool terrain_alt = false);

    /// set waypoint destination using NED position vector from ekf origin in meters
    bool set_wp_destination_NED(const Vector3f& destination_NED);

    /// set_wp_origin_and_destination - set origin and destination waypoints using position vectors (distance from ekf origin in cm)
    ///     terrain_alt should be true if origin.z and destination.z are desired altitudes above terrain (false if these are alt-above-ekf-origin)
    ///     returns false on failure (likely caused by missing terrain data)
    bool set_wp_origin_and_destination(const Vector3f& origin, const Vector3f& destination, bool terrain_alt = false);

    /// shift_wp_origin_to_current_pos - shifts the origin and destination so the origin starts at the current position
    ///     used to reset the position just before takeoff
    ///     relies on set_wp_destination or set_wp_origin_and_destination having been called first
    void shift_wp_origin_to_current_pos();

    /// get_wp_stopping_point_xy - calculates stopping point based on current position, velocity, waypoint acceleration
    ///		results placed in stopping_position vector
    void get_wp_stopping_point_xy(Vector3f& stopping_point) const;
    void get_wp_stopping_point(Vector3f& stopping_point) const;

    /// get_wp_distance_to_destination - get horizontal distance to destination in cm
    float get_wp_distance_to_destination() const;

    /// get_bearing_to_destination - get bearing to next waypoint in centi-degrees
    int32_t get_wp_bearing_to_destination() const;

    /// reached_destination - true when we have come within RADIUS cm of the waypoint
    bool reached_wp_destination() const { return _flags.reached_destination; }

    /// set_fast_waypoint - set to true to ignore the waypoint radius and consider the waypoint 'reached' the moment the intermediate point reaches it
    void set_fast_waypoint(bool fast) { _flags.fast_waypoint = fast; }

    /// update_wpnav - run the wp controller - should be called at 100hz or higher
    bool update_wpnav();

    // check_wp_leash_length - check recalc_wp_leash flag and calls calculate_wp_leash_length() if necessary
    //  should be called after _pos_control.update_xy_controller which may have changed the position controller leash lengths
    void check_wp_leash_length();

    /// calculate_wp_leash_length - calculates track speed, acceleration and leash lengths for waypoint controller
    void calculate_wp_leash_length();

    ///
    /// spline methods
    ///

    // segment start types
    // stop - vehicle is not moving at origin
    // straight-fast - vehicle is moving, previous segment is straight.  vehicle will fly straight through the waypoint before beginning it's spline path to the next wp
    //     _flag.segment_type holds whether prev segment is straight vs spline but we don't know if it has a delay
    // spline-fast - vehicle is moving, previous segment is splined, vehicle will fly through waypoint but previous segment should have it flying in the correct direction (i.e. exactly parallel to position difference vector from previous segment's origin to this segment's destination)

    // segment end types
    // stop - vehicle is not moving at destination
    // straight-fast - next segment is straight, vehicle's destination velocity should be directly along track from this segment's destination to next segment's destination
    // spline-fast - next segment is spline, vehicle's destination velocity should be parallel to position difference vector from previous segment's origin to this segment's destination

    // get target yaw in centi-degrees (used for wp and spline navigation)
    float get_yaw() const;

    /// set_spline_destination waypoint using location class
    ///     returns false if conversion from location to vector from ekf origin cannot be calculated
    ///     terrain_alt should be true if destination.z is a desired altitude above terrain (false if its desired altitude above ekf origin)
    ///     stopped_at_start should be set to true if vehicle is stopped at the origin
    ///     seg_end_type should be set to stopped, straight or spline depending upon the next segment's type
    ///     next_destination should be set to the next segment's destination if the seg_end_type is SEGMENT_END_STRAIGHT or SEGMENT_END_SPLINE
    bool set_spline_destination(const Location_Class& destination, bool stopped_at_start, spline_segment_end_type seg_end_type, Location_Class next_destination);

    /// set_spline_destination waypoint using position vector (distance from ekf origin in cm)
    ///     returns false if conversion from location to vector from ekf origin cannot be calculated
    ///     terrain_alt should be true if destination.z is a desired altitudes above terrain (false if its desired altitudes above ekf origin)
    ///     stopped_at_start should be set to true if vehicle is stopped at the origin
    ///     seg_end_type should be set to stopped, straight or spline depending upon the next segment's type
    ///     next_destination should be set to the next segment's destination if the seg_end_type is SEGMENT_END_STRAIGHT or SEGMENT_END_SPLINE
    ///     next_destination.z  must be in the same "frame" as destination.z (i.e. if destination is a alt-above-terrain, next_destination should be too)
    bool set_spline_destination(const Vector3f& destination, bool terrain_alt, bool stopped_at_start, spline_segment_end_type seg_end_type, const Vector3f& next_destination);

    /// set_spline_origin_and_destination - set origin and destination waypoints using position vectors (distance from ekf origin in cm)
    ///     terrain_alt should be true if origin.z and destination.z are desired altitudes above terrain (false if desired altitudes above ekf origin)
    ///     stopped_at_start should be set to true if vehicle is stopped at the origin
    ///     seg_end_type should be set to stopped, straight or spline depending upon the next segment's type
    ///     next_destination should be set to the next segment's destination if the seg_end_type is SEGMENT_END_STRAIGHT or SEGMENT_END_SPLINE
    bool set_spline_origin_and_destination(const Vector3f& origin, const Vector3f& destination, bool terrain_alt, bool stopped_at_start, spline_segment_end_type seg_end_type, const Vector3f& next_destination);

    /// reached_spline_destination - true when we have come within RADIUS cm of the waypoint
    bool reached_spline_destination() const { return _flags.reached_destination; }

    /// update_spline - update spline controller
    bool update_spline();

    ///
    /// shared methods
    ///

    /// get desired roll, pitch which should be fed into stabilize controllers
    int32_t get_roll() const { return _pos_control.get_roll(); }
    int32_t get_pitch() const { return _pos_control.get_pitch(); }

    /// get_desired_alt - get desired altitude (in cm above ekf origin) from loiter or wp controller which should be fed into throttle controller
    float get_desired_alt() const { return _pos_control.get_alt_target(); }

    /// set_desired_alt - set desired altitude (in cm above ekf origin)
    void set_desired_alt(float desired_alt) { _pos_control.set_alt_target(desired_alt); }

    /// advance_wp_target_along_track - move target location along track from origin to destination
    bool advance_wp_target_along_track(float dt);

    static const struct AP_Param::GroupInfo var_info[];

protected:

    // segment types, either straight or spine
    enum SegmentType {
        SEGMENT_STRAIGHT = 0,
        SEGMENT_SPLINE = 1
    };

    // flags structure
    struct wpnav_flags {
        uint8_t reached_destination     : 1;    // true if we have reached the destination
        uint8_t fast_waypoint           : 1;    // true if we should ignore the waypoint radius and consider the waypoint complete once the intermediate target has reached the waypoint
        uint8_t slowing_down            : 1;    // true when target point is slowing down before reaching the destination
        uint8_t recalc_wp_leash         : 1;    // true if we need to recalculate the leash lengths because of changes in speed or acceleration
        uint8_t new_wp_destination      : 1;    // true if we have just received a new destination.  allows us to freeze the position controller's xy feed forward
        SegmentType segment_type        : 1;    // active segment is either straight or spline
        uint8_t wp_yaw_set              : 1;    // true if yaw target has been set
    } _flags;

    /// calc_loiter_desired_velocity - updates desired velocity (i.e. feed forward) with pilot requested acceleration and fake wind resistance
    ///		updated velocity sent directly to position controller
    void calc_loiter_desired_velocity(float nav_dt, float ekfGndSpdLimit);

    /// calc_slow_down_distance - calculates distance before waypoint that target point should begin to slow-down assuming it is traveling at full speed
    void calc_slow_down_distance(float speed_cms, float accel_cmss);

    /// get_slow_down_speed - returns target speed of target point based on distance from the destination (in cm)
    float get_slow_down_speed(float dist_from_dest_cm, float accel_cmss);

    /// spline protected functions

    /// update_spline_solution - recalculates hermite_spline_solution grid
    void update_spline_solution(const Vector3f& origin, const Vector3f& dest, const Vector3f& origin_vel, const Vector3f& dest_vel);

    /// advance_spline_target_along_track - move target location along track from origin to destination
    ///     returns false if it is unable to advance (most likely because of missing terrain data)
    bool advance_spline_target_along_track(float dt);

    /// calc_spline_pos_vel - update position and velocity from given spline time
    /// 	relies on update_spline_solution being called since the previous
    void calc_spline_pos_vel(float spline_time, Vector3f& position, Vector3f& velocity);

    // get terrain's altitude (in cm above the ekf origin) at the current position (+ve means terrain below vehicle is above ekf origin's altitude)
    bool get_terrain_offset(float& offset_cm);

    // convert location to vector from ekf origin.  terrain_alt is set to true if resulting vector's z-axis should be treated as alt-above-terrain
    //      returns false if conversion failed (likely because terrain data was not available)
    bool get_vector_NEU(const Location_Class &loc, Vector3f &vec, bool &terrain_alt);

    // set heading used for spline and waypoint navigation
    void set_yaw_cd(float heading_cd);

    // references and pointers to external libraries
    const AP_InertialNav&   _inav;
    const AP_AHRS_View&     _ahrs;
    AC_PosControl&          _pos_control;
    const AC_AttitudeControl& _attitude_control;
    AP_Terrain              *_terrain = nullptr;
    AC_Avoid                *_avoid = nullptr;

    // parameters
    AP_Float    _loiter_angle_max;      // maximum pilot commanded angle in degrees. Set to zero for 2/3 Angle Max
    AP_Float    _loiter_speed_cms;      // maximum horizontal speed in cm/s while in loiter
    AP_Float    _loiter_accel_cmss;     // loiter's max acceleration in cm/s/s
    AP_Float    _loiter_brake_accel_cmss; // loiter's acceleration during braking in cm/s/s
    AP_Float    _loiter_brake_jerk_max_cmsss;
    AP_Float    _loiter_brake_delay;    // delay (in seconds) before loiter braking begins after sticks are released
    AP_Float    _wp_speed_cms;          // maximum horizontal speed in cm/s during missions
    AP_Float    _wp_speed_up_cms;       // climb speed target in cm/s
    AP_Float    _wp_speed_down_cms;     // descent speed target in cm/s
    AP_Float    _wp_radius_cm;          // distance from a waypoint in cm that, when crossed, indicates the wp has been reached
    AP_Float    _wp_accel_cms;          // horizontal acceleration in cm/s/s during missions
    AP_Float    _wp_accel_z_cms;        // vertical acceleration in cm/s/s during missions

    // loiter controller internal variables
    Vector2f    _loiter_desired_accel;  // slewed pilot's desired acceleration in lat/lon frame
    Vector2f    _loiter_predicted_accel;//
    Vector2f    _loiter_predicted_euler_angle;//
    Vector2f    _loiter_predicted_euler_rate; //
    float       _brake_timer;           //
    float       _loiter_brake_accel;    //

    // waypoint controller internal variables
    uint32_t    _wp_last_update;        // time of last update_wpnav call
    uint8_t     _wp_step;               // used to decide which portion of wpnav controller to run during this iteration
    Vector3f    _origin;                // starting point of trip to next waypoint in cm from ekf origin
    Vector3f    _destination;           // target destination in cm from ekf origin
    Vector3f    _pos_delta_unit;        // each axis's percentage of the total track from origin to destination
    float       _track_length;          // distance in cm between origin and destination
    float       _track_length_xy;       // horizontal distance in cm between origin and destination
    float       _track_desired;         // our desired distance along the track in cm
    float       _limited_speed_xy_cms;  // horizontal speed in cm/s used to advance the intermediate target towards the destination.  used to limit extreme acceleration after passing a waypoint
    float       _track_accel;           // acceleration along track
    float       _track_speed;           // speed in cm/s along track
    float       _track_leash_length;    // leash length along track
    float       _slow_down_dist;        // vehicle should begin to slow down once it is within this distance from the destination

    // spline variables
    float       _spline_time;           // current spline time between origin and destination
    float       _spline_time_scale;     // current spline time between origin and destination
    Vector3f    _spline_origin_vel;     // the target velocity vector at the origin of the spline segment
    Vector3f    _spline_destination_vel;// the target velocity vector at the destination point of the spline segment
    Vector3f    _hermite_spline_solution[4]; // array describing spline path between origin and destination
    float       _spline_vel_scaler;	    //
    float       _yaw;                   // heading according to yaw

    // terrain following variables
    bool        _terrain_alt = false;   // true if origin and destination.z are alt-above-terrain, false if alt-above-ekf-origin
    bool        _ekf_origin_terrain_alt_set = false;
    bool        _rangefinder_available;
    AP_Int8     _rangefinder_use;
    bool        _rangefinder_healthy = false;
    float       _rangefinder_alt_cm = 0.0f;
};
