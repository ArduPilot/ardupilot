#pragma once

#include <AP_Common/AP_Common.h>

/*
 * SCurves calculate paths between waypoints (including the corners) using specified speed, acceleration and jerk limits
 *
 * How to use:
 *  1. create three SCurve objects called something like "prev_leg", "this_leg" and "next_leg"
 *  2. call this_leg.calculate_track() to calculate the path from the origin to the destination for the given speed, accel and jerk limits
 *  3. if the vehicle will fly past the destination to another "next destination":
 *        a) call next_leg.calculate_track() with the appropriate arguments
 *        b) set a "fast_waypoint" boolean to true.  this will be passed into "advance_target_along_track()" in the next step
 *     if there is no "next destination"
 *        a) call next_leg.init()
 *        b) set the "fast_waypoint" boolean to false
 *  4. call this_leg.advance_target_along_track() with a small "dt" value and retrieve the resulting target position, velocity and acceleration
 *     Note: the target_pos should be set to the segments's earth frame origin before this function is called
 *  5. pass the target position, velocity and acceleration into the position controller
 *  6. repeat steps 4 and 5 until finished() returns true
 *  7. promote the legs:
 *        a) set prev_leg = this_leg
 *        b) set this_leg = next_leg
 *        c) jump back to step 3
 *
 * Other features:
 *  1. set_speed_max() allows changing the max speeds mid path.  The path will be recalculated
 *  2. set_origin_speed_max() and set_destination_speed_max() allows setting the speed along the path at the beginning and end of the leg
 *     this is used to smoothly integrate with spline segments
 *
 * This library works with any units (meters, cm, etc) as long as they are used consistently.
 *    e.g. if origin and destination are meters, speeds should be in m/s, accel in m/s/s, etc.
 *
 * Terminology:
 *    position: a point in space
 *    velocity: rate of change of position.  aka speed
 *    acceleration: rate of change of speed
 *    jerk: rate of change of acceleration
 *    snap: rate of change of jerk
 *    jerk time: the time (in seconds) for jerk to increase from zero to its maximum value
 *    track: 3D path that the vehicle will follow
 *    path: position, velocity, accel and jerk kinematic profile that this library generates
 */

class SCurve {

public:

    // constructor
    SCurve();

    // initialise and clear the path
    void init();

    // calculate the segment times for the trigonometric S-Curve path defined by:
    // Sm - maximum value of the snap profile
    // Jm - maximum value of the raised cosine jerk profile
    // V0 - initial velocity magnitude
    // Am - maximum constant acceleration
    // Vm - maximum constant velocity
    // L - Length of the path
    // tj_out, t2_out, t4_out, t6_out are the segment durations needed to achieve the kinematic path specified by the input variables
    // this is an internal function, static for test suite
    static void calculate_path(float Sm, float Jm, float V0, float Am, float Vm, float L, float &Jm_out, float &tj_out, float &t2_out, float &t4_out, float &t6_out);

    // generate a trigonometric track in 3D space that moves over a straight line
    // between two points defined by the origin and destination
    void calculate_track(const Vector3f &origin, const Vector3f &destination,
                         float speed_xy, float speed_up, float speed_down,
                         float accel_xy, float accel_z,
                         float snap_maximum, float jerk_maximum);

    // set maximum velocity and re-calculate the path using these limits
    void set_speed_max(float speed_xy, float speed_up, float speed_down);

    // set the maximum vehicle speed at the origin
    // returns the expected speed at the origin which will always be equal or lower than speed
    float set_origin_speed_max(float speed);

    // set the maximum vehicle speed at the destination
    void set_destination_speed_max(float speed);

    // move target location along path from origin to destination
    // prev_leg and next_leg - the paths before and after this path
    // wp_radius - max distance from the waypoint at the apex of the turn
    // accel_corner - max acceleration that the aircraft may use during the corner
    // fast_waypoint - true if vehicle will not stop at end of this leg
    // dt - the time increment the vehicle will move along the path
    // target_pos - set to this segment's origin and it will be updated to the current position target
    // target_vel and target_accel - updated with new targets
    // advance_target_along_track returns true if vehicle has passed the apex of the corner
    bool advance_target_along_track(SCurve &prev_leg, SCurve &next_leg, float wp_radius, float accel_corner, bool fast_waypoint, float dt, Vector3f &target_pos, Vector3f &target_vel, Vector3f &target_accel) WARN_IF_UNUSED;

    // time has reached the end of the sequence
    bool finished() const WARN_IF_UNUSED;

private:

    // increment time and return the position, velocity and acceleration vectors relative to the origin
    void move_from_pos_vel_accel(float dt, Vector3f &pos, Vector3f &vel, Vector3f &accel);

    // increment time and return the position, velocity and acceleration vectors relative to the destination
    void move_to_pos_vel_accel(float dt, Vector3f &pos, Vector3f &vel, Vector3f &accel);

    // return the position, velocity and acceleration vectors relative to the origin at a specified time along the path
    void move_from_time_pos_vel_accel(float t, Vector3f &pos, Vector3f &vel, Vector3f &accel);

    // get desired maximum speed along track
    float get_speed_along_track() const WARN_IF_UNUSED { return vel_max; }

    // get desired maximum acceleration along track
    float get_accel_along_track() const WARN_IF_UNUSED { return accel_max; }

    // return the change in position from origin to destination
    const Vector3f& get_track() const WARN_IF_UNUSED { return track; };

    // return the current time elapsed
    float get_time_elapsed() const WARN_IF_UNUSED { return time; }

    // time at the end of the sequence
    float time_end() const WARN_IF_UNUSED;

    // time left before sequence will complete
    float get_time_remaining() const WARN_IF_UNUSED;

    // time when acceleration section of the sequence will complete
    float get_accel_finished_time() const WARN_IF_UNUSED;

    // return true if the sequence is braking to a stop
    bool braking() const WARN_IF_UNUSED;

    // return time offset used to initiate the turn onto leg
    float time_turn_in() const WARN_IF_UNUSED;

    // return time offset used to initiate the turn from leg
    float time_turn_out() const WARN_IF_UNUSED;

    // increment the internal time
    void advance_time(float dt);

    // calculate the jerk, acceleration, velocity and position at time t
    void get_jerk_accel_vel_pos_at_time(float time_now, float &Jt_out, float &At_out, float &Vt_out, float &Pt_out) const;

    // calculate the jerk, acceleration, velocity and position at time t when running the constant jerk time segment
    void calc_javp_for_segment_const_jerk(float time_now, float J0, float A0, float V0, float P0, float &Jt, float &At, float &Vt, float &Pt) const;

    // Calculate the jerk, acceleration, velocity and position at time t when running the increasing jerk magnitude time segment based on a raised cosine profile
    void calc_javp_for_segment_incr_jerk(float time_now, float tj, float Jm, float A0, float V0, float P0, float &Jt, float &At, float &Vt, float &Pt) const;

    // Calculate the jerk, acceleration, velocity and position at time t when running the decreasing jerk magnitude time segment based on a raised cosine profile
    void calc_javp_for_segment_decr_jerk(float time_now, float tj, float Jm, float A0, float V0, float P0, float &Jt, float &At, float &Vt, float &Pt) const;

    // generate time segments for straight segment
    void add_segments(float L);

    // generate three time segments forming the jerk profile
    void add_segments_jerk(uint8_t &seg_pnt, float Jm, float tj, float Tcj);

    // generate constant jerk time segment
    void add_segment_const_jerk(uint8_t &seg_pnt, float J0, float tin);

    // generate increasing jerk magnitude time segment based on a raised cosine profile
    void add_segment_incr_jerk(uint8_t &seg_pnt, float Jm, float tj);

    // generate decreasing jerk magnitude time segment based on a raised cosine profile
    void add_segment_decr_jerk(uint8_t &seg_pnt, float Jm, float tj);

    // set speed and acceleration limits for the path
    // origin and destination are offsets from EKF origin
    // speed and acceleration parameters are given in horizontal, up and down.
    void set_kinematic_limits(const Vector3f &origin, const Vector3f &destination,
                              float speed_xy, float speed_up, float speed_down,
                              float accel_xy, float accel_z);

    // return true if the curve is valid.  Used to identify and protect against code errors
    bool valid() const WARN_IF_UNUSED;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // debugging messages
    void debug() const;
#endif

    // segment types
    enum class SegmentType {
        CONSTANT_JERK,
        POSITIVE_JERK,
        NEGATIVE_JERK
    };

    // add single segment
    void add_segment(uint8_t &seg_pnt, float end_time, SegmentType seg_type, float jerk_ref, float end_accel, float end_vel, float end_pos);

    // members
    float snap_max;     // maximum snap magnitude
    float jerk_max;     // maximum jerk magnitude
    float accel_max;    // maximum acceleration magnitude
    float vel_max;      // maximum velocity magnitude
    float time;         // time that defines position on the path
    float position_sq;  // position (squared) on the path at the last time step (used to detect finish)

    // segment 0 is the initial segment and holds the vehicle's initial position and velocity
    // segments 1 to 7 are the acceleration segments
    // segments 8 to 14 are the speed change segments
    // segment 15 is the constant velocity segment
    // segment 16 to 22 is the deceleration segment
    const static uint8_t segments_max = 23; // maximum number of time segments

    uint8_t num_segs;       // number of time segments being used
    struct {
        float jerk_ref;     // jerk reference value for time segment (the jerk at the beginning, middle or end depending upon the segment type)
        SegmentType seg_type;   // segment type (jerk is constant, increasing or decreasing)
        float end_time;     // final time value for segment
        float end_accel;    // final acceleration value for segment
        float end_vel;      // final velocity value for segment
        float end_pos;      // final position value for segment
    } segment[segments_max];

    Vector3f track;       // total change in position from origin to destination
    Vector3f delta_unit;  // reference direction vector for path
};
