#pragma once

#include <AP_Buffer/AP_Buffer.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <DataFlash/DataFlash.h>
#include <GCS_MAVLink/GCS.h>

#include <bitset>
#include <vector>

#define SAFERTL_ACCURACY_DEFAULT 2.0f
#define SAFERTL_PATH_LEN_DEFAULT 110 // the amount of memory used by safe RTL will be slightly higher than 3*8*MAX_PATH_LEN bytes. Increasing this number will improve path pruning, but will use more memory, and running a path cleanup will take longer. No longer than 255.

#define SAFERTL_MAX_ALLOWABLE_PATH_LEN 500 // even if the user sets the path len parameter higher than this, it will default to this.
#define SAFERTL_POSITION_DELTA 2.0f // how many meters to move before appending a new position to return_path
#define SAFERTL_PRUNING_DELTA (accuracy * 0.99) // XXX must be smaller than position_delta! How many meters apart must two points be, such that we can assume that there is no obstacle between those points
#define SAFERTL_SIMPLIFICATION_EPSILON (accuracy * 0.5)

#define SAFERTL_SIMPLIFICATION_STACK_LEN_MULT (2/3)+1 // the amount of memory to be allocated for the SIMPLIFICATION algorithm to write its to do list.
// XXX A number too small for SAFERTL_SIMPLIFICATION_STACK_LEN can cause a buffer overflow! The number to put here is int((s/2-1)+min(s/2, MAX_PATH_LEN-s)), where s = pow(2, floor(log(MAX_PATH_LEN)/log(2)))
// To avoid this annoying math, a good-enough overestimate is ciel(MAX_PATH_LEN*2./3.)
#define SAFERTL_SIMPLIFICATION_TIME 200
#define SAFERTL_LOOP_BUFFER_LEN_MULT 1/4
#define SAFERTL_LOOP_TIME 300
#define SAFERTL_BAD_POSITION_TIME 15000 // the time in ms with no valid position, before SafeRTL is disabled.

#define HYPOT(a,b) (a-b).length()

class SafeRTL_Path {

public:

    // constructor
    SafeRTL_Path(bool log);

    // call this a couple of times per second with a position, regardless of what mode the vehicle is in
    void update(bool position_ok, Vector3f current_pos);

    // turn on/off accepting new points in calls to append_if_far_enough
    void accepting_new_points(bool value) { _accepting_new_points = value; }

    // perform thoroguh clean-up.  This should be run just before initiating the RTL. Returns a pointer to the cleaned-up path or nullptr if clean-up is not complete
    Vector3f* thorough_cleanup();

    // get a point on the path
    const Vector3f& get_point(int32_t index) const { return path[index]; }

    // get next point on the path to home
    bool pop_point(Vector3f& point);

    // clear return path and set home locatione
    void reset_path(bool position_ok, const Vector3f start);

    bool cleanup_ready() const { return _pruning_complete && _simplification_complete; }
    bool is_active() const { return _active; }
    // the two cleanup steps. These should be run regularly, preferably not int the main thread
    void detect_simplifications();
    void detect_loops();

    // *** User Parameters ***
    static const struct AP_Param::GroupInfo var_info[];
    AP_Float accuracy;
    AP_Int16 desired_path_len;

private:
    // perform clean-up regularly from main loop
    bool _routine_cleanup();
    // misc cleanup helper methods:
    void _reset_simplification(bool hard);
    void _reset_pruning(bool hard);
    void _zero_points_by_simplification_bitmask();
    void _remove_unacceptable_overlapping_loops();
    void _zero_points_by_loops(int32_t points_to_delete);
    void _remove_empty_points();
    // _segment_segment_dist returns two things, the closest distance reached between 2 line segments, and the point exactly between them.
    typedef struct {
        float distance;
        Vector3f point;
    } dist_point;
    // typedef struct dist_point dist_point;
    static dist_point _segment_segment_dist(const Vector3f& p1, const Vector3f& p2, const Vector3f& p3, const Vector3f& p4);
    static float _point_line_dist(const Vector3f& point, const Vector3f& line1, const Vector3f& line2);

    // *** SafeRTL State Variables ***

    // points are stored in meters from EKF origin in NED
    Vector3f* path;
    int32_t _current_path_len; // after the array has been allocated, we will need to know how big it is. We can't use the parameter, because a user could change the parameter in-flight
    int32_t _last_index;
    uint32_t _time_of_last_good_position; // the time when a last good position was reported. If no position is available for a while, SafeRTL will be disabled.
    bool _accepting_new_points; // false means that any call to append_if_far_enough() will fail. This should be unset when entering SafeRTL mode, and set when exiting.
    bool _active; // if the path becomes too long to keep in memory, and too convoluted to be cleaned up, SafeRTL will be permanently deactivated (for the remainder of the flight)
    bool _logging_enabled;

    // *** Simplification state ***

    bool _simplification_complete;
    // structure and buffer to hold the "to-do list" for the SIMPLIFICATION algorithm.
    typedef struct {
        int32_t start;
        int32_t finish;
    } start_finish;
    start_finish* _simplification_stack;
    int32_t _simplification_stack_last_index;
    // the result of the simplification algorithm
    std::bitset<SAFERTL_MAX_ALLOWABLE_PATH_LEN> _simplification_bitmask;
    // everything before _simplification_clean_until has been calculated already to be un-simplify-able. This avoids recalculating a known result.
    int32_t _simplification_clean_until;

    // *** Pruning state ***

    bool _pruning_complete;
    int32_t _pruning_current_i;
    int32_t _pruning_min_j;
    typedef struct {
        int32_t start_index;
        int32_t end_index;
        Vector3f halfway_point;
    } loop;
    // the result of the pruning algorithm
    loop* _prunable_loops;
    int32_t _prunable_loops_last_index;
    // everything before _pruning_clean_until has been calculated already to be un-simplify-able. This avoids recalculating a known result.
    int32_t _pruning_clean_until;
};
