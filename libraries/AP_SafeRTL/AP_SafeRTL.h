#pragma once

#include <AP_Buffer/AP_Buffer.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/Bitmask.h>
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include <DataFlash/DataFlash.h>
#include <GCS_MAVLink/GCS.h>

// definitions and macros
#define SAFERTL_ACCURACY_DEFAULT        2.0f    // default _ACCURACY parameter value.  Points will be no closer than this distance together.
#define SAFERTL_POINTS_MAX_DEFAULT      150     // default _POINTS_MAX parameter value.  High numbers improve path pruning but use more memory and CPU for cleanup. Memory used will be 20bytes * this number.
#define SAFERTL_POINTS_MAX              500     // the absolute maximum number of points this library can support.
#define SAFERTL_BAD_POSITION_TIMEOUT    15000   // the time in milliseconds with no valid position, before SafeRTL is disabled for the flight
#define SAFERTL_PRUNING_DELTA (_accuracy * 0.99) // How many meters apart must two points be, such that we can assume that there is no obstacle between them.  must be smaller than _ACCURACY parameter
#define SAFERTL_SIMPLIFICATION_EPSILON (_accuracy * 0.5)
#define SAFERTL_SIMPLIFICATION_STACK_LEN_MULT (2/3)+1 // the amount of memory to be allocated for the SIMPLIFICATION algorithm to write its to do list.
// If SAFERTL_SIMPLIFICATION_STACK_LEN_MULT is too low it can cause a buffer overflow! The number to put here is int((s/2-1)+min(s/2, SAFERTL_POINTS_MAX-s)), where s = pow(2, floor(log(SAFERTL_POINTS_MAX)/log(2)))
// To avoid this annoying math, a good-enough overestimate is ceil(SAFERTL_POINTS_MAX*2./3.)
#define SAFERTL_SIMPLIFICATION_TIME_US  200 // maximum time (in microseconds) the simplification algorithm will run before returning
#define SAFERTL_LOOP_BUFFER_LEN_MULT    1/4
#define SAFERTL_LOOP_TIME_US            300 // maximum time (in microseconds) that the loop finding algorithm will run before returning
#define HYPOT(a,b)                      (a-b).length()  // macro to calculate length between two points

class AP_SafeRTL {

public:

    // constructor
    AP_SafeRTL(const AP_AHRS& ahrs);

    // initialise safe rtl including setting up background processes
    void init();

    // clear return path and set home location.  This should be called as part of the arming procedure
    // example sketches use method that allows providing vehicle position directly
    void reset_path(bool position_ok);
    void reset_path(bool position_ok, const Vector3f& current_pos);

    // call this a couple of times per second regardless of what mode the vehicle is in
    // example sketches use method that allows providing vehicle position directly
    void update(bool position_ok, bool save_position);
    void update(bool position_ok, const Vector3f& current_pos);

    // return true if safe_rtl is usable (it may become unusable if the user took off without GPS lock or the path became too long)
    bool is_active() const { return (_initialised && _active); }

    // perform thorough clean-up of the return path.  This should be run just before initiating the RTL.
    // Returns true if the cleanup is complete.  This should be called intermittently and the return journey should not begin until this has returned tru
    bool thorough_cleanup();

    // get a point on the path
    const Vector3f& get_point(int16_t index) const { return _path[index]; }

    // get next point on the path to home, returns true if point is the last point (i.e. home)
    bool pop_point(Vector3f& point);

    // the two cleanup steps. These are run regularly from the IO thread
    // these are public so that they can be tested by the example sketch
    void detect_simplifications();
    void detect_loops();

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

private:

    // enums for logging latest actions
    enum SRTL_Actions {
        SRTL_POINT_ADD,
        SRTL_POINT_PRUNE,
        SRTL_POINT_SIMPLIFY,
        SRTL_DEACTIVATED_INIT_FAILED,
        SRTL_DEACTIVATED_BAD_POSITION,
        SRTL_DEACTIVATED_CLEANUP_FAILED
    };

    // external references
    const AP_AHRS& _ahrs;

    // parameters
    AP_Float _accuracy;
    AP_Int16 _points_max;

    // perform clean-up regularly from main loop
    bool routine_cleanup();
    // misc cleanup helper methods:
    void reset_simplification(bool hard);
    void reset_pruning(bool hard);
    void zero_points_by_simplification_bitmask();
    void zero_points_by_loops(int16_t points_to_delete);
    void remove_empty_points();

    // dist_point holds the closest distance reached between 2 line segments, and the point exactly between them
    typedef struct {
        float distance;
        Vector3f point;
    } dist_point;

    // get the closest distance between 2 line segments and the point midway between the closest points
    static dist_point segment_segment_dist(const Vector3f& p1, const Vector3f& p2, const Vector3f& p3, const Vector3f& p4);
    static float point_line_dist(const Vector3f& point, const Vector3f& line1, const Vector3f& line2);

    // logging
    void log_action(SRTL_Actions action, const Vector3f point = Vector3f());

    // SafeRTL State Variables
    bool _initialised;  // true once initialised (arrays have been allocated successfully)
    bool _active;       // true if safeRTL is usable.  may become unusable if the path becomes too long to keep in memory, and too convoluted to be cleaned up, SafeRTL will be permanently deactivated (for the remainder of the flight)
    Vector3f* _path;    // points are stored in meters from EKF origin in NED
    int16_t _current_path_len;  // after the array has been allocated, we will need to know how big it is. We can't use the parameter, because a user could change the parameter in-flight
    int16_t _last_index;        // index of most recent point added to path
    uint32_t _time_of_last_good_position; // the time when a last good position was reported. If no position is available for a while, SafeRTL will be disabled.

    // Simplification state
    bool _simplification_complete;
    // structure and buffer to hold the "to-do list" for the SIMPLIFICATION algorithm.
    typedef struct {
        int16_t start;
        int16_t finish;
    } start_finish;
    start_finish* _simplification_stack;
    int16_t _simplification_stack_last_index = -1;
    Bitmask _simplification_bitmask = Bitmask(SAFERTL_POINTS_MAX);  // simplify algorithm clears bits for each point that can be removed
    // everything before _simplification_clean_until has been calculated already to be un-simplify-able. This avoids recalculating a known result.
    int16_t _simplification_clean_until;

    // Pruning state
    bool _pruning_complete;
    int16_t _pruning_current_i;
    int16_t _pruning_min_j;
    typedef struct {
        int16_t start_index;
        int16_t end_index;
        Vector3f halfway_point;
    } loop;
    // the result of the pruning algorithm
    loop* _prunable_loops;
    int16_t _prunable_loops_last_index = -1;
    // everything before _pruning_clean_until has been calculated already to be un-simplify-able. This avoids recalculating a known result.
    int16_t _pruning_clean_until;
};
