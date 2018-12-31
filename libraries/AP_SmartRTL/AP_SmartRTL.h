#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/Bitmask.h>
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include <DataFlash/DataFlash.h>
#include <GCS_MAVLink/GCS.h>

// definitions and macros
#define SMARTRTL_ACCURACY_DEFAULT        2.0f   // default _ACCURACY parameter value.  Points will be no closer than this distance (in meters) together.
#define SMARTRTL_POINTS_DEFAULT          300    // default _POINTS parameter value.  High numbers improve path pruning but use more memory and CPU for cleanup. Memory used will be 20bytes * this number.
#define SMARTRTL_POINTS_MAX              500    // the absolute maximum number of points this library can support.
#define SMARTRTL_TIMEOUT                 15000  // the time in milliseconds with no points saved to the path (for whatever reason), before SmartRTL is disabled for the flight
#define SMARTRTL_CLEANUP_POINT_TRIGGER   50     // simplification will trigger when this many points are added to the path
#define SMARTRTL_CLEANUP_START_MARGIN    10     // routine cleanup algorithms begin when the path array has only this many empty slots remaining
#define SMARTRTL_CLEANUP_POINT_MIN       10     // cleanup algorithms will remove points if they remove at least this many points
#define SMARTRTL_SIMPLIFY_EPSILON (_accuracy * 0.5f)
#define SMARTRTL_SIMPLIFY_STACK_LEN_MULT (2.0f/3.0f)+1  // simplify buffer size as compared to maximum number of points.
                                                // The minimum is int((s/2-1)+min(s/2, SMARTRTL_POINTS_MAX-s)), where s = pow(2, floor(log(SMARTRTL_POINTS_MAX)/log(2)))
                                                // To avoid this annoying math, a good-enough overestimate is ceil(SMARTRTL_POINTS_MAX*2.0f/3.0f)
#define SMARTRTL_SIMPLIFY_TIME_US        200    // maximum time (in microseconds) the simplification algorithm will run before returning
#define SMARTRTL_PRUNING_DELTA (_accuracy * 0.99)   // How many meters apart must two points be, such that we can assume that there is no obstacle between them.  must be smaller than _ACCURACY parameter
#define SMARTRTL_PRUNING_LOOP_BUFFER_LEN_MULT 0.25f // pruning loop buffer size as compared to maximum number of points
#define SMARTRTL_PRUNING_LOOP_TIME_US    200    // maximum time (in microseconds) that the loop finding algorithm will run before returning

class AP_SmartRTL {

public:

    // constructor, destructor
    AP_SmartRTL(bool example_mode = false);

    // initialise safe rtl including setting up background processes
    void init();

    // return true if smart_rtl is usable (it may become unusable if the user took off without GPS lock or the path became too long)
    bool is_active() const { return _active; }

    // returns number of points on the path
    uint16_t get_num_points() const;

    // get a point on the path
    const Vector3f& get_point(uint16_t index) const { return _path[index]; }

    // get next point on the path to home, returns true on success
    bool pop_point(Vector3f& point);

    // clear return path and set return location if position_ok is true.  This should be called as part of the arming procedure
    // if position_ok is false, SmartRTL will not be available.
    // example sketches use the method that allows providing vehicle position directly
    void set_home(bool position_ok);
    void set_home(bool position_ok, const Vector3f& current_pos);

    // call this at 3hz (or higher) regardless of what mode the vehicle is in
    // example sketches use method that allows providing vehicle position directly
    void update(bool position_ok, bool save_position);
    void update(bool position_ok, const Vector3f& current_pos);

    // enum for argument passed to request_through_cleanup
    enum ThoroughCleanupType {
        THOROUGH_CLEAN_DEFAULT = 0,     // perform simplify and prune (used by vehicle code)
        THOROUGH_CLEAN_ALL,             // same as above but used by example sketch
        THOROUGH_CLEAN_SIMPLIFY_ONLY,   // perform simplify only (used by example sketch)
        THOROUGH_CLEAN_PRUNE_ONLY,      // perform prune only (used by example sketch)
    };

    // triggers thorough cleanup including simplification, pruning and removal of all unnecessary points
    // returns true if the thorough cleanup was completed, false if it has not yet completed
    // this method should be called repeatedly until it returns true before initiating the return journey
    // clean_type should only be set by the example sketch
    bool request_thorough_cleanup(ThoroughCleanupType clean_type = THOROUGH_CLEAN_DEFAULT);

    // cancel request for thorough cleanup
    void cancel_request_for_thorough_cleanup();

    // run background cleanup - should be run regularly from the IO thread
    void run_background_cleanup();

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

private:

    // enums for logging latest actions
    enum SRTL_Actions {
        SRTL_POINT_ADD,
        SRTL_POINT_PRUNE,
        SRTL_POINT_SIMPLIFY,
        SRTL_ADD_FAILED_NO_SEMAPHORE,
        SRTL_ADD_FAILED_PATH_FULL,
        SRTL_POP_FAILED_NO_SEMAPHORE,
        SRTL_DEACTIVATED_INIT_FAILED,
        SRTL_DEACTIVATED_BAD_POSITION,
        SRTL_DEACTIVATED_BAD_POSITION_TIMEOUT,
        SRTL_DEACTIVATED_PATH_FULL_TIMEOUT,
        SRTL_DEACTIVATED_PROGRAM_ERROR,
    };

    // add point to end of path
    bool add_point(const Vector3f& point);

    // routine cleanup attempts to remove 10 points (see SMARTRTL_CLEANUP_POINT_MIN definition) by simplification or loop pruning
    void routine_cleanup(uint16_t path_points_count, uint16_t path_points_complete_limit);

    // thorough cleanup simplifies and prunes all loops.  returns true if the cleanup was completed.
    // path_points_count is _path_points_count but passed in to avoid having to take the semaphore
    bool thorough_cleanup(uint16_t path_points_count, ThoroughCleanupType clean_type);

    // the two cleanup steps run from the background thread
    // these are public so that they can be tested by the example sketch
    void detect_simplifications();
    void detect_loops();

    // restart simplify or pruning if new points have been added to path
    // path_points_count is _path_points_count but passed in to avoid having to take the semaphore
    void restart_simplify_if_new_points(uint16_t path_points_count);

    // restart pruning if new points have been simplified
    void restart_pruning_if_new_points();

    // restart simplify algorithm so that detect_simplify will check all new points that have been added
    // to the path since it last completed.
    // path_points_count is _path_points_count but passed in to avoid having to take the semaphore
    void restart_simplification(uint16_t path_points_count);

    // reset simplify algorithm so that it will re-check all points in the path
    void reset_simplification();

    // restart pruning algorithm so that detect_loops will check all new points that have been added
    // to the path since it last completed.
    // path_points_count is _path_points_count but passed in to avoid having to take the semaphore
    void restart_pruning(uint16_t path_points_count);

    // reset pruning algorithm so that it will re-check all points in the path
    void reset_pruning();

    // remove all simplify-able points from the path
    void remove_points_by_simplify_bitmask();

    // remove loops until at least num_point_to_remove have been removed from path
    // does not necessarily prune all loops
    // returns false if it failed to remove points (because it could not take semaphore)
    bool remove_points_by_loops(uint16_t num_points_to_remove);

    // add loop to loops array
    //  returns true if loop added successfully, false on failure (because loop array is full)
    //  checks if loop overlaps with an existing loop, keeps only the longer loop
    //  example: segment_a(point2~point3) overlaps with segment_b (point5~point6), add_loop(3,5,midpoint)
    bool add_loop(uint16_t start_index, uint16_t end_index, const Vector3f& midpoint);

    // dist_point holds the closest distance reached between 2 line segments, and the point exactly between them
    typedef struct {
        float distance;
        Vector3f midpoint;
    } dist_point;

    // get the closest distance between 2 line segments and the point midway between the closest points
    static dist_point segment_segment_dist(const Vector3f& p1, const Vector3f& p2, const Vector3f& p3, const Vector3f& p4);

    // de-activate SmartRTL, send warning to GCS and log to dataflash
    void deactivate(SRTL_Actions action, const char *reason);

    // logging
    void log_action(SRTL_Actions action, const Vector3f &point = Vector3f());

    // parameters
    AP_Float _accuracy;
    AP_Int16 _points_max;

    // SmartRTL State Variables
    bool _active;       // true if SmartRTL is usable.  may become unusable if the path becomes too long to keep in memory, and too convoluted to be cleaned up, SmartRTL will be permanently deactivated (for the remainder of the flight)
    bool _example_mode; // true when being called from the example sketch, logging and background tasks are disabled
    bool _home_saved;   // true once home has been saved successfully by the set_home or update methods
    uint32_t _last_good_position_ms;    // the last system time a last good position was reported. If no position is available for a while, SmartRTL will be disabled.
    uint32_t _last_position_save_ms;    // the system time a position was saved to the path (used for timeout)
    uint32_t _thorough_clean_request_ms;// the last system time the thorough cleanup was requested (set by thorough_cleanup method, used by background cleanup)
    uint32_t _thorough_clean_complete_ms; // set to _thorough_clean_request_ms when the background thread completes the thorough cleanup
    ThoroughCleanupType _thorough_clean_type;   // used by example sketch to test simplify and prune separately

    // path variables
    Vector3f* _path;    // points are stored in meters from EKF origin in NED
    uint16_t _path_points_max;  // after the array has been allocated, we will need to know how big it is. We can't use the parameter, because a user could change the parameter in-flight
    uint16_t _path_points_count;// number of points in the path array
    uint16_t _path_points_completed_limit;  // set by main thread to the path_point_count when a point is popped.  used by simplify and prune algorithms to detect path shrinking
    HAL_Semaphore _path_sem;   // semaphore for updating path

    // Simplify
    // structure and buffer to hold the "to-do list" for the simplify algorithm.
    typedef struct {
        uint16_t start;
        uint16_t finish;
    } simplify_start_finish_t;
    struct {
        bool complete;          // true after simplify_detection has completed
        bool removal_required;  // true if some simplify-able points have been found on the path, set true by detect_simplifications, set false by remove_points_by_simplify_bitmask
        uint16_t path_points_count; // copy of _path_points_count taken when the simply algorithm started
        uint16_t path_points_completed = SMARTRTL_POINTS_MAX; // number of points in that path that have already been simplified and should be ignored
        simplify_start_finish_t* stack;
        uint16_t stack_max;     // maximum number of elements in the _simplify_stack array
        uint16_t stack_count;   // number of elements in _simplify_stack array
        Bitmask bitmask{SMARTRTL_POINTS_MAX};  // simplify algorithm clears bits for each point that can be removed
    } _simplify;

    // Pruning
    typedef struct {
        uint16_t start_index;   // index of the first point in the loop
        uint16_t end_index;     // index of the last point in the loop
        Vector3f midpoint;      // midpoint which should replace the first point when the loop is removed
        float length_squared;   // length squared (in meters) of the loop (used so we can remove the longest loops)
    } prune_loop_t;
    struct {
        bool complete;
        uint16_t path_points_count;  // copy of _path_points_count taken when the prune algorithm started
        uint16_t path_points_completed; // number of points in that path that have already been checked for loops and should be ignored
        uint16_t i;     // loop search's outer loop index
        uint16_t j;     // loop search's inner loop index
        prune_loop_t* loops;// the result of the pruning algorithm
        uint16_t loops_max; // maximum number of elements in the _prunable_loops array
        uint16_t loops_count;   // number of elements in the _prunable_loops array
    } _prune;

    // returns true if the two loops overlap (used within add_loop to determine which loops to keep or throw away)
    bool loops_overlap(const prune_loop_t& loop1, const prune_loop_t& loop2) const;
};
