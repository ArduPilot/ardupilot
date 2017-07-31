/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "AP_SafeRTL.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_SafeRTL::var_info[] = {
    // @Param: ACCURACY
    // @DisplayName: SafeRTL _accuracy
    // @Description: SafeRTL _accuracy. The minimum distance between points.
    // @Units: m
    // @Range: 0 10
    // @User: Advanced
    AP_GROUPINFO("ACCURACY", 0, AP_SafeRTL, _accuracy, SAFERTL_ACCURACY_DEFAULT),

    // @Param: POINTS
    // @DisplayName: SafeRTL maximum number of points on path
    // @Description: SafeRTL maximum number of points on path. Set to 0 to disable SafeRTL.  100 points consumes about 3k of memory.
    // @Range: 0 500
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("POINTS", 1, AP_SafeRTL, _points_max, SAFERTL_POINTS_DEFAULT),

    AP_GROUPEND
};

/*
*    This library is used for the Safe Return-to-Launch feature. The vehicle's position
*    (aka "bread crumbs") are stored into an array in memory at regular intervals.
*    After a certain number of bread crumbs have been stored and space within the array
*    is low, clean-up algorithms are run to reduce the total number of points.
*    When Safe-RTL is initiated by the vehicle code, a more thorough cleanup runs and
*    the resulting path is fed into navigation controller to return the vehicle to home.
*
*    The cleanup consists of two parts, pruning and simplification:
*
*    1. Pruning calculates the closest distance between two line segments formed by
*    two pairs of sequential points, and then cuts out anything between two points
*    when their line segments get close. This algorithm will never compare two
*    consecutive line segments. Obviously the segments (p1,p2) and (p2,p3) will
*    get very close (they touch), but there would be nothing to trim between them.
*
*    2. Simplification uses the Ramer-Douglas-Peucker algorithm. See Wikipedia for
*    a more complete description.
*
*    The simplification and pruning algorithms run in the background and do not alter
*    the path in memory.  Two definitions, SAFERTL_SIMPLIFY_TIME_US and
*    SAFERTL_LOOP_TIME_US are used to limit how long each algorithm will be run
*    before they save their state and return.
*
*    Both algorithm are "anytime algorithms" meaning they can be interrupted before
*    they complete which is helpful when memory is filling up and we just need to
*    quickly identify a handful of points which can be deleted.
*
*    Once the algorithms have completed the _simplify_complete and _prune_complete
*    flags are set to true.  The "thorough cleanup" procedure which is run as the
*    vehicle initiates RTL, waits for these flags to become true.  This can force
*    the vehicle to pause for a few seconds before initiating the return journey.
*/

AP_SafeRTL::AP_SafeRTL(const AP_AHRS& ahrs, bool example_mode) :
    _ahrs(ahrs),
    _example_mode(example_mode)
{
    AP_Param::setup_object_defaults(this, var_info);
    _simplify_bitmask.setall();
}

AP_SafeRTL::~AP_SafeRTL()
{
    delete _path_sem;
}

// initialise safe rtl including setting up background processes
void AP_SafeRTL::init()
{
    // protect against repeated call to init
    if (_path != nullptr) {
        return;
    }

    // constrain the path length, in case the user decided to make the path unreasonably long.
    _points_max = constrain_int16(_points_max, 0, SAFERTL_POINTS_MAX);

    // check if user has disabled SafeRTL
    if (_points_max == 0 || !is_positive(_accuracy)) {
        return;
    }

    // allocate arrays
    _path = (Vector3f*)calloc(_points_max, sizeof(Vector3f));

    _prunable_loops_max = _points_max * SAFERTL_PRUNING_LOOP_BUFFER_LEN_MULT;
    _prunable_loops = (prune_loop_t*)calloc(_prunable_loops_max, sizeof(prune_loop_t));

    _simplify_stack_max = _points_max * SAFERTL_SIMPLIFY_STACK_LEN_MULT;
    _simplify_stack = (simplify_start_finish_t*)calloc(_simplify_stack_max, sizeof(simplify_start_finish_t));

    // check if memory allocation failed
    if (_path == nullptr || _prunable_loops == nullptr || _simplify_stack == nullptr) {
        log_action(SRTL_DEACTIVATED_INIT_FAILED);
        gcs().send_text(MAV_SEVERITY_WARNING, "SafeRTL deactivated: init failed");
        free(_path);
        free(_prunable_loops);
        free(_simplify_stack);
        return;
    }

    _path_points_max = _points_max;

    // create semaphore
    _path_sem = hal.util->new_semaphore();

    // when running the example sketch, we want the cleanup tasks to run when we tell them to, no in the background (so that they can be timed.)
    if (!_example_mode){
        // register background cleanup to run in IO thread
        hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_SafeRTL::run_background_cleanup, void));
    }
}

// returns number of points on the path
uint16_t AP_SafeRTL::get_num_points() const
{
    return _path_points_count;
}

// get next point on the path to home, returns true on success
bool AP_SafeRTL::pop_point(Vector3f& point)
{
    // check we are active
    if (!_active) {
        return false;
    }

    // get semaphore
    if (!_path_sem->take_nonblocking()) {
        log_action(SRTL_POP_FAILED_NO_SEMAPHORE);
        return false;
    }

    // check we have another point
    if (_path_points_count == 0) {
        _path_sem->give();
        return false;
    }

    // return last point and remove from path
    point = _path[--_path_points_count];

    _path_sem->give();
    return true;
}

// clear return path and set home location.  This should be called as part of the arming procedure
void AP_SafeRTL::reset_path(bool position_ok)
{
    Vector3f current_pos;
    position_ok &= _ahrs.get_relative_position_NED_origin(current_pos);
    reset_path(position_ok, current_pos);
}

void AP_SafeRTL::reset_path(bool position_ok, const Vector3f& current_pos)
{
    if (_path == nullptr) {
        return;
    }

    // clear path
    _path_points_count = 0;

    // reset simplification and pruning.  These functions access members that should normally only
    // be touched by the background thread but it will not be running because active should be false
    reset_simplification(0);
    reset_pruning(0);

    // de-activate if no position at take-off
    if (!position_ok) {
        _active = false;
        log_action(SRTL_DEACTIVATED_BAD_POSITION);
        gcs().send_text(MAV_SEVERITY_WARNING, "SafeRTL deactivated: bad position");
        return;
    }

    // save current position as first point in path
    if (!add_point(current_pos)) {
        _active = false;
        gcs().send_text(MAV_SEVERITY_WARNING, "SafeRTL deactivated: failed to save point");
        return;
    }

    // successfully added point and reset path
    _last_good_position_ms = AP_HAL::millis();
    _active = true;
}

// call this a couple of times per second regardless of what mode the vehicle is in
void AP_SafeRTL::update(bool position_ok, bool save_position)
{
    if (!_active || !position_ok || !save_position) {
        return;
    }

    Vector3f current_pos;
    position_ok &= _ahrs.get_relative_position_NED_origin(current_pos);
    update(position_ok, current_pos);
}

void AP_SafeRTL::update(bool position_ok, const Vector3f& current_pos)
{
    if (!_active) {
        return;
    }

    if (position_ok) {
        uint32_t now = AP_HAL::millis();
        _last_good_position_ms = now;
        // add the point
        if (add_point(current_pos)) {
            _last_position_save_ms = now;
        }
    }

    // check for timeout due to bad position
    if (AP_HAL::millis() - _last_good_position_ms > SAFERTL_TIMEOUT) {
        _active = false;
        log_action(SRTL_DEACTIVATED_BAD_POSITION_TIMEOUT);
        gcs().send_text(MAV_SEVERITY_WARNING,"SafeRTL deactivated: bad position");
        return;
    }

    // check for timeout due to failure to save points to path (most likely due to buffer filling up)
    if (AP_HAL::millis() - _last_position_save_ms > SAFERTL_TIMEOUT) {
        _active = false;
        log_action(SRTL_DEACTIVATED_PATH_FULL_TIMEOUT);
        gcs().send_text(MAV_SEVERITY_WARNING,"SafeRTL deactivated: buffer full");
    }
}

// request thorough cleanup including simplification, pruning and removal of all unnecessary points
// returns true if the thorough cleanup was completed, false if it has not yet completed
// this method should be called repeatedly until it returns true before initiating the return journey
bool AP_SafeRTL::request_thorough_cleanup(ThoroughCleanupType clean_type)
{
    // this should never happen but just in case
    if (!_active) {
        return false;
    }

    // request thorough cleanup
    if (_thorough_clean_request_ms == 0) {
        _thorough_clean_request_ms = AP_HAL::millis();
        if (clean_type != THOROUGH_CLEAN_DEFAULT) {
            _thorough_clean_type = clean_type;
        }
        return false;
    }

    // check if background thread has completed request
    if (_thorough_clean_complete_ms == _thorough_clean_request_ms) {
        _thorough_clean_request_ms = 0;
        return true;
    }

    return false;
}

// cancel request for thorough cleanup
void AP_SafeRTL::cancel_request_for_thorough_cleanup()
{
    _thorough_clean_request_ms = 0;
}

//
// Private methods
//

// add point to end of path (if necessary), returns true on success
bool AP_SafeRTL::add_point(const Vector3f& point)
{
    // get semaphore
    if (!_path_sem->take_nonblocking()) {
        log_action(SRTL_ADD_FAILED_NO_SEMAPHORE, point);
        return false;
    }

    // check if we have traveled far enough
    if (_path_points_count > 0) {
        const Vector3f& last_pos = _path[_path_points_count-1];
        if (last_pos.distance_squared(point) < sq(_accuracy.get())) {
            _path_sem->give();
            return true;
        }
    }

    // check we have space in the path
    if (_path_points_count >= _path_points_max) {
        _path_sem->give();
        log_action(SRTL_ADD_FAILED_PATH_FULL, point);
        return false;
    }

    // add point to path
    _path[_path_points_count++] = point;
    log_action(SRTL_POINT_ADD, point);

    _path_sem->give();
    return true;
}

// run background cleanup - should be run regularly from the IO thread
void AP_SafeRTL::run_background_cleanup()
{
    if (!_active) {
        return;
    }

    // get semaphore
    if (!_path_sem->take_nonblocking()) {
        return;
    }
    // local copy of _path_points_count
    uint16_t path_points_count = _path_points_count;
    _path_sem->give();

    // check if thorough cleanup is required
    if (_thorough_clean_request_ms > 0) {
        // check if we have already completed the request
        if (_thorough_clean_complete_ms != _thorough_clean_request_ms) {
            if (thorough_cleanup(path_points_count, _thorough_clean_type)) {
                // record completion
                _thorough_clean_complete_ms = _thorough_clean_request_ms;
            }
        }
        // we do not perform any further detection or cleanup until the requester acknowledges
        // they have what they need by setting _thorough_clean_request_ms back to zero
        return;
    } else {
        // ensure clean complete time is zero
        _thorough_clean_complete_ms = 0;
    }

    // check if path array is nearly full, if yes we should do a routine cleanup (i.e. remove 10 points)
    bool path_nearly_full = path_points_count >= MAX(_path_points_max - SAFERTL_CLEANUP_START_MARGIN, 0);
    if (path_nearly_full) {
        routine_cleanup();
    }

    // detect simplifications
    if (!_simplify_complete) {
        detect_simplifications();
        return;
    }

    // detect prunable loops
    if (!_prune_complete) {
        detect_loops();
        return;
    }

    // checks if new points have appeared on the path and resets simplification and pruning
    reset_if_new_points(path_points_count);
}

// routine cleanup attempts to remove 10 points (see SAFERTL_CLEANUP_POINT_MIN definition) by simplification or loop pruning
//   it is called from run_background_cleanup if the buffer is nearly full (has only SAFERTL_CLEANUP_START_MARGIN empty slots remaining)
//   this routine first tries to regain 10 points through simplification, failing that it tries to free 10 through pruning loops
//   and finally if neither of these yields 10 points it will remove whatever it can through both simplification and pruning loops
//   the calls to remove_empty_points causes the detect_ algorithms to begin their calculations from scratch
void AP_SafeRTL::routine_cleanup()
{
    uint16_t potential_amount_to_simplify = _simplify_bitmask.size() - _simplify_bitmask.count();

    // if simplifying will remove more than 10 points, just do it
    if (potential_amount_to_simplify >= SAFERTL_CLEANUP_POINT_MIN) {
        // take semaphore to avoid conflicts with new points being added
        if (!_path_sem->take_nonblocking()) {
            return;
        }
        zero_points_by_simplify_bitmask();
        remove_empty_points();
        _path_sem->give();
        return;
    }

    uint16_t potential_amount_to_prune = 0;
    for (uint16_t i = 0; i < _prunable_loops_count; i++) {
        // add 1 at the end, because a pruned loop is always replaced by one new point.
        potential_amount_to_prune += _prunable_loops[i].end_index - _prunable_loops[i].start_index + 1;
    }

    // if pruning could remove 10+ points, prune loops until 10 or more points have been removed (doesn't necessarily prune all loops)
    if (potential_amount_to_prune >= SAFERTL_CLEANUP_POINT_MIN) {
        // take semaphore to avoid conflicts with new points being added
        if (!_path_sem->take_nonblocking()) {
            return;
        }
        zero_points_by_loops(SAFERTL_CLEANUP_POINT_MIN);
        remove_empty_points();
        _path_sem->give();
        return;
    }

    // as a last resort, see if pruning and simplifying together would remove 10+ points.
    if (potential_amount_to_prune + potential_amount_to_simplify >= SAFERTL_CLEANUP_POINT_MIN) {
        // take semaphore to avoid conflicts with new points being added
        if (!_path_sem->take_nonblocking()) {
            return;
        }
        zero_points_by_simplify_bitmask();
        zero_points_by_loops(SAFERTL_CLEANUP_POINT_MIN);
        remove_empty_points();
        _path_sem->give();
    }
}

// thorough cleanup simplifies and prunes all loops.  returns true if the cleanup was completed.
// path_points_count is _path_points_count but passed into avoid having to take the semaphore
bool AP_SafeRTL::thorough_cleanup(uint16_t path_points_count, ThoroughCleanupType clean_type)
{
    // reset simplify and pruning if new points have appeared on path
    reset_if_new_points(path_points_count);

    // if simplification is not complete, run it
    if (!_simplify_complete && (clean_type != THOROUGH_CLEAN_PRUNE_ONLY)) {
        detect_simplifications();
        return false;
    }
    if (!_prune_complete && (clean_type != THOROUGH_CLEAN_SIMPLIFY_ONLY)) {
        detect_loops();
        return false;
    }

    // take semaphore to avoid conflicts with new points being added
    if (!_path_sem->take_nonblocking()) {
        return false;
    }

    // apply simplification
    if (clean_type != THOROUGH_CLEAN_PRUNE_ONLY) {
        zero_points_by_simplify_bitmask();
    }

    // apply pruning, prune every single loop
    if (clean_type != THOROUGH_CLEAN_SIMPLIFY_ONLY) {
        zero_points_by_loops(SAFERTL_POINTS_MAX);
    }

    // remove all simplified and pruned points
    remove_empty_points();

    _path_sem->give();

    return true;
}

// Simplifies a 3D path, according to the Ramer-Douglas-Peucker algorithm.
// _simplify_complete is set to true when all simplifications on the path have been identified
void AP_SafeRTL::detect_simplifications()
{
    // complete immediately if only one segment
    if (_simplify_path_points_count < 3) {
        _simplify_complete = true;
        return;
    }

    // if not complete but also nothing to do, we must be restarting
    if (_simplify_stack_count == 0) {
        // reset to beginning state.  a single element in the array with start = first path point, finish = final path point
        _simplify_stack[0].start = 0;
        _simplify_stack[0].finish = _simplify_path_points_count-1;
        _simplify_stack_count++;
    }

    const uint32_t start_time_us = AP_HAL::micros();
    while (_simplify_stack_count > 0) { // while there is something to do

        // if this method has run for long enough, exit
        if (AP_HAL::micros() - start_time_us > SAFERTL_SIMPLIFY_TIME_US) {
            return;
        }

        // pop last item off the simplify stack
        const simplify_start_finish_t tmp = _simplify_stack[--_simplify_stack_count];
        const uint16_t start_index = tmp.start;
        const uint16_t end_index = tmp.finish;

        // find the point between start and end points that is farthest from the start-end line segment
        float max_dist = 0.0f;
        uint16_t farthest_point_index = start_index;
        for (uint16_t i = start_index + 1; i < end_index; i++) {
            // only check points that have not already been flagged for simplification
            if (_simplify_bitmask.get(i)) {
                const float dist = _path[i].distance_to_segment(_path[start_index], _path[end_index]);
                if (dist > max_dist) {
                    farthest_point_index = i;
                    max_dist = dist;
                }
            }
        }

        // if the farthest point is more than ACCURACY * 0.5 add two new elements to the _simplification_stack
        // so that on the next iteration we will check between start-to-farthestpoint and farthestpoint-to-end
        if (max_dist > SAFERTL_SIMPLIFY_EPSILON) {
            // if the to-do list is full, give up on simplifying. This should never happen.
            if (_simplify_stack_count >= _simplify_stack_max) {
                _simplify_complete = true;
                return;
            }
            _simplify_stack[_simplify_stack_count++] = simplify_start_finish_t {start_index, farthest_point_index};
            _simplify_stack[_simplify_stack_count++] = simplify_start_finish_t {farthest_point_index, end_index};
        } else {
            // if the farthest point was closer than ACCURACY * 0.5 we can simplify all points between start and end
            for (uint16_t i = start_index + 1; i < end_index; i++) {
                _simplify_bitmask.clear(i);
            }
        }
    }
    _simplify_complete = true;
}

/**
*   This method runs for the allotted time, and detects loops in a path. Any detected loops are added to _prunable_loops,
*   this function does not alter the path in memory. It works by comparing the line segment between any two sequential points
*   to the line segment between any other two sequential points. If they get close enough, anything between them could be pruned.
*
*   reset_pruning should have been called at least once before this function is called to setup the indexes (_prune_i, etc)
*/
void AP_SafeRTL::detect_loops()
{
    // if there are less than 4 points (3 segments), mark complete
    if (_prune_path_points_count < 4) {
        _prune_complete = true;
        return;
    }

    // capture start time
    const uint32_t start_time_us = AP_HAL::micros();

    // run for defined amount of time
    while (AP_HAL::micros() - start_time_us < SAFERTL_LOOP_TIME_US) {

        // advance inner loop
        _prune_j++;
        if (_prune_j > _prune_path_points_count-2) {
            // advance outer loop
            _prune_i++;
            if (_prune_i > _prune_path_points_count - 4) {
                _prune_complete = true;
                return;
            }
            // push inner loop to start from outer loop+2 and skip over known loops
            _prune_j = MAX(_prune_i + 2, _prune_j_min);
        }

        // find the closest distance between two line segments and the mid-point
        dist_point dp = segment_segment_dist(_path[_prune_i], _path[_prune_i+1], _path[_prune_j], _path[_prune_j+1]);
        if (dp.distance < SAFERTL_PRUNING_DELTA) { // if there is a loop here
            // if the buffer is full, stop trying to prune
            if (_prunable_loops_count >= _prunable_loops_max) {
                _prune_complete = true;
                return;
            }
            // add loop to _prunable_loops array
            _prunable_loops[_prunable_loops_count].start_index = _prune_i + 1;
            _prunable_loops[_prunable_loops_count].end_index = _prune_j + 1;
            _prunable_loops[_prunable_loops_count].midpoint = dp.midpoint;
            _prunable_loops_count++;
            // record inner loop should start no lower than 2nd segment
            _prune_j_min = _prune_j + 1;
        }
    }
}

// reset simplification and pruning if new points have been added to path
// path_points_count is _path_points_count but passed into avoid having to take the semaphore
void AP_SafeRTL::reset_if_new_points(uint16_t path_points_count)
{
    // any difference in the number of points is because of new points being added to path
    if (_simplify_path_points_count != path_points_count) {
        reset_simplification(path_points_count);
    }
    if (_prune_path_points_count != path_points_count) {
        reset_pruning(path_points_count);
    }
}

// reset simplification algorithm so that it will re-check all points in the path
// should be called if the existing path is altered for example when a loop as been removed
void AP_SafeRTL::reset_simplification(uint16_t path_points_count)
{
    _simplify_complete = false;
    _simplify_stack_count = 0;
    _simplify_bitmask.setall();
    _simplify_path_points_count = path_points_count;
}

// reset pruning algorithm so that it will re-check all points in the path
// should be called if the existing path is altered for example when a loop as been removed
void AP_SafeRTL::reset_pruning(uint16_t path_points_count)
{
    _prune_complete = false;
    _prune_i = 0;
    _prune_j = _prune_i+1;  // detect_loops will increment this to the correct starting point of _prune_i+2.
    _prune_j_min = _prune_j;
    _prunable_loops_count = 0; // clear the loops that we've recorded
    _prune_path_points_count = path_points_count;
}

// set all points that can be removed to zero
void AP_SafeRTL::zero_points_by_simplify_bitmask()
{
    for (uint16_t i = 0; i < _path_points_count; i++) {
        if (!_simplify_bitmask.get(i)) {
            if (!_path[i].is_zero()) {
                log_action(SRTL_POINT_SIMPLIFY, _path[i]);
                _path[i].zero();
            }
        }
    }
}

// prunes loops until points_to_delete points have been removed. It does not necessarily prune all loops.
void AP_SafeRTL::zero_points_by_loops(uint16_t points_to_delete)
{
    uint16_t removed_points = 0;
    for (uint16_t i = 0; i < _prunable_loops_count; i++) {
        prune_loop_t l = _prunable_loops[i];
        for (uint16_t j = l.start_index; j < l.end_index; j++) {
            // zero this point if it wasn't already zeroed
            if (!_path[j].is_zero()) {
                log_action(SRTL_POINT_PRUNE, _path[j]);
                _path[j].zero();
            }
        }
        _path[(uint16_t)((l.start_index+l.end_index)/2.0)] = l.midpoint;
        removed_points += l.end_index - l.start_index - 1;
        if (removed_points > points_to_delete) {
            return;
        }
    }
}

/**
*  Removes all 0,0,0 points from the path, and shifts remaining items to correct position.
*  The first item will not be removed.
*/
void AP_SafeRTL::remove_empty_points()
{
    uint16_t src = 0;
    uint16_t dest = 0;
    uint16_t removed = 0;
    while (++src < _path_points_count) { // never removes the first point
        if (!_path[src].is_zero()) {
            _path[++dest] = _path[src];
        } else {
            removed++;
        }
    }
    _path_points_count -= removed;

    // reset state of simplification and pruning
    reset_simplification(_path_points_count);
    reset_pruning(_path_points_count);
}

/**
*  Returns the closest distance in 3D space between any part of two input segments, defined from p1 to p2 and from p3 to p4.
*  Also returns the point which is halfway between
*
*  Limitation: This function does not work for parallel lines. In this case, it will return FLT_MAX. This does not matter for the path cleanup algorithm because
*  the pruning will still occur fine between the first parallel segment and a segment which is directly before or after the second segment.
*/
AP_SafeRTL::dist_point AP_SafeRTL::segment_segment_dist(const Vector3f &p1, const Vector3f &p2, const Vector3f &p3, const Vector3f &p4)
{
    Vector3f line1 = p2-p1;
    Vector3f line2 = p4-p3;
    Vector3f line_start_diff = p1-p3; // from the beginning of the second line to the beginning of the first line

    // these don't really have a physical representation. They're only here to break up the longer formulas below.
    float a = line1*line1;
    float b = line1*line2;
    float c = line2*line2;
    float d = line1*line_start_diff;
    float e = line2*line_start_diff;

    // the parameter for the position on line1 and line2 which define the closest points.
    float t1 = 0.0f;
    float t2 = 0.0f;

    // if lines are almost parallel, return a garbage answer. This is irrelevant, since the loop
    // could always be pruned start/end of the previous/subsequent line segment
    if (is_zero((a*c)-(b*b))) {
        return {FLT_MAX, Vector3f(0.0f, 0.0f, 0.0f)};
    } else {
        t1 = (b*e-c*d)/(a*c-b*b);
        t2 = (a*e-b*d)/(a*c-b*b);

        // restrict both parameters between 0 and 1.
        t1 = constrain_float(t1, 0.0f, 1.0f);
        t2 = constrain_float(t2, 0.0f, 1.0f);

        // difference between two closest points
        Vector3f dP = line_start_diff+line1*t1-line2*t2;

        Vector3f midpoint = (p1+line1*t1 + p3+line2*t2)/2.0f;
        return {dP.length(), midpoint};
    }
}

// logging
void AP_SafeRTL::log_action(SRTL_Actions action, const Vector3f point)
{
    if (!_example_mode) {
        DataFlash_Class::instance()->Log_Write_SRTL(_active, _path_points_count, _path_points_max, action, point);
    }
}
