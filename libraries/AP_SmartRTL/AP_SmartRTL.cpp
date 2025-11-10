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

#include "AP_SmartRTL.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_SmartRTL::var_info[] = {
    // @Param: ACCURACY
    // @DisplayName: SmartRTL accuracy
    // @Description: SmartRTL accuracy. The minimum distance between points.
    // @Units: m
    // @Range: 0 10
    // @User: Advanced
    AP_GROUPINFO("ACCURACY", 0, AP_SmartRTL, _accuracy, SMARTRTL_ACCURACY_DEFAULT),

    // @Param: POINTS
    // @DisplayName: SmartRTL maximum number of points on path
    // @Description: SmartRTL maximum number of points on path. Set to 0 to disable SmartRTL.  100 points consumes about 3k of memory.
    // @Range: 0 500
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("POINTS", 1, AP_SmartRTL, _points_max, SMARTRTL_POINTS_DEFAULT),

    // @Param: OPTIONS
    // @DisplayName: SmartRTL options
    // @Description: Bitmask of SmartRTL options.
    // @Bitmask: 2:Ignore pilot yaw
    // @User: Standard
    AP_GROUPINFO("OPTIONS", 2, AP_SmartRTL, _options, 0),

    AP_GROUPEND
};

/*
*    This library is used for the Safe Return-to-Launch feature. The vehicle's
*    position (aka "bread crumbs") are stored into an array in memory at
*    regular intervals.  After a certain number of bread crumbs have been
*    stored and space within the array is low, clean-up algorithms are run to
*    reduce the total number of points.  When Safe-RTL is initiated by the
*    vehicle code, a more thorough cleanup runs and the resulting path is fed
*    into navigation controller to return the vehicle to home.
*
*    The cleanup consists of two parts, pruning and simplification:
*
*    1. Pruning calculates the closest distance between two line segments formed
*    by two pairs of sequential points, and then cuts out anything between two
*    points when their line segments get close. This algorithm will never
*    compare two consecutive line segments. Obviously the segments (p1,p2) and
*    (p2,p3) will get very close (they touch), but there would be nothing to
*    trim between them.
*
*    2. Simplification uses the Ramer-Douglas-Peucker algorithm. See Wikipedia
*    for a more complete description.
*
*    The simplification and pruning algorithms run in the background and do not
*    alter the path in memory.  Two definitions, SMARTRTL_SIMPLIFY_TIME_US and
*    SMARTRTL_PRUNING_LOOP_TIME_US are used to limit how long each algorithm will
*    be run before they save their state and return.
*
*    Both algorithms are "anytime algorithms" meaning they can be interrupted
*    before they complete which is helpful when memory is filling up and we just
*    need to quickly identify a handful of points which can be deleted.
*
*    Once the algorithms have completed the simplify.complete and
*    prune.complete flags are set to true.  The "thorough cleanup" procedure,
*    which is run as the vehicle initiates the SmartRTL flight mode, waits for
*    these flags to become true.  This can force the vehicle to pause for a few
*    seconds before initiating the return journey.
*/

AP_SmartRTL::AP_SmartRTL(bool example_mode) :
    _example_mode(example_mode)
{
    AP_Param::setup_object_defaults(this, var_info);
    _simplify.bitmask.setall();
}

// initialise safe rtl including setting up background processes
void AP_SmartRTL::init()
{
    // protect against repeated call to init
    if (_path != nullptr) {
        return;
    }

    // constrain the path length, in case the user decided to make the path unreasonably long.
    _points_max.set(constrain_int16(_points_max, 0, SMARTRTL_POINTS_MAX));

    // check if user has disabled SmartRTL
    if (_points_max == 0 || !is_positive(_accuracy)) {
        return;
    }

    // allocate arrays
    _path = (Vector3f*)calloc(_points_max, sizeof(Vector3f));

    _prune.loops_max = _points_max * SMARTRTL_PRUNING_LOOP_BUFFER_LEN_MULT;
    _prune.loops = (prune_loop_t*)calloc(_prune.loops_max, sizeof(prune_loop_t));

    _simplify.stack_max = _points_max * SMARTRTL_SIMPLIFY_STACK_LEN_MULT;
    _simplify.stack = (simplify_start_finish_t*)calloc(_simplify.stack_max, sizeof(simplify_start_finish_t));

    // check if memory allocation failed
    if (_path == nullptr || _prune.loops == nullptr || _simplify.stack == nullptr) {
        log_action(Action::DEACTIVATED_INIT_FAILED);
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SmartRTL deactivated: init failed");
        free(_path);
        free(_prune.loops);
        free(_simplify.stack);
        return;
    }

    _path_points_max = _points_max;

    // when running the example sketch, we want the cleanup tasks to run when we tell them to, not in the background (so that they can be timed.)
    if (!_example_mode){
        // register background cleanup to run in IO thread
        hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_SmartRTL::run_background_cleanup, void));
    }
}

// returns number of points on the path
uint16_t AP_SmartRTL::get_num_points() const
{
    return _path_points_count;
}

// get next point on the path to home, returns true on success
bool AP_SmartRTL::pop_point(Vector3p& point)
{
    // check we are active
    if (!_active) {
        return false;
    }

    // get semaphore
    if (!_path_sem.take_nonblocking()) {
        log_action(Action::POP_FAILED_NO_SEMAPHORE);
        return false;
    }

    // check we have another point
    if (_path_points_count == 0) {
        _path_sem.give();
        return false;
    }

    // return last point and remove from path
    point = _path[--_path_points_count].topostype();

    // record count of last point popped
    _path_points_completed_limit = _path_points_count;

    _path_sem.give();
    return true;
}

// peek at next point on the path without removing it form the path. Returns true on success
bool AP_SmartRTL::peek_point(Vector3p& point)
{
    // check we are active
    if (!_active) {
        return false;
    }

    // get semaphore
    if (!_path_sem.take_nonblocking()) {
        log_action(Action::PEEK_FAILED_NO_SEMAPHORE);
        return false;
    }

    // check we have another point
    if (_path_points_count == 0) {
        _path_sem.give();
        return false;
    }

    // return last point
    point = _path[_path_points_count-1].topostype();

    _path_sem.give();
    return true;
}

// clear return path and set home location.  This should be called as part of the arming procedure
void AP_SmartRTL::set_home(bool position_ok)
{
    Vector3p current_pos;
    position_ok &= AP::ahrs().get_relative_position_NED_origin(current_pos);
    set_home(position_ok, current_pos);
}

void AP_SmartRTL::set_home(bool position_ok, const Vector3p& current_pos)
{
    if (_path == nullptr) {
        return;
    }

    // clear path
    _path_points_count = 0;

    // reset simplification and pruning.  These functions access members that should normally only
    // be touched by the background thread but it will not be running because active should be false
    reset_simplification();
    reset_pruning();

    // don't continue if no position at take-off
    if (!position_ok) {
        return;
    }

    // save current position as first point in path
    if (!add_point(current_pos)) {
        return;
    }

    // successfully added point and reset path
    const uint32_t now = AP_HAL::millis();
    _last_good_position_ms = now;
    _last_position_save_ms = now;
    _active = true;
    _home_saved = true;
}

// call this at 3hz (or higher) regardless of what mode the vehicle is in
void AP_SmartRTL::update(bool position_ok, bool save_position)
{
    // try to save home if not already saved
    if (position_ok && !_home_saved) {
        set_home(true);
    }

    if (!_active || !save_position) {
        return;
    }

    Vector3p current_pos;
    position_ok &= AP::ahrs().get_relative_position_NED_origin(current_pos);
    update(position_ok, current_pos);
}

void AP_SmartRTL::update(bool position_ok, const Vector3p& current_pos)
{
    if (!_active) {
        return;
    }

    if (position_ok) {
        const uint32_t now = AP_HAL::millis();
        _last_good_position_ms = now;
        // add the point
        if (add_point(current_pos)) {
            _last_position_save_ms = now;
        } else if (AP_HAL::millis() - _last_position_save_ms > SMARTRTL_TIMEOUT) {
            // deactivate after timeout due to failure to save points to path (most likely due to buffer filling up)
            deactivate(Action::DEACTIVATED_PATH_FULL_TIMEOUT, "buffer full");
        }
    } else {
        // check for timeout due to bad position
        if (AP_HAL::millis() - _last_good_position_ms > SMARTRTL_TIMEOUT) {
            deactivate(Action::DEACTIVATED_BAD_POSITION_TIMEOUT, "bad position");
            return;
        }
    }
}

// request thorough cleanup including simplification, pruning and removal of all unnecessary points
// returns true if the thorough cleanup was completed, false if it has not yet completed
// this method should be called repeatedly until it returns true before initiating the return journey
bool AP_SmartRTL::request_thorough_cleanup(ThoroughCleanupType clean_type)
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
void AP_SmartRTL::cancel_request_for_thorough_cleanup()
{
    _thorough_clean_request_ms = 0;
}

//
// Private methods
//

// add point to end of path (if necessary), returns true on success
bool AP_SmartRTL::add_point(const Vector3p& point)
{
    // get semaphore
    if (!_path_sem.take_nonblocking()) {
        log_action(Action::ADD_FAILED_NO_SEMAPHORE, point.tofloat());
        return false;
    }

    // check if we have traveled far enough
    if (_path_points_count > 0) {
        const Vector3p& last_pos = _path[_path_points_count-1].topostype();
        if (last_pos.distance_squared(point) < sq(_accuracy.get())) {
            _path_sem.give();
            return true;
        }
    }

    // check we have space in the path
    if (_path_points_count >= _path_points_max) {
        _path_sem.give();
        log_action(Action::ADD_FAILED_PATH_FULL, point.tofloat());
        return false;
    }

    // add point to path
    _path[_path_points_count++] = point.tofloat();
    log_action(Action::POINT_ADD, point.tofloat());

    _path_sem.give();
    return true;
}

// run background cleanup - should be run regularly from the IO thread
void AP_SmartRTL::run_background_cleanup()
{
    if (!_active) {
        return;
    }

    // get semaphore
    if (!_path_sem.take_nonblocking()) {
        return;
    }
    // local copy of _path_points_count and _path_points_completed_limit
    const uint16_t path_points_count = _path_points_count;
    const uint16_t path_points_completed_limit = _path_points_completed_limit;
    _path_points_completed_limit = SMARTRTL_POINTS_MAX;
    _path_sem.give();

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
    }

    // ensure clean complete time is zero
    _thorough_clean_complete_ms = 0;

    // perform routine cleanup which removes 10 to 50 points if possible
    routine_cleanup(path_points_count, path_points_completed_limit);

    // warn if buffer is about to be filled
    uint32_t now_ms = AP_HAL::millis();
    if ((path_points_count >0) && (path_points_count >= _path_points_max - 9) && (now_ms - _last_low_space_notify_ms > 10000)) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SmartRTL Low on space!");
       _last_low_space_notify_ms = now_ms;
    }

}

// routine cleanup is called regularly from run_background_cleanup
//   simplifies the path after SMARTRTL_CLEANUP_POINT_TRIGGER points (50 points) have been added OR
//   SMARTRTL_CLEANUP_POINT_MIN (10 points) have been added and the path has less than SMARTRTL_CLEANUP_START_MARGIN spaces (10 spaces) remaining
//   prunes the path if the path has less than SMARTRTL_CLEANUP_START_MARGIN spaces (10 spaces) remaining
void AP_SmartRTL::routine_cleanup(uint16_t path_points_count, uint16_t path_points_completed_limit)
{
    // if simplify is running, let it run to completion
    if (!_simplify.complete) {
        detect_simplifications();
        return;
    }

    // remove simplified from path if required
    if (_simplify.removal_required) {
        remove_points_by_simplify_bitmask();
        return;
    }

    // if necessary restart detect_pruning up to last point simplified
    if (_prune.complete) {
        restart_pruning_if_new_points();
    }
    // if pruning is running, let it run to completion
    if (!_prune.complete) {
        detect_loops();
        return;
    }

    // detect path shrinkage and reduce simplify and prune path_points_completed count
    if (_simplify.path_points_completed > path_points_completed_limit) {
        _simplify.path_points_completed = path_points_completed_limit;
    }
    if (_prune.path_points_completed > path_points_completed_limit) {
        _prune.path_points_completed = path_points_completed_limit;
    }

    // calculate the number of points we could simplify
    const uint16_t points_to_simplify = (path_points_count > _simplify.path_points_completed) ? (path_points_count - _simplify.path_points_completed) : 0 ;
    const bool low_on_space = (_path_points_max - path_points_count) <= SMARTRTL_CLEANUP_START_MARGIN;

    // if 50 points can be simplified or we are low on space and at least 10 points can be simplified
    if ((points_to_simplify >= SMARTRTL_CLEANUP_POINT_TRIGGER) || (low_on_space && (points_to_simplify >= SMARTRTL_CLEANUP_POINT_MIN))) {
        restart_simplification(path_points_count);
        return;
    }

    // we are low on space, prune
    if (low_on_space) {
        // remove at least 10 points
        remove_points_by_loops(SMARTRTL_CLEANUP_POINT_MIN);
    }
}

// thorough cleanup simplifies and prunes all loops.  returns true if the cleanup was completed.
// path_points_count is _path_points_count but passed in to avoid having to take the semaphore
bool AP_SmartRTL::thorough_cleanup(uint16_t path_points_count, ThoroughCleanupType clean_type)
{
    if (clean_type != THOROUGH_CLEAN_PRUNE_ONLY) {
        // restart simplify if new points have appeared on path
        if (_simplify.complete) {
            restart_simplify_if_new_points(path_points_count);
        }
        // if simplification is not complete, run it
        if (!_simplify.complete) {
            detect_simplifications();
            return false;
        }
        // remove simplified points from path if required
        if (_simplify.removal_required) {
            remove_points_by_simplify_bitmask();
            return false;
        }
    }

    if (clean_type != THOROUGH_CLEAN_SIMPLIFY_ONLY) {
        // if necessary restart detect_pruning up to last point simplified
        if (_prune.complete) {
            restart_pruning_if_new_points();
        }
        // if pruning is not complete, run it
        if (!_prune.complete) {
            detect_loops();
            return false;
        }
        // remove pruning points
        if (!remove_points_by_loops(SMARTRTL_POINTS_MAX)) {
            return false;
        }
    }

    return true;
}

// Simplifies a 3D path, according to the Ramer-Douglas-Peucker algorithm.
// _simplify.complete is set to true when all simplifications on the path have been identified
void AP_SmartRTL::detect_simplifications()
{
    // complete immediately if only one segment
    if (_simplify.path_points_count < 3) {
        _simplify.complete = true;
        return;
    }

    // if not complete but also nothing to do, we must be restarting
    if (_simplify.stack_count == 0) {
        // reset to beginning state. add a single element in the array with:
        //   start = first path point OR the index of the last already-simplified point
        //   finish = final path point
        _simplify.stack[0].start = (_simplify.path_points_completed > 0) ? _simplify.path_points_completed - 1 : 0;
        _simplify.stack[0].finish = _simplify.path_points_count-1;
        _simplify.stack_count++;
    }

    const uint32_t start_time_us = AP_HAL::micros();
    while (_simplify.stack_count > 0) { // while there is something to do

        // if this method has run for long enough, exit
        if (AP_HAL::micros() - start_time_us > SMARTRTL_SIMPLIFY_TIME_US) {
            return;
        }

        // pop last item off the simplify stack
        const simplify_start_finish_t tmp = _simplify.stack[--_simplify.stack_count];
        const uint16_t start_index = tmp.start;
        const uint16_t end_index = tmp.finish;

        // find the point between start and end points that is farthest from the start-end line segment
        float max_dist = 0.0f;
        uint16_t farthest_point_index = start_index;
        for (uint16_t i = start_index + 1; i < end_index; i++) {
            // only check points that have not already been flagged for simplification
            if (_simplify.bitmask.get(i)) {
                const float dist = _path[i].distance_to_segment(_path[start_index], _path[end_index]);
                if (dist > max_dist) {
                    farthest_point_index = i;
                    max_dist = dist;
                }
            }
        }

        // if the farthest point is more than ACCURACY * 0.5 add two new elements to the _simplification_stack
        // so that on the next iteration we will check between start-to-farthestpoint and farthestpoint-to-end
        if (max_dist > SMARTRTL_SIMPLIFY_EPSILON) {
            // if the to-do list is full, give up on simplifying. This should never happen.
            if (_simplify.stack_count >= _simplify.stack_max) {
                _simplify.complete = true;
                return;
            }
            _simplify.stack[_simplify.stack_count++] = simplify_start_finish_t {start_index, farthest_point_index};
            _simplify.stack[_simplify.stack_count++] = simplify_start_finish_t {farthest_point_index, end_index};
        } else {
            // if the farthest point was closer than ACCURACY * 0.5 we can simplify all points between start and end
            for (uint16_t i = start_index + 1; i < end_index; i++) {
                _simplify.bitmask.clear(i);
                _simplify.removal_required = true;
            }
        }
    }
    _simplify.path_points_completed = _simplify.path_points_count;
    _simplify.complete = true;
}

/**
*   This method runs for the allotted time, and detects loops in a path. Any detected loops are added to _prune.loops,
*   this function does not alter the path in memory. It works by comparing the line segment between any two sequential points
*   to the line segment between any other two sequential points. If they get close enough, anything between them could be pruned.
*
*   reset_pruning should have been called at least once before this function is called to setup the indexes (_prune.i, etc)
*/
void AP_SmartRTL::detect_loops()
{
    // if there are less than 4 points (3 segments), mark complete
    if (_prune.path_points_count < 4) {
        _prune.complete = true;
        return;
    }

    // capture start time
    const uint32_t start_time_us = AP_HAL::micros();

    // run for defined amount of time
    while (AP_HAL::micros() - start_time_us < SMARTRTL_PRUNING_LOOP_TIME_US) {

        // advance inner loop
        _prune.j++;
        if (_prune.j > _prune.i - 2) {
            // set inner loop back to first point
            _prune.j = 1;
            // reduce outer loop
            _prune.i--;
            // complete when outer loop has run out of new points to check
            if (_prune.i < 4 || _prune.i < _prune.path_points_completed) {
                _prune.complete = true;
                _prune.path_points_completed = _prune.path_points_count;
                return;
            }
        }

        // find the closest distance between two line segments and the mid-point
        dist_point dp = segment_segment_dist(_path[_prune.i], _path[_prune.i-1], _path[_prune.j-1], _path[_prune.j]);
        if (dp.distance < SMARTRTL_PRUNING_DELTA) {
            // if there is a loop here, add to loop array
            if (!add_loop(_prune.j, _prune.i-1, dp.midpoint)) {
                // if the buffer is full, stop trying to prune
                _prune.complete = true;
            }
            // set inner loop forward to trigger outer loop move to next segment
            _prune.j = _prune.i;
        }
    }
}

// restart simplify if new points have been added to path
// path_points_count is _path_points_count but passed in to avoid having to take the semaphore
void AP_SmartRTL::restart_simplify_if_new_points(uint16_t path_points_count)
{
    // any difference in the number of points is because of new points being added to path
    if (_simplify.path_points_count != path_points_count) {
        restart_simplification(path_points_count);
    }
}

// reset pruning if new points have been simplified
void AP_SmartRTL::restart_pruning_if_new_points()
{
    // any difference in the number of points is because of new points being added to path
    if (_prune.path_points_count != _simplify.path_points_completed) {
        restart_pruning(_simplify.path_points_completed);
    }
}

// restart simplification algorithm so that it will check new points in the path
void AP_SmartRTL::restart_simplification(uint16_t path_points_count)
{
    _simplify.complete = false;
    _simplify.removal_required = false;
    _simplify.bitmask.setall();
    _simplify.stack_count = 0;
    _simplify.path_points_count = path_points_count;
}

// reset simplification algorithm so that it will re-check all points in the path
void AP_SmartRTL::reset_simplification()
{
    restart_simplification(0);
    _simplify.path_points_completed = 0;
}

// restart pruning algorithm to check new points that have arrived
void AP_SmartRTL::restart_pruning(uint16_t path_points_count)
{
    _prune.complete = false;
    _prune.i = (path_points_count > 0) ? path_points_count - 1 : 0;
    _prune.j = 0;
    _prune.path_points_count = path_points_count;
}

// reset pruning algorithm so that it will re-check all points in the path
void AP_SmartRTL::reset_pruning()
{
    restart_pruning(0);
    _prune.loops_count = 0; // clear the loops that we've recorded
    _prune.path_points_completed = 0;
}

// remove all simplify-able points from the path
void AP_SmartRTL::remove_points_by_simplify_bitmask()
{
    // get semaphore before modifying path
    if (!_path_sem.take_nonblocking()) {
        return;
    }
    uint16_t dest = 1;
    uint16_t removed = 0;
    for (uint16_t src = 1; src < _path_points_count; src++) {
        if (!_simplify.bitmask.get(src)) {
            log_action(Action::POINT_SIMPLIFY, _path[src]);
            removed++;
        } else {
            _path[dest] = _path[src];
            dest++;
        }
    }

    // reduce count of the number of points simplified
    if (_path_points_count > removed && _simplify.path_points_count > removed) {
        _path_points_count -= removed;
        _simplify.path_points_count -= removed;
        _simplify.path_points_completed = _simplify.path_points_count;
    } else {
        // this is an error that should never happen so deactivate
        deactivate(Action::DEACTIVATED_PROGRAM_ERROR, "program error");
    }

    _path_sem.give();

    // flag point removal is complete
    _simplify.bitmask.setall();
    _simplify.removal_required = false;
}

// remove loops until at least num_point_to_delete have been removed from path
// does not necessarily prune all loops
// returns false if it failed to remove points (because it could not take semaphore)
bool AP_SmartRTL::remove_points_by_loops(uint16_t num_points_to_remove)
{
    // exit immediately if no loops to prune
    if (_prune.loops_count == 0) {
        return true;
    }

    // get semaphore before modifying path
    if (!_path_sem.take_nonblocking()) {
        return false;
    }

    uint16_t removed_points = 0;
    uint16_t i = _prune.loops_count;
    while ((i > 0) && (removed_points < num_points_to_remove)) {
        i--;
        prune_loop_t loop = _prune.loops[i];

        // midpoint goes into start_index (this is the end point of the first segment)
        _path[loop.start_index] = loop.midpoint;

        // shift points after the end of the loop down by the number of points in the loop
        uint16_t loop_num_points_to_remove = loop.end_index - loop.start_index;
        for (uint16_t dest = loop.start_index + 1; dest < _path_points_count - loop_num_points_to_remove; dest++) {
            log_action(Action::POINT_PRUNE, _path[dest]);
            _path[dest] = _path[dest + loop_num_points_to_remove];
        }

        if (_path_points_count > loop_num_points_to_remove) {
            _path_points_count -= loop_num_points_to_remove;
            removed_points += loop_num_points_to_remove;
        } else {
            // this is an error that should never happen so deactivate
            deactivate(Action::DEACTIVATED_PROGRAM_ERROR, "program error");
            _path_sem.give();
            // we return true so thorough_cleanup does not get stuck
            return true;
        }

        // fix the indices of any existing prune loops
        // we do not check for overlapping loops because add_loops should have caught them
        for (uint16_t loop_cnt = 0; loop_cnt < i; loop_cnt++) {
            if (_prune.loops[loop_cnt].start_index >= loop.end_index) {
                _prune.loops[loop_cnt].start_index -= loop_num_points_to_remove;
            }
            if (_prune.loops[loop_cnt].end_index >= loop.end_index) {
                _prune.loops[loop_cnt].end_index -= loop_num_points_to_remove;
            }
        }

        // remove last prune loop from array
        _prune.loops_count--;
    }

    _path_sem.give();
    return true;
}

// add loop to loops array
//  returns true if loop added successfully, false if loop array is full
//  checks if loop overlaps with an existing loop, keeps only the longer loop
bool AP_SmartRTL::add_loop(uint16_t start_index, uint16_t end_index, const Vector3f& midpoint)
{
    // if the buffer is full, return failure
    if (_prune.loops_count >= _prune.loops_max) {
        return false;
    }

    // sanity check indices
    if (end_index <= start_index) {
        return false;
    }

    // create new loop structure and calculate length squared of loop
    prune_loop_t new_loop = {start_index, end_index, midpoint, 0.0f};
    new_loop.length_squared = midpoint.distance_squared(_path[start_index]) + midpoint.distance_squared(_path[end_index]);
    for (uint16_t i = start_index; i < end_index; i++) {
        new_loop.length_squared += _path[i].distance_squared(_path[i+1]);
    }

    // look for overlapping loops and find their combined length
    bool overlapping_loops = false;
    float overlapping_loop_length = 0.0f;
    for (uint16_t loop_idx = 0; loop_idx < _prune.loops_count; loop_idx++) {
        if (loops_overlap(_prune.loops[loop_idx], new_loop)) {
            overlapping_loops = true;
            overlapping_loop_length += _prune.loops[loop_idx].length_squared;
        }
    }

    // handle overlapping loops
    if (overlapping_loops) {
        // if adding this loop would lengthen the path, discard the new loop but return success
        if (overlapping_loop_length > new_loop.length_squared) {
            return true;
        }
        // remove overlapping loops
        uint16_t dest_idx = 0;
        uint16_t removed = 0;
        for (uint16_t src_idx = 0; src_idx < _prune.loops_count; src_idx++) {
            if (loops_overlap(_prune.loops[src_idx], new_loop)) {
                removed++;
            } else {
                _prune.loops[dest_idx] = _prune.loops[src_idx];
                dest_idx++;
            }
        }
        _prune.loops_count -= removed;
    }

    // add new loop to _prune.loops array
    _prune.loops[_prune.loops_count] = new_loop;
    _prune.loops_count++;
    return true;
}

/**
*  Returns the closest distance in 3D space between any part of two input segments, defined from p1 to p2 and from p3 to p4.
*  Also returns the point which is halfway between
*
*  Limitation: This function does not work for parallel lines. In this case, dist_point.distance will be FLT_MAX.
*  This does not matter for the path cleanup algorithm because the pruning will still occur fine between the first
*  parallel segment and a segment which is directly before or after the second segment.
*/
AP_SmartRTL::dist_point AP_SmartRTL::segment_segment_dist(const Vector3f &p1, const Vector3f &p2, const Vector3f &p3, const Vector3f &p4)
{
    const Vector3f line1 = p2 - p1;
    const Vector3f line2 = p4 - p3;
    const Vector3f line_start_diff = p1 - p3; // from the beginning of the second line to the beginning of the first line

    // these don't really have a physical representation. They're only here to break up the longer formulas below.
    const float a = line1 * line1;
    const float b = line1 * line2;
    const float c = line2 * line2;
    const float d = line1 * line_start_diff;
    const float e = line2 * line_start_diff;

    // the parameter for the position on line1 and line2 which define the closest points.
    float t1 = 0.0f;
    float t2 = 0.0f;

    // if lines are almost parallel, return a garbage answer. This is irrelevant, since the loop
    // could always be pruned start/end of the previous/subsequent line segment
    if (is_zero((a*c)-(b*b))) {
        return {FLT_MAX, Vector3f(0.0f, 0.0f, 0.0f)};
    }

    t1 = (b * e - c * d) / (a * c - b * b);
    t2 = (a * e - b * d) / (a * c - b * b);

    // restrict both parameters between 0 and 1.
    t1 = constrain_float(t1, 0.0f, 1.0f);
    t2 = constrain_float(t2, 0.0f, 1.0f);

    // difference between two closest points
    const Vector3f dP = line_start_diff + line1 * t1 - line2 * t2;

    const Vector3f midpoint = (p1 + (line1 * t1) + p3 + (line2 * t2)) / 2.0;
    return {dP.length(), midpoint};
}

// de-activate SmartRTL, send warning to GCS and logger
void AP_SmartRTL::deactivate(Action action, const char *reason)
{
    _active = false;
    log_action(action);
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SmartRTL deactivated: %s", reason);
}

#if HAL_LOGGING_ENABLED
// logging
void AP_SmartRTL::log_action(Action action, const Vector3f &point) const
{
    if (!_example_mode) {
        AP::logger().Write_SRTL(_active, _path_points_count, _path_points_max, action, point);
    }
}
#endif

// returns true if the two loops overlap (used within add_loop to determine which loops to keep or throw away)
bool AP_SmartRTL::loops_overlap(const prune_loop_t &loop1, const prune_loop_t &loop2) const
{
    // check if loop1 within loop2
    if (loop1.start_index >= loop2.start_index && loop1.end_index <= loop2.end_index) {
        return true;
    }
    // check if loop2 within loop1
    if (loop2.start_index >= loop1.start_index && loop2.end_index <= loop1.end_index) {
        return true;
    }
    // check for partial overlap (loop1's start OR end point is within loop2)
    const bool loop1_start_in_loop2 = (loop1.start_index >= loop2.start_index) && (loop1.start_index <= loop2.end_index);
    const bool loop1_end_in_loop2 = (loop1.end_index >= loop2.start_index) && (loop1.end_index <= loop2.end_index);
    if (loop1_start_in_loop2 != loop1_end_in_loop2) {
        return true;
    }
    // if we got here, no overlap
    return false;
}

// returns true if pilot's yaw input should be used to adjust vehicle's heading
bool AP_SmartRTL::use_pilot_yaw(void) const
{
    return (_options.get() & uint32_t(Options::IgnorePilotYaw)) == 0;
}

