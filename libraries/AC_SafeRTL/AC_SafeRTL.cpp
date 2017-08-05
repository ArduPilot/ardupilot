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

#include "AC_SafeRTL.h"

/*
*    This library is used for Copter's Safe Return-to-Launch feature. It stores
*    "breadcrumbs" in memory, up to a certain number  of breadcrumbs have been stored,
*    and then cleans up those breadcrumbs when space is filling up. When Safe-RTL is
*    triggered, a more thorough cleanup occurs, and then the resulting path can
*    is fed into WP_NAV to fly it.
*
*    The cleanup consists of two parts, pruning and simplification. Pruning works
*    by calculating the closest distance reached by the line segment between any
*    two pairs of sequential points, and cuts out anything between two points
*    when their line segments get close. This algorithm will never compare two
*    consecutive line segments. Obviously the segments (p1,p2) and (p2,p3) will
*    get very close (they touch), but there would be nothing to trim between them.
*    The simplification step uses the Ramer-Douglas-Peucker algorithm.
*    See Wikipedia for description.
*
*    The simplification and pruning algorithms do not alter the path in memory,
*    and are designed to run in the background. They each have a parameter that
*    decides how long they will run before saving their state and returning. They
*    are both anytime algorithms, which is helpful when memory is filling up and
*    we just need to quickly identify a handful of points which can be deleted.
*    The algorithms can also report if they are done. If not, the thorough cleanup
*    will just have to wait for a bit until they are both done.
*/

SafeRTL_Path::SafeRTL_Path(bool log) :
    _logging_enabled(log),
    _active(false),
    _accepting_new_points(true),
    _prunable_loops_last_index(-1)
{
    _simplification_bitmask = std::bitset<SAFERTL_MAX_PATH_LEN>().set(); //initialize to 0b1111...
    path[0] = {0.0f, 0.0f, 0.0f};
    _time_of_last_good_position = AP_HAL::millis();
}

void SafeRTL_Path::update(bool position_ok, Vector3f current_pos, GCS* gcs)
{
    if (!_active || !_accepting_new_points) {
        return;
    }

    if (position_ok) {
        _time_of_last_good_position = AP_HAL::millis();
    } else {
        if (AP_HAL::millis() - _time_of_last_good_position > SAFERTL_BAD_POSITION_TIME) {
            _active = false;
            if (_logging_enabled) {
                DataFlash_Class::instance()->Log_Write_SRTL(DataFlash_Class::SRTL_DEACTIVATED_BAD_POSITION, {0.0f, 0.0f, 0.0f});
            }
            if (gcs) {
                gcs->send_text(MAV_SEVERITY_WARNING,"SafeRTL Unavailable: Bad Position");
            }
        }
    }

    // it's important to do the cleanup before adding the point, because appending a point will reset the cleanup methods,
    // so there will not be anything to clean up immediately after adding a point.
    // The cleanup usually returns immediately. If it decides to actually perform the cleanup, it takes about 100us.
    if (!_routine_cleanup()) {
        _active = false;
        if (_logging_enabled) {
            DataFlash_Class::instance()->Log_Write_SRTL(DataFlash_Class::SRTL_DEACTIVATED_CLEANUP_FAILED, {0.0f, 0.0f, 0.0f});
        }
        if (gcs) {
            gcs->send_text(MAV_SEVERITY_WARNING,"SafeRTL Unavailable: Path Cleanup Failed");
        }
        return;
    }

    // TODO give up on SafeRTL if the position has been bad for X seconds.

    if (position_ok) { // meters from origin, NED
        // append the new point, if appropriate
        if (HYPOT(current_pos, path[_last_index]) > SAFERTL_POSITION_DELTA) {
            // add the breadcrumb
            path[++_last_index] = current_pos;

            if (_logging_enabled) {
                DataFlash_Class::instance()->Log_Write_SRTL(DataFlash_Class::SRTL_POINT_ADD, current_pos);
            }

            // if cleanup algorithms are finished (And therefore not runnning), reset them
            if (_simplification_complete) {
                _reset_simplification();
            }
            if (_pruning_complete) {
                _reset_pruning();
            }
        }
    }
}

/**
*  Run this method only when preparing to initiate the RTL procedure. Returns a
*  pointer to the cleaned-up path. Returns nullptr if the cleanup algorithms aren't ready yet.
*  If this happens, just run this method again a bit later.
*
*  Probably best not to run this unless cleanup_ready() is returning true
*/
Vector3f* SafeRTL_Path::thorough_cleanup()
{
    // apply simplification
    _zero_points_by_simplification_bitmask();

    // apply pruning
    _zero_points_by_loops(SAFERTL_MAX_PATH_LEN); // prune every single loop

    _remove_empty_points();

    // end by resetting the state of the cleanup methods.
    _reset_simplification();
    _reset_pruning();

    return path;
}

/**
*   Returns true if the list is empty after popping this point.
*/
bool SafeRTL_Path::pop_point(Vector3f& point)
{
    if (_last_index == 0) {
        point = path[0];
        return true;
    }
    point =  path[_last_index--];
    return false;
}

/**
*   Pass nullptr for the gcs argument if you don't want errors being reported to the user.
*/
void SafeRTL_Path::reset_path(bool position_ok, const Vector3f start, GCS* gcs)
{
    _last_index = 0;
    path[_last_index] = start;
    _simplification_complete = false;
    _simplification_clean_until = 0;
    _pruning_complete = false;
    _pruning_clean_until = 0;
    _prunable_loops_last_index = -1;

    if (position_ok) {
        _active = true;
        _time_of_last_good_position = AP_HAL::millis();
    } else {
        _active = false;
        if (_logging_enabled) {
            DataFlash_Class::instance()->Log_Write_SRTL(DataFlash_Class::SRTL_DEACTIVATED_BAD_POSITION, {0.0f, 0.0f, 0.0f});
        }
        if (gcs) {
            gcs->send_text(MAV_SEVERITY_WARNING, "SafeRTL Unavailable: Bad Position");
        }
    }
}

/**
*    Simplifies a 3D path, according to the Ramer-Douglas-Peucker algorithm.
*    Returns the number of items which were removed. end_index is the index of the last element in the path.
*/
void SafeRTL_Path::detect_simplifications()
{
    if (_simplification_complete || !_active || _last_index < 2) {
        return;
    } else if (_simplification_stack.is_empty()) {  // if not complete but also nothing to do, we must be restarting.
        // reset to beginning state
        _simplification_stack.push_back(start_finish {0, _last_index});
    }
    uint32_t start_time = AP_HAL::micros();
    uint32_t start_index, end_index;
    while (!_simplification_stack.is_empty()) {
        if (AP_HAL::micros() - start_time > SAFERTL_SIMPLIFICATION_TIME) {
            return;
        }

        start_finish tmp {}; // initialize to zero to suppress warnings
        _simplification_stack.pop_front(tmp);
        start_index = tmp.start;
        end_index = tmp.finish;

        // if we've already verified that everything before here is clean
        if (tmp.finish <= _simplification_clean_until) {
            continue;
        }

        float max_dist = 0.0f;
        uint32_t index = start_index;
        for (uint32_t i = index + 1; i < end_index; i++) {
            if (_simplification_bitmask[i]) {
                float dist = _point_line_dist(path[i], path[start_index], path[end_index]);
                if (dist > max_dist) {
                    index = i;
                    max_dist = dist;
                }
            }
        }

        if (max_dist > SAFERTL_SIMPLIFICATION_EPSILON) {
            _simplification_stack.push_back(start_finish{start_index, index});
            _simplification_stack.push_back(start_finish{index, end_index});
        } else {
            for (uint32_t i = start_index + 1; i < end_index; i++) {
                _simplification_bitmask[i] = false;
            }
        }
    }
    _simplification_complete = true;
}

/**
*   This method runs for the allotted time, and detects loops in a path. All detected loops are added to _prunable_loops,
*   this function does not alter the path in memory. It works by comparing the line segment between any two sequential points
*   to the line segment between any other two sequential points. If they get close enough, anything between them could be pruned.
*
*   Note that this method might take a bit longer than LOOP_TIME. It only stops after it's already run longer.
*/
void SafeRTL_Path::detect_loops()
{
    if (!_active || _pruning_complete || _prunable_loops_last_index >= SAFERTL_LOOP_BUFFER_LEN - 1 || _last_index < 3) {
        return;
    }
    uint32_t start_time = AP_HAL::micros();

    while (_pruning_current_i < _last_index - 1) {
        // if this method has run for long enough, exit
        if (AP_HAL::micros() - start_time > SAFERTL_LOOP_TIME) {
            return;
        }

        // this check prevents detection of a loop-within-a-loop
        uint32_t j = MAX(_pruning_current_i + 2, _pruning_min_j);
        while (j < _last_index) {
            dist_point dp = _segment_segment_dist(path[_pruning_current_i], path[_pruning_current_i+1], path[j], path[j+1]);
            if (dp.distance <= SAFERTL_PRUNING_DELTA) {
                _pruning_min_j = j;
                // int promotion rules disallow using i+1 and j+1
                _prunable_loops[++_prunable_loops_last_index] = {(++_pruning_current_i)--,(++j)--,dp.point};
            }
            j++;
        }
        _pruning_current_i++;
    }
    _pruning_complete = true;
}

//
// Private methods
//

/**
*   Run this regularly, in the main loop (don't worry - it runs quickly, 100us). If no cleanup is needed, it will immediately return.
*   Otherwise, it will run a cleanup, based on info computed by the background methods, detect_simplifications() and detect_loops().
*   If no cleanup is possible, this method returns false. This should be treated as an error condition.
*/
bool SafeRTL_Path::_routine_cleanup()
{
    // We only do a routine cleanup if the memory is almost full. Cleanup deletes
    // points which are potentially useful, so it would be bad to clean up if we don't have to
    if (_last_index < SAFERTL_MAX_PATH_LEN - 10) {
        return true;
    }

    uint32_t potential_amount_to_simplify = _simplification_bitmask.size() - _simplification_bitmask.count();

    // if simplifying will remove more than 10 points, just do it
    if (potential_amount_to_simplify >= 10) {
        _zero_points_by_simplification_bitmask();
        _remove_empty_points();
        // end by resetting the state of the cleanup methods.
        _reset_simplification();
        _reset_pruning();
        return true;
    }

    uint32_t potential_amount_to_prune = 0;
    for (int32_t i = 0; i <= _prunable_loops_last_index; i++) {
        // add 1 at the end, because a pruned loop is always replaced by one new point.
        potential_amount_to_prune += _prunable_loops[i].end_index - _prunable_loops[i].start_index + 1;
    }

    // if pruning could remove 10+ points, prune loops until 10 or more points have been removed (doesn't necessarily prune all loops)
    if (potential_amount_to_prune >= 10) {
        _zero_points_by_loops(10);
        _remove_empty_points();
        // end by resetting the state of the cleanup methods.
        _reset_simplification();
        _reset_pruning();
        return true;
    }

    // as a last resort, see if pruning and simplifying together would remove 10+ points.
    if (potential_amount_to_prune + potential_amount_to_simplify >= 10) {
        _zero_points_by_simplification_bitmask();
        _zero_points_by_loops(10);
        _remove_empty_points();
        // end by resetting the state of the cleanup methods.
        _reset_simplification();
        _reset_pruning();
        return true;
    }
    return false;
}

void SafeRTL_Path::_reset_simplification()
{
    _simplification_complete = false;
    _simplification_stack.clear();
    _simplification_bitmask.set();
}

void SafeRTL_Path::_reset_pruning()
{
    _pruning_complete = false;
    _pruning_current_i = _pruning_clean_until;
    _pruning_min_j = _pruning_clean_until+2;
    _prunable_loops_last_index = -1; // clear the loops that we've recorded
}

void SafeRTL_Path::_zero_points_by_simplification_bitmask()
{
    for (uint32_t i = 0; i <= _last_index; i++) {
        if (!_simplification_bitmask[i]) {
            _simplification_clean_until = MIN(_simplification_clean_until, i-1);
            if (path[i] != Vector3f(0.0f, 0.0f, 0.0f)) {
                if (_logging_enabled) {
                    DataFlash_Class::instance()->Log_Write_SRTL(DataFlash_Class::SRTL_POINT_SIMPLIFY, path[i]);
                }
                path[i] = Vector3f(0.0f, 0.0f, 0.0f);
            }
        }
    }
}

/**
*   Only prunes loops until $points_to_delete points have been removed. It does not necessarily prune all loops.
*/
void SafeRTL_Path::_zero_points_by_loops(uint32_t points_to_delete)
{
    int removed_points = 0;
    for (int32_t i = 0; i <= _prunable_loops_last_index; i++) {
        loop l = _prunable_loops[i];
        _pruning_clean_until = MIN(_pruning_clean_until, l.start_index-1);
        for (uint32_t j = l.start_index; j < l.end_index; j++) {
            // zero this point if it wasn't already zeroed
            if (path[j] != Vector3f(0.0f, 0.0f, 0.0f)) {
                if (_logging_enabled) {
                    DataFlash_Class::instance()->Log_Write_SRTL(DataFlash_Class::SRTL_POINT_PRUNE, path[j]);
                }
                path[j] = Vector3f(0.0f, 0.0f, 0.0f);
            }
        }
        path[int((l.start_index+l.end_index)/2.0)] = l.halfway_point;
        removed_points += l.end_index - l.start_index - 1;
        if (removed_points > points_to_delete) {
            return;
        }
    }
}

/**
*  Removes all NULL points from the path, and shifts remaining items to correct position.
*  The first item will not be removed.
*/
void SafeRTL_Path::_remove_empty_points()
{
    uint32_t i = 0;
    uint32_t j = 0;
    uint32_t removed = 0;
    while (++i <= _last_index) { // never removes the first item. This should always be {0,0,0}
        if (path[i] != Vector3f(0.0f, 0.0f, 0.0f)) {
            path[++j] = path[i];
        } else {
            removed++;
        }
    }
    _last_index -= removed;
}

/**
*  Returns the closest distance in 3D space between any part of two input segments, defined from p1 to p2 and from p3 to p4.
*  Also returns the point which is halfway between
*
*  Limitation: This function does not work for parallel lines. In this case, it will return FLT_MAX. This does not matter for the path cleanup algorithm because
*  the pruning will still occur fine between the first parallel segment and a segment which is directly before or after the second segment.
*/
SafeRTL_Path::dist_point SafeRTL_Path::_segment_segment_dist(const Vector3f &p1, const Vector3f &p2, const Vector3f &p3, const Vector3f &p4)
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

        Vector3f halfway_point = (p1+line1*t1 + p3+line2*t2)/2.0f;
        return {dP.length(), halfway_point};
    }
}


/**
*  Returns the closest distance from a point to a 3D line. The line is defined by any 2 points
*/
float SafeRTL_Path::_point_line_dist(const Vector3f &point, const Vector3f &line1, const Vector3f &line2)
{
    // triangle side lengths
    float a = HYPOT(point, line1);
    float b = HYPOT(line1, line2);
    float c = HYPOT(line2, point);

    // semiperimeter of triangle
    float s = (a+b+c)/2.0f;

    float area_squared = s*(s-a)*(s-b)*(s-c);
    // must be constrained above 0 because a triangle where all 3 points could be on a line. float rounding could push this under 0.
    if (area_squared < 0.0f) {
        area_squared = 0.0f;
    }
    float area = sqrt(area_squared);
    return 2.0f*area/b;
}
