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

#include "AP_OADijkstra.h"
#include "AP_OAPathPlanner.h"

#include <AC_Fence/AC_Fence.h>

#if AP_FENCE_ENABLED

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

#define OA_DIJKSTRA_EXPANDING_ARRAY_ELEMENTS_PER_CHUNK  32      // expanding arrays for fence points and paths to destination will grow in increments of 20 elements
#define OA_DIJKSTRA_POLYGON_SHORTPATH_NOTSET_IDX        255     // index use to indicate we do not have a tentative short path for a node
#define OA_DIJKSTRA_ERROR_REPORTING_INTERVAL_MS         5000    // failure messages sent to GCS every 5 seconds

/// Constructor
AP_OADijkstra::AP_OADijkstra(AP_Int16 &options) :
        _options(options),
        _inclusion_polygon_pts(OA_DIJKSTRA_EXPANDING_ARRAY_ELEMENTS_PER_CHUNK),
        _exclusion_polygon_pts(OA_DIJKSTRA_EXPANDING_ARRAY_ELEMENTS_PER_CHUNK),
        _exclusion_circle_pts(OA_DIJKSTRA_EXPANDING_ARRAY_ELEMENTS_PER_CHUNK),
        _short_path_data(OA_DIJKSTRA_EXPANDING_ARRAY_ELEMENTS_PER_CHUNK),
        _path(OA_DIJKSTRA_EXPANDING_ARRAY_ELEMENTS_PER_CHUNK)
{
}

// calculate a destination to avoid fences
// returns DIJKSTRA_STATE_SUCCESS and populates origin_new and destination_new if avoidance is required
AP_OADijkstra::AP_OADijkstra_State AP_OADijkstra::update(const Location &current_loc, const Location &destination, Location& origin_new, Location& destination_new)
{
    WITH_SEMAPHORE(AP::fence()->polyfence().get_loaded_fence_semaphore());

    // avoidance is not required if no fences
    if (!some_fences_enabled()) {
        Write_OADijkstra(DIJKSTRA_STATE_NOT_REQUIRED, 0, 0, 0, destination, destination);
        return DIJKSTRA_STATE_NOT_REQUIRED;
    }

    // no avoidance required if destination is same as current location
    if (current_loc.same_latlon_as(destination)) {
        Write_OADijkstra(DIJKSTRA_STATE_NOT_REQUIRED, 0, 0, 0, destination, destination);
        return DIJKSTRA_STATE_NOT_REQUIRED;
    }

    // check for inclusion polygon updates
    if (check_inclusion_polygon_updated()) {
        _inclusion_polygon_with_margin_ok = false;
        _polyfence_visgraph_ok = false;
        _shortest_path_ok = false;
    }

    // check for exclusion polygon updates
    if (check_exclusion_polygon_updated()) {
        _exclusion_polygon_with_margin_ok = false;
        _polyfence_visgraph_ok = false;
        _shortest_path_ok = false;
    }

    // check for exclusion circle updates
    if (check_exclusion_circle_updated()) {
        _exclusion_circle_with_margin_ok = false;
        _polyfence_visgraph_ok = false;
        _shortest_path_ok = false;
    }

    // create inner polygon fence
    AP_OADijkstra_Error error_id;
    if (!_inclusion_polygon_with_margin_ok) {
        _inclusion_polygon_with_margin_ok = create_inclusion_polygon_with_margin(_polyfence_margin * 100.0f, error_id);
        if (!_inclusion_polygon_with_margin_ok) {
            report_error(error_id);
            Write_OADijkstra(DIJKSTRA_STATE_ERROR, (uint8_t)error_id, 0, 0, destination, destination);
            return DIJKSTRA_STATE_ERROR;
        }
    }

    // create exclusion polygon outer fence
    if (!_exclusion_polygon_with_margin_ok) {
        _exclusion_polygon_with_margin_ok = create_exclusion_polygon_with_margin(_polyfence_margin * 100.0f, error_id);
        if (!_exclusion_polygon_with_margin_ok) {
            report_error(error_id);
            Write_OADijkstra(DIJKSTRA_STATE_ERROR, (uint8_t)error_id, 0, 0, destination, destination);
            return DIJKSTRA_STATE_ERROR;
        }
    }

    // create exclusion circle points
    if (!_exclusion_circle_with_margin_ok) {
        _exclusion_circle_with_margin_ok = create_exclusion_circle_with_margin(_polyfence_margin * 100.0f, error_id);
        if (!_exclusion_circle_with_margin_ok) {
            report_error(error_id);
            Write_OADijkstra(DIJKSTRA_STATE_ERROR, (uint8_t)error_id, 0, 0, destination, destination);
            return DIJKSTRA_STATE_ERROR;
        }
    }

    // create visgraph for all fence (with margin) points
    if (!_polyfence_visgraph_ok) {
        _polyfence_visgraph_ok = create_fence_visgraph(error_id);
        if (!_polyfence_visgraph_ok) {
            _shortest_path_ok = false;
            report_error(error_id);
            Write_OADijkstra(DIJKSTRA_STATE_ERROR, (uint8_t)error_id, 0, 0, destination, destination);
            return DIJKSTRA_STATE_ERROR;
        }
        // reset logging count to restart logging updated graph
        _log_num_points = 0;
        _log_visgraph_version++;
    }

    // Log one visgraph point per loop
    if (_polyfence_visgraph_ok && (_log_num_points < total_numpoints()) && (_options & AP_OAPathPlanner::OARecoveryOptions::OA_OPTION_LOG_DIJKSTRA_POINTS) ) {
        Vector2f vis_point;
        if (get_point(_log_num_points, vis_point)) {
            Location log_location(Vector3f{vis_point.x, vis_point.y, 0.0}, Location::AltFrame::ABOVE_ORIGIN);
            Write_Visgraph_point(_log_visgraph_version, _log_num_points, log_location.lat, log_location.lng);
            _log_num_points++;
        }
    }

    // rebuild path if destination has changed
    if (!destination.same_latlon_as(_destination_prev)) {
        _destination_prev = destination;
        _shortest_path_ok = false;
    }

    // calculate shortest path from current_loc to destination
    if (!_shortest_path_ok) {
        _shortest_path_ok = calc_shortest_path(current_loc, destination, error_id);
        if (!_shortest_path_ok) {
            report_error(error_id);
            Write_OADijkstra(DIJKSTRA_STATE_ERROR, (uint8_t)error_id, 0, 0, destination, destination);
            return DIJKSTRA_STATE_ERROR;
        }
        // start from 2nd point on path (first is the original origin)
        _path_idx_returned = 1;
    }

    // path has been created, return latest point
    Vector2f dest_pos;
    if (get_shortest_path_point(_path_idx_returned, dest_pos)) {

        // for the first point return origin as current_loc
        Vector2f origin_pos;
        if ((_path_idx_returned > 0) && get_shortest_path_point(_path_idx_returned-1, origin_pos)) {
            // convert offset from ekf origin to Location
            Location temp_loc(Vector3f{origin_pos.x, origin_pos.y, 0.0}, Location::AltFrame::ABOVE_ORIGIN);
            origin_new = temp_loc;
        } else {
            // for first point use current loc as origin
            origin_new = current_loc;
        }

        // convert offset from ekf origin to Location
        Location temp_loc(Vector3f{dest_pos.x, dest_pos.y, 0.0}, Location::AltFrame::ABOVE_ORIGIN);
        destination_new = destination;
        destination_new.lat = temp_loc.lat;
        destination_new.lng = temp_loc.lng;

        // check if we should advance to next point for next iteration
        const bool near_oa_wp = current_loc.get_distance(destination_new) <= 2.0f;
        const bool past_oa_wp = current_loc.past_interval_finish_line(origin_new, destination_new);
        if (near_oa_wp || past_oa_wp) {
            _path_idx_returned++;
        }
        // log success
        Write_OADijkstra(DIJKSTRA_STATE_SUCCESS, 0, _path_idx_returned, _path_numpoints, destination, destination_new);
        return DIJKSTRA_STATE_SUCCESS;
    }

    // we have reached the destination so avoidance is no longer required
    Write_OADijkstra(DIJKSTRA_STATE_NOT_REQUIRED, 0, 0, 0, destination, destination);
    return DIJKSTRA_STATE_NOT_REQUIRED;
}

// returns true if at least one inclusion or exclusion zone is enabled
bool AP_OADijkstra::some_fences_enabled() const
{
    const AC_Fence *fence = AC_Fence::get_singleton();
    if (fence == nullptr) {
        return false;
    }
    if ((fence->polyfence().get_inclusion_polygon_count() == 0) &&
        (fence->polyfence().get_exclusion_polygon_count() == 0) &&
        (fence->polyfence().get_exclusion_circle_count() == 0)) {
        return false;
    }
    return ((fence->get_enabled_fences() & AC_FENCE_TYPE_POLYGON) > 0);
}

// return error message for a given error id
const char* AP_OADijkstra::get_error_msg(AP_OADijkstra_Error error_id) const
{
    switch (error_id) {
    case AP_OADijkstra_Error::DIJKSTRA_ERROR_NONE:
        return "no error";
        break;
    case AP_OADijkstra_Error::DIJKSTRA_ERROR_OUT_OF_MEMORY:
        return "out of memory";
        break;
    case AP_OADijkstra_Error::DIJKSTRA_ERROR_OVERLAPPING_POLYGON_POINTS:
        return "overlapping polygon points";
        break;
    case AP_OADijkstra_Error::DIJKSTRA_ERROR_FAILED_TO_BUILD_INNER_POLYGON:
        return "failed to build inner polygon";
        break;
    case AP_OADijkstra_Error::DIJKSTRA_ERROR_OVERLAPPING_POLYGON_LINES:
        return "overlapping polygon lines";
        break;
    case AP_OADijkstra_Error::DIJKSTRA_ERROR_FENCE_DISABLED:
        return "fence disabled";
        break;
    case AP_OADijkstra_Error::DIJKSTRA_ERROR_TOO_MANY_FENCE_POINTS:
        return "too many fence points";
        break;
    case AP_OADijkstra_Error::DIJKSTRA_ERROR_NO_POSITION_ESTIMATE:
        return "no position estimate";
        break;
    case AP_OADijkstra_Error::DIJKSTRA_ERROR_COULD_NOT_FIND_PATH:
        return "could not find path";
        break;
    }

    // we should never reach here but just in case
    return "unknown error";
}

void AP_OADijkstra::report_error(AP_OADijkstra_Error error_id)
{
    // report errors to GCS every 5 seconds
    uint32_t now_ms = AP_HAL::millis();
    if ((error_id != AP_OADijkstra_Error::DIJKSTRA_ERROR_NONE) &&
        ((error_id != _error_last_id) || ((now_ms - _error_last_report_ms) > OA_DIJKSTRA_ERROR_REPORTING_INTERVAL_MS))) {
        const char* error_msg = get_error_msg(error_id);
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Dijkstra: %s", error_msg);
        _error_last_id = error_id;
        _error_last_report_ms = now_ms;
    }
}

// check if polygon fence has been updated since we created the inner fence. returns true if changed
bool AP_OADijkstra::check_inclusion_polygon_updated() const
{
    // exit immediately if polygon fence is not enabled
    const AC_Fence *fence = AC_Fence::get_singleton();
    if (fence == nullptr) {
        return false;
    }
    return (_inclusion_polygon_update_ms != fence->polyfence().get_inclusion_polygon_update_ms());
}

// create polygons inside the existing inclusion polygons
// returns true on success.  returns false on failure and err_id is updated
bool AP_OADijkstra::create_inclusion_polygon_with_margin(float margin_cm, AP_OADijkstra_Error &err_id)
{
    const AC_Fence *fence = AC_Fence::get_singleton();

    if (fence == nullptr) {
        err_id = AP_OADijkstra_Error::DIJKSTRA_ERROR_FENCE_DISABLED;
        return false;
    }

    // skip unnecessary retry to build inclusion polygon if previous fence points have not changed 
    if (_inclusion_polygon_update_ms == fence->polyfence().get_inclusion_polygon_update_ms()) {
        return false;
    }

    _inclusion_polygon_update_ms = fence->polyfence().get_inclusion_polygon_update_ms();

    // clear all points
    _inclusion_polygon_numpoints = 0;

    // return immediately if no polygons
    const uint8_t num_inclusion_polygons = fence->polyfence().get_inclusion_polygon_count();

    // iterate through polygons and create inner points
    for (uint8_t i = 0; i < num_inclusion_polygons; i++) {
        uint16_t num_points;
        const Vector2f* boundary = fence->polyfence().get_inclusion_polygon(i, num_points);

        // for each point in inclusion polygon
        // Note: boundary is "unclosed" meaning the last point is *not* the same as the first
        uint16_t new_points = 0;
        for (uint16_t j = 0; j < num_points; j++) {

            // find points before and after current point (relative to current point)
            const uint16_t before_idx = (j == 0) ? num_points-1 : j-1;
            const uint16_t after_idx = (j == num_points-1) ? 0 : j+1;
            Vector2f before_pt = boundary[before_idx] - boundary[j];
            Vector2f after_pt = boundary[after_idx] - boundary[j];

            // if points are overlapping fail
            if (before_pt.is_zero() || after_pt.is_zero() || (before_pt == after_pt)) {
                err_id = AP_OADijkstra_Error::DIJKSTRA_ERROR_OVERLAPPING_POLYGON_POINTS;
                return false;
            }

            // scale points to be unit vectors
            before_pt.normalize();
            after_pt.normalize();

            // calculate intermediate point and scale to margin
            Vector2f intermediate_pt = after_pt + before_pt;
            intermediate_pt.normalize();
            intermediate_pt *= margin_cm;

            // find final point which is outside the inside polygon
            Vector2f temp_point = boundary[j] + intermediate_pt;
            if (Polygon_outside(temp_point, boundary, num_points)) {
                intermediate_pt *= -1.0;
                temp_point = boundary[j] + intermediate_pt;
                if (Polygon_outside(temp_point, boundary, num_points)) {
                    // could not find a point on either side that was outside the exclusion polygon so fail
                    // this may happen if the exclusion polygon has overlapping lines
                    err_id = AP_OADijkstra_Error::DIJKSTRA_ERROR_OVERLAPPING_POLYGON_LINES;
                    return false;
                }
            }

            // don't add points in corners
            if (fabsf(intermediate_pt.angle() - before_pt.angle()) < M_PI_2) {
                continue;
            }

            // expand array if required
            if (!_inclusion_polygon_pts.expand_to_hold(_inclusion_polygon_numpoints + new_points + 1)) {
                err_id = AP_OADijkstra_Error::DIJKSTRA_ERROR_OUT_OF_MEMORY;
                return false;
            }
            // add point
            _inclusion_polygon_pts[_inclusion_polygon_numpoints + new_points] = temp_point;
            new_points++;
        }

        // update total number of points
        _inclusion_polygon_numpoints += new_points;
    }
    return true;
}

// check if exclusion polygons have been updated since create_exclusion_polygon_with_margin was run
// returns true if changed
bool AP_OADijkstra::check_exclusion_polygon_updated() const
{
    const AC_Fence *fence = AC_Fence::get_singleton();
    if (fence == nullptr) {
        return false;
    }
    return (_exclusion_polygon_update_ms != fence->polyfence().get_exclusion_polygon_update_ms());
}

// create polygons around existing exclusion polygons
// returns true on success.  returns false on failure and err_id is updated
bool AP_OADijkstra::create_exclusion_polygon_with_margin(float margin_cm, AP_OADijkstra_Error &err_id)
{
    const AC_Fence *fence = AC_Fence::get_singleton();

    if (fence == nullptr) {
        err_id = AP_OADijkstra_Error::DIJKSTRA_ERROR_FENCE_DISABLED;
        return false;
    }

    // skip unnecessary retry to build exclusion polygon if previous fence points have not changed 
    if (_exclusion_polygon_update_ms == fence->polyfence().get_exclusion_polygon_update_ms()) {
        return false;
    }

    _exclusion_polygon_update_ms = fence->polyfence().get_exclusion_polygon_update_ms();


    // clear all points
    _exclusion_polygon_numpoints = 0;

    // return immediately if no exclusion polygons
    const uint8_t num_exclusion_polygons = fence->polyfence().get_exclusion_polygon_count();

    // iterate through exclusion polygons and create outer points
    for (uint8_t i = 0; i < num_exclusion_polygons; i++) {
        uint16_t num_points;
        const Vector2f* boundary = fence->polyfence().get_exclusion_polygon(i, num_points);
   
        // for each point in exclusion polygon
        // Note: boundary is "unclosed" meaning the last point is *not* the same as the first
        uint16_t new_points = 0;
        for (uint16_t j = 0; j < num_points; j++) {

            // find points before and after current point (relative to current point)
            const uint16_t before_idx = (j == 0) ? num_points-1 : j-1;
            const uint16_t after_idx = (j == num_points-1) ? 0 : j+1;
            Vector2f before_pt = boundary[before_idx] - boundary[j];
            Vector2f after_pt = boundary[after_idx] - boundary[j];

            // if points are overlapping fail
            if (before_pt.is_zero() || after_pt.is_zero() || (before_pt == after_pt)) {
                err_id = AP_OADijkstra_Error::DIJKSTRA_ERROR_OVERLAPPING_POLYGON_POINTS;
                return false;
            }

            // scale points to be unit vectors
            before_pt.normalize();
            after_pt.normalize();

            // calculate intermediate point and scale to margin
            Vector2f intermediate_pt = after_pt + before_pt;
            intermediate_pt.normalize();
            intermediate_pt *= margin_cm;

            // find final point which is outside the original polygon
            Vector2f temp_point = boundary[j] + intermediate_pt;
            if (!Polygon_outside(temp_point, boundary, num_points)) {
                intermediate_pt *= -1;
                temp_point = boundary[j] + intermediate_pt;
                if (!Polygon_outside(temp_point, boundary, num_points)) {
                    // could not find a point on either side that was outside the exclusion polygon so fail
                    // this may happen if the exclusion polygon has overlapping lines
                    err_id = AP_OADijkstra_Error::DIJKSTRA_ERROR_OVERLAPPING_POLYGON_LINES;
                    return false;
                }
            }

            // don't add points in corners
            if (fabsf(intermediate_pt.angle() - before_pt.angle()) < M_PI_2) {
                continue;
            }

            // expand array if required
            if (!_exclusion_polygon_pts.expand_to_hold(_exclusion_polygon_numpoints + new_points + 1)) {
                err_id = AP_OADijkstra_Error::DIJKSTRA_ERROR_OUT_OF_MEMORY;
                return false;
            }
            // add point
            _exclusion_polygon_pts[_exclusion_polygon_numpoints + new_points] = temp_point;
            new_points++;
        }

        // update total number of points
        _exclusion_polygon_numpoints += new_points;
    }
    return true;
}

// check if exclusion circles have been updated since create_exclusion_circle_with_margin was run
// returns true if changed
bool AP_OADijkstra::check_exclusion_circle_updated() const
{
    // exit immediately if fence is not enabled
    const AC_Fence *fence = AC_Fence::get_singleton();
    if (fence == nullptr) {
        return false;
    }
    return (_exclusion_circle_update_ms != fence->polyfence().get_exclusion_circle_update_ms());
}

// create polygons around existing exclusion circles
// returns true on success.  returns false on failure and err_id is updated
bool AP_OADijkstra::create_exclusion_circle_with_margin(float margin_cm, AP_OADijkstra_Error &err_id)
{
    // exit immediately if fence is not enabled
    const AC_Fence *fence = AC_Fence::get_singleton();
    if (fence == nullptr) {
        err_id = AP_OADijkstra_Error::DIJKSTRA_ERROR_FENCE_DISABLED;
        return false;
    }

    // clear all points
    _exclusion_circle_numpoints = 0;

    // unit length offsets for polygon points around circles
    const Vector2f unit_offsets[] = {
            {cosf(radians(30)), cosf(radians(30-90))},  // north-east
            {cosf(radians(90)), cosf(radians(90-90))},  // east
            {cosf(radians(150)), cosf(radians(150-90))},// south-east
            {cosf(radians(210)), cosf(radians(210-90))},// south-west
            {cosf(radians(270)), cosf(radians(270-90))},// west
            {cosf(radians(330)), cosf(radians(330-90))},// north-west
    };
    const uint8_t num_points_per_circle = ARRAY_SIZE(unit_offsets);

    // expand polygon point array if required
    const uint8_t num_exclusion_circles = fence->polyfence().get_exclusion_circle_count();
    if (!_exclusion_circle_pts.expand_to_hold(num_exclusion_circles * num_points_per_circle)) {
        err_id = AP_OADijkstra_Error::DIJKSTRA_ERROR_OUT_OF_MEMORY;
        return false;
    }

    // iterate through exclusion circles and create outer polygon points
    for (uint8_t i = 0; i < num_exclusion_circles; i++) {
        Vector2f circle_pos_cm;
        float radius;
        if (fence->polyfence().get_exclusion_circle(i, circle_pos_cm, radius)) {
            // scaler to ensure lines between points do not intersect circle
            const float scaler = (1.0f / cosf(radians(180.0f / (float)num_points_per_circle))) * ((radius * 100.0f) + margin_cm);

            // add points to array
            for (uint8_t j = 0; j < num_points_per_circle; j++) {
                _exclusion_circle_pts[_exclusion_circle_numpoints] = circle_pos_cm + (unit_offsets[j] * scaler);
                _exclusion_circle_numpoints++;
            }
        }
    }

    // record fence update time so we don't process these exclusion circles again
    _exclusion_circle_update_ms = fence->polyfence().get_exclusion_circle_update_ms();

    return true;
}

// returns total number of points across all fence types
uint16_t AP_OADijkstra::total_numpoints() const
{
    return _inclusion_polygon_numpoints + _exclusion_polygon_numpoints + _exclusion_circle_numpoints;
}

// get a single point across the total list of points from all fence types
bool AP_OADijkstra::get_point(uint16_t index, Vector2f &point) const
{
    // sanity check index
    if (index >= total_numpoints()) {
        return false;
    }

    // return an inclusion polygon point
    if (index < _inclusion_polygon_numpoints) {
        point = _inclusion_polygon_pts[index];
        return true;
    }
    index -= _inclusion_polygon_numpoints;

    // return an exclusion polygon point
    if (index < _exclusion_polygon_numpoints) {
        point = _exclusion_polygon_pts[index];
        return true;
    }
    index -= _exclusion_polygon_numpoints;

    // return an exclusion circle point
    if (index < _exclusion_circle_numpoints) {
        point = _exclusion_circle_pts[index];
        return true;
    }

    // we should never get here but just in case
    return false;
}

// returns true if line segment intersects polygon or circular fence
bool AP_OADijkstra::intersects_fence(const Vector2f &seg_start, const Vector2f &seg_end) const
{
    // return immediately if fence is not enabled
    const AC_Fence *fence = AC_Fence::get_singleton();
    if (fence == nullptr) {
        return false;
    }

    // determine if segment crosses any of the inclusion polygons
    uint16_t num_points = 0;
    for (uint8_t i = 0; i < fence->polyfence().get_inclusion_polygon_count(); i++) {
        const Vector2f* boundary = fence->polyfence().get_inclusion_polygon(i, num_points);
        if (boundary != nullptr) {
            Vector2f intersection;
            if (Polygon_intersects(boundary, num_points, seg_start, seg_end, intersection)) {
                return true;
            }
        }
    }

    // determine if segment crosses any of the exclusion polygons
    for (uint8_t i = 0; i < fence->polyfence().get_exclusion_polygon_count(); i++) {
        const Vector2f* boundary = fence->polyfence().get_exclusion_polygon(i, num_points);
        if (boundary != nullptr) {
            Vector2f intersection;
            if (Polygon_intersects(boundary, num_points, seg_start, seg_end, intersection)) {
                return true;
            }
        }
    }

    // determine if segment crosses any of the inclusion circles
    for (uint8_t i = 0; i < fence->polyfence().get_inclusion_circle_count(); i++) {
        Vector2f center_pos_cm;
        float radius;
        if (fence->polyfence().get_inclusion_circle(i, center_pos_cm, radius)) {
            // intersects circle if either start or end is further from the center than the radius
            const float radius_cm_sq = sq(radius * 100.0f) ;
            if ((seg_start - center_pos_cm).length_squared() > radius_cm_sq) {
                return true;
            }
            if ((seg_end - center_pos_cm).length_squared() > radius_cm_sq) {
                return true;
            }
        }
    }

    // determine if segment crosses any of the exclusion circles
    for (uint8_t i = 0; i < fence->polyfence().get_exclusion_circle_count(); i++) {
        Vector2f center_pos_cm;
        float radius;
        if (fence->polyfence().get_exclusion_circle(i, center_pos_cm, radius)) {
            // calculate distance between circle's center and segment
            const float dist_cm = Vector2f::closest_distance_between_line_and_point(seg_start, seg_end, center_pos_cm);

            // intersects if distance is less than radius
            if (dist_cm <= (radius * 100.0f)) {
                return true;
            }
        }
    }

    // if we got this far then no intersection
    return false;
}

// create visibility graph for all fence (with margin) points
// returns true on success.  returns false on failure and err_id is updated
// requires these functions to have been run create_inclusion_polygon_with_margin, create_exclusion_polygon_with_margin, create_exclusion_circle_with_margin
bool AP_OADijkstra::create_fence_visgraph(AP_OADijkstra_Error &err_id)
{
    // exit immediately if fence is not enabled
    const AC_Fence *fence = AC_Fence::get_singleton();
    if (fence == nullptr) {
        err_id = AP_OADijkstra_Error::DIJKSTRA_ERROR_FENCE_DISABLED;
        return false;
    }

    // fail if more fence points than algorithm can handle
    if (total_numpoints() >= OA_DIJKSTRA_POLYGON_SHORTPATH_NOTSET_IDX) {
        err_id = AP_OADijkstra_Error::DIJKSTRA_ERROR_TOO_MANY_FENCE_POINTS;
        return false;
    }

    // clear fence points visibility graph
    _fence_visgraph.clear();

    // calculate distance from each point to all other points
    for (uint8_t i = 0; i < total_numpoints() - 1; i++) {
        Vector2f start_seg;
        if (get_point(i, start_seg)) {
            for (uint8_t j = i + 1; j < total_numpoints(); j++) {
                Vector2f end_seg;
                if (get_point(j, end_seg)) {
                    // if line segment does not intersect with any inclusion or exclusion zones add to visgraph
                    if (!intersects_fence(start_seg, end_seg)) {
                        if (!_fence_visgraph.add_item({AP_OAVisGraph::OATYPE_INTERMEDIATE_POINT, i},
                                                      {AP_OAVisGraph::OATYPE_INTERMEDIATE_POINT, j},
                                                      (start_seg - end_seg).length())) {
                            // failure to add a point can only be caused by out-of-memory
                            err_id = AP_OADijkstra_Error::DIJKSTRA_ERROR_OUT_OF_MEMORY;
                            return false;
                        }
                    }
                }
            }
        }
    }

    return true;
}

// updates visibility graph for a given position which is an offset (in cm) from the ekf origin
// to add an additional position (i.e. the destination) set add_extra_position = true and provide the position in the extra_position argument
// requires create_inclusion_polygon_with_margin to have been run
// returns true on success
bool AP_OADijkstra::update_visgraph(AP_OAVisGraph& visgraph, const AP_OAVisGraph::OAItemID& oaid, const Vector2f &position, bool add_extra_position, Vector2f extra_position)
{
    // clear visibility graph
    visgraph.clear();

    // calculate distance from position to all inclusion/exclusion fence points
    for (uint8_t i = 0; i < total_numpoints(); i++) {
        Vector2f seg_end;
        if (get_point(i, seg_end)) {
            if (!intersects_fence(position, seg_end)) {
                // line segment does not intersect with fences so add to visgraph
                if (!visgraph.add_item(oaid, {AP_OAVisGraph::OATYPE_INTERMEDIATE_POINT, i}, (position - seg_end).length())) {
                    return false;
                }
            }
        }
    }

    // add extra point to visibility graph if it doesn't intersect with polygon fence or exclusion polygons
    if (add_extra_position) {
        if (!intersects_fence(position, extra_position)) {
            if (!visgraph.add_item(oaid, {AP_OAVisGraph::OATYPE_DESTINATION, 0}, (position - extra_position).length())) {
                return false;
            }
        }
    }

    return true;
}

// update total distance for all nodes visible from current node
// curr_node_idx is an index into the _short_path_data array
void AP_OADijkstra::update_visible_node_distances(node_index curr_node_idx)
{
    // sanity check
    if (curr_node_idx >= _short_path_data_numpoints) {
        return;
    }

    // get current node for convenience
    const ShortPathNode &curr_node = _short_path_data[curr_node_idx];

    // for each visibility graph
    const AP_OAVisGraph* visgraphs[] = {&_fence_visgraph, &_destination_visgraph};
    for (uint8_t v=0; v<ARRAY_SIZE(visgraphs); v++) {

        // skip if empty
        const AP_OAVisGraph &curr_visgraph = *visgraphs[v];
        if (curr_visgraph.num_items() == 0) {
            continue;
        }

        // search visibility graph for items visible from current_node
        for (uint16_t i = 0; i < curr_visgraph.num_items(); i++) {
            const AP_OAVisGraph::VisGraphItem &item = curr_visgraph[i];
            // match if current node's id matches either of the id's in the graph (i.e. either end of the vector)
            if ((curr_node.id == item.id1) || (curr_node.id == item.id2)) {
                AP_OAVisGraph::OAItemID matching_id = (curr_node.id == item.id1) ? item.id2 : item.id1;
                // find item's id in node array
                node_index item_node_idx;
                if (find_node_from_id(matching_id, item_node_idx)) {
                    // if current node's distance + distance to item is less than item's current distance, update item's distance
                    const float dist_to_item_via_current_node = _short_path_data[curr_node_idx].distance_cm + item.distance_cm;
                    if (dist_to_item_via_current_node < _short_path_data[item_node_idx].distance_cm) {
                        // update item's distance and set "distance_from_idx" to current node's index
                        _short_path_data[item_node_idx].distance_cm = dist_to_item_via_current_node;
                        _short_path_data[item_node_idx].distance_from_idx = curr_node_idx;
                    }
                }
            }
        }
    }
}

// find a node's index into _short_path_data array from it's id (i.e. id type and id number)
// returns true if successful and node_idx is updated
bool AP_OADijkstra::find_node_from_id(const AP_OAVisGraph::OAItemID &id, node_index &node_idx) const
{
    switch (id.id_type) {
    case AP_OAVisGraph::OATYPE_SOURCE:
        // source node is always the first node
        if (_short_path_data_numpoints > 0) {
            node_idx = 0;
            return true;
        }
        break;
    case AP_OAVisGraph::OATYPE_DESTINATION:
        // destination is always the 2nd node
        if (_short_path_data_numpoints > 1) {
            node_idx = 1;
            return true;
        }
        break;
    case AP_OAVisGraph::OATYPE_INTERMEDIATE_POINT:
        // intermediate nodes start from 3rd node
        if (_short_path_data_numpoints > id.id_num + 2) {
            node_idx = id.id_num + 2;
            return true;
        }
        break;
    }

    // could not find node
    return false;
}

// find index of node with lowest tentative distance (ignore visited nodes)
// returns true if successful and node_idx argument is updated
bool AP_OADijkstra::find_closest_node_idx(node_index &node_idx) const
{
    node_index lowest_idx = 0;
    float lowest_dist = FLT_MAX;

    // scan through all nodes looking for closest
    for (node_index i=0; i<_short_path_data_numpoints; i++) {
        const ShortPathNode &node = _short_path_data[i];
        if (node.visited || is_equal(_short_path_data[i].distance_cm, FLT_MAX)) {
            // if node is already visited OR cannot be reached yet, we can't use it
            continue;
        }
        // figure out the pos of this node
        Vector2f node_pos;
        float dist_with_heuristics = FLT_MAX;
        if (convert_node_to_point(node.id, node_pos)) {
            // heuristics is is simple Euclidean distance from the node to the destination
            // This should be admissible, therefore optimal path is guaranteed
            const float heuristics = (node_pos-_path_destination).length();
            dist_with_heuristics = node.distance_cm + heuristics;
        } else {
            // shouldn't happen
            return false;
        }
        if (dist_with_heuristics < lowest_dist) {
            // for NOW, this is the closest node
            lowest_idx = i;
            lowest_dist = dist_with_heuristics;
        }
    }

    if (lowest_dist < FLT_MAX) {
        // found the closest node
        node_idx = lowest_idx;
        return true;
    }
    return false;
}

// calculate shortest path from origin to destination
// returns true on success.  returns false on failure and err_id is updated
// requires these functions to have been run: create_inclusion_polygon_with_margin, create_exclusion_polygon_with_margin, create_exclusion_circle_with_margin, create_polygon_fence_visgraph
// resulting path is stored in _shortest_path array as vector offsets from EKF origin
bool AP_OADijkstra::calc_shortest_path(const Location &origin, const Location &destination, AP_OADijkstra_Error &err_id)
{
    // convert origin and destination to offsets from EKF origin
    if (!origin.get_vector_xy_from_origin_NE(_path_source) || !destination.get_vector_xy_from_origin_NE(_path_destination)) {
        err_id = AP_OADijkstra_Error::DIJKSTRA_ERROR_NO_POSITION_ESTIMATE;
        return false;
    }

    // create visgraphs of origin and destination to fence points
    if (!update_visgraph(_source_visgraph, {AP_OAVisGraph::OATYPE_SOURCE, 0}, _path_source, true, _path_destination)) {
        err_id = AP_OADijkstra_Error::DIJKSTRA_ERROR_OUT_OF_MEMORY;
        return false;
    }
    if (!update_visgraph(_destination_visgraph, {AP_OAVisGraph::OATYPE_DESTINATION, 0}, _path_destination)) {
        err_id = AP_OADijkstra_Error::DIJKSTRA_ERROR_OUT_OF_MEMORY;
        return false;
    }

    // expand _short_path_data if necessary
    if (!_short_path_data.expand_to_hold(2 + total_numpoints())) {
        err_id = AP_OADijkstra_Error::DIJKSTRA_ERROR_OUT_OF_MEMORY;
        return false;
    }

    // add origin and destination (node_type, id, visited, distance_from_idx, distance_cm) to short_path_data array
    _short_path_data[0] = {{AP_OAVisGraph::OATYPE_SOURCE, 0}, false, 0, 0};
    _short_path_data[1] = {{AP_OAVisGraph::OATYPE_DESTINATION, 0}, false, OA_DIJKSTRA_POLYGON_SHORTPATH_NOTSET_IDX, FLT_MAX};
    _short_path_data_numpoints = 2;

    // add all inclusion and exclusion fence points to short_path_data array (node_type, id, visited, distance_from_idx, distance_cm)
    for (uint8_t i=0; i<total_numpoints(); i++) {
        _short_path_data[_short_path_data_numpoints++] = {{AP_OAVisGraph::OATYPE_INTERMEDIATE_POINT, i}, false, OA_DIJKSTRA_POLYGON_SHORTPATH_NOTSET_IDX, FLT_MAX};
    }

    // start algorithm from source point
    node_index current_node_idx = 0;

    // update nodes visible from source point
    for (uint16_t i = 0; i < _source_visgraph.num_items(); i++) {
        node_index node_idx;
        if (find_node_from_id(_source_visgraph[i].id2, node_idx)) {
            _short_path_data[node_idx].distance_cm = _source_visgraph[i].distance_cm;
            _short_path_data[node_idx].distance_from_idx = current_node_idx;
        } else {
            err_id = AP_OADijkstra_Error::DIJKSTRA_ERROR_COULD_NOT_FIND_PATH;
            return false;
        }
    }
    // mark source node as visited
    _short_path_data[current_node_idx].visited = true;

    // move current_node_idx to node with lowest distance
    while (find_closest_node_idx(current_node_idx)) {
        node_index dest_node;
        // See if this next "closest" node is actually the destination
        if (find_node_from_id({AP_OAVisGraph::OATYPE_DESTINATION,0}, dest_node) && current_node_idx == dest_node) {
            // We have discovered destination.. Don't bother with the rest of the graph
            break;
        }
        // update distances to all neighbours of current node
        update_visible_node_distances(current_node_idx);

        // mark current node as visited
        _short_path_data[current_node_idx].visited = true;
    }

    // extract path starting from destination
    bool success = false;
    node_index nidx;
    if (!find_node_from_id({AP_OAVisGraph::OATYPE_DESTINATION,0}, nidx)) {
        err_id = AP_OADijkstra_Error::DIJKSTRA_ERROR_COULD_NOT_FIND_PATH;
        return false;
    }
    _path_numpoints = 0;
    while (true) {
        if (!_path.expand_to_hold(_path_numpoints + 1)) {
            err_id = AP_OADijkstra_Error::DIJKSTRA_ERROR_OUT_OF_MEMORY;
            return false;
        }
        // fail if newest node has invalid distance_from_index
        if ((_short_path_data[nidx].distance_from_idx == OA_DIJKSTRA_POLYGON_SHORTPATH_NOTSET_IDX) ||
            (_short_path_data[nidx].distance_cm >= FLT_MAX)) {
            break;
        } else {
            // add node's id to path array
            _path[_path_numpoints] = _short_path_data[nidx].id;
            _path_numpoints++;

            // we are done if node is the source
            if (_short_path_data[nidx].id.id_type == AP_OAVisGraph::OATYPE_SOURCE) {
                success = true;
                break;
            } else {
                // follow node's "distance_from_idx" to previous node on path
                nidx = _short_path_data[nidx].distance_from_idx;
            }
        }
    }
    // report error incase path not found
    if (!success) {
        err_id = AP_OADijkstra_Error::DIJKSTRA_ERROR_COULD_NOT_FIND_PATH;
    }

    return success;
}

// return point from final path as an offset (in cm) from the ekf origin
bool AP_OADijkstra::get_shortest_path_point(uint8_t point_num, Vector2f& pos)
{
    if ((_path_numpoints == 0) || (point_num >= _path_numpoints)) {
        return false;
    }

    // get id from path
    AP_OAVisGraph::OAItemID id = _path[_path_numpoints - point_num - 1];

    return convert_node_to_point(id, pos);
}

// find the position of a node as an offset (in cm) from the ekf origin
bool AP_OADijkstra::convert_node_to_point(const AP_OAVisGraph::OAItemID& id, Vector2f& pos) const
{
    // convert id to a position offset from EKF origin
    switch (id.id_type) {
    case AP_OAVisGraph::OATYPE_SOURCE:
        pos = _path_source;
        return true;
    case AP_OAVisGraph::OATYPE_DESTINATION:
        pos = _path_destination;
        return true;
    case AP_OAVisGraph::OATYPE_INTERMEDIATE_POINT:
        return get_point(id.id_num, pos);
    }

    // we should never reach here but just in case
    return false;
}
#endif // AP_FENCE_ENABLED

