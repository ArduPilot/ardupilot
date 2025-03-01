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

#include "AC_Avoidance_config.h"

#if AP_OAPATHPLANNER_DIJKSTRA_ENABLED

#include "AP_OADijkstra_StaticData.h"
#include "AP_OAPathPlanner.h"

#include <AC_Fence/AC_Fence.h>

#if AP_FENCE_ENABLED

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>

#define OA_DIJKSTRA_EXPANDING_ARRAY_ELEMENTS_PER_CHUNK  32      // expanding arrays for fence points and paths to destination will grow in increments of 20 elements
#define OA_DIJKSTRA_POLYGON_SHORTPATH_NOTSET_IDX        255     // index use to indicate we do not have a tentative short path for a node

/// Constructor
AP_OADijkstra_StaticData::AP_OADijkstra_StaticData(AP_OADijkstra_Common& common, const AP_Int16 &options):
        _inclusion_polygon_pts(OA_DIJKSTRA_EXPANDING_ARRAY_ELEMENTS_PER_CHUNK),
        _exclusion_polygon_pts(OA_DIJKSTRA_EXPANDING_ARRAY_ELEMENTS_PER_CHUNK),
        _exclusion_circle_pts(OA_DIJKSTRA_EXPANDING_ARRAY_ELEMENTS_PER_CHUNK),
        _common(common),
        _options(options)
{
}

// returns true if at least one inclusion or exclusion zone is enabled
bool AP_OADijkstra_StaticData::some_fences_enabled() const
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

// updates static data required for Dijkstra's algorithm
// returns READY on success, UPDATED if change in static data since previous call means path should be re-calculated
AP_OADijkstra_StaticData::UpdateState AP_OADijkstra_StaticData::update(AP_OADijkstra_Common::ErrorId& error_id)
{
    WITH_SEMAPHORE(AP::fence()->polyfence().get_loaded_fence_semaphore());

    // if no fences then no static data is required
    if (!some_fences_enabled()) {
        return UpdateState::NOT_REQUIRED;
    }

    // check for inclusion polygon updates
    bool updated = false;
    if (check_inclusion_polygon_updated()) {
        if (!create_inclusion_polygon_with_margin(_polyfence_margin * 100.0f, error_id)) {
            _common.report_error(error_id);
            return UpdateState::ERROR;
        }
        updated = true;
    }

    // check for exclusion polygon updates
    if (check_exclusion_polygon_updated()) {
        if (!create_exclusion_polygon_with_margin(_polyfence_margin * 100.0f, error_id)) {
            _common.report_error(error_id);
            return UpdateState::ERROR;
        }
        updated = true;
    }

    // check for exclusion circle updates
    if (check_exclusion_circle_updated()) {
        if (!create_exclusion_circle_with_margin(_polyfence_margin * 100.0f, error_id)) {
            _common.report_error(error_id);
            return UpdateState::ERROR;
        }
        updated = true;
    }

    // create visgraph for all fence (with margin) points
    if (!_polyfence_visgraph_ok || updated) {
        _polyfence_visgraph_ok = create_fence_visgraph(error_id);
        if (!_polyfence_visgraph_ok) {
            _common.report_error(error_id);
            return UpdateState::ERROR;
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

    return updated ? UpdateState::UPDATED : UpdateState::READY;
}

// check if polygon fence has been updated since we created the inner fence. returns true if changed
bool AP_OADijkstra_StaticData::check_inclusion_polygon_updated() const
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
bool AP_OADijkstra_StaticData::create_inclusion_polygon_with_margin(float margin_cm, AP_OADijkstra_Common::ErrorId &error_id)
{
    const AC_Fence *fence = AC_Fence::get_singleton();

    if (fence == nullptr) {
        error_id = AP_OADijkstra_Common::ErrorId::DIJKSTRA_ERROR_FENCE_DISABLED;
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
                error_id = AP_OADijkstra_Common::ErrorId::DIJKSTRA_ERROR_OVERLAPPING_POLYGON_POINTS;
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
                    error_id = AP_OADijkstra_Common::ErrorId::DIJKSTRA_ERROR_OVERLAPPING_POLYGON_LINES;
                    return false;
                }
            }

            // don't add points in corners
            if (fabsf(intermediate_pt.angle() - before_pt.angle()) < M_PI_2) {
                continue;
            }

            // expand array if required
            if (!_inclusion_polygon_pts.expand_to_hold(_inclusion_polygon_numpoints + new_points + 1)) {
                error_id = AP_OADijkstra_Common::ErrorId::DIJKSTRA_ERROR_OUT_OF_MEMORY;
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
bool AP_OADijkstra_StaticData::check_exclusion_polygon_updated() const
{
    const AC_Fence *fence = AC_Fence::get_singleton();
    if (fence == nullptr) {
        return false;
    }
    return (_exclusion_polygon_update_ms != fence->polyfence().get_exclusion_polygon_update_ms());
}

// create polygons around existing exclusion polygons
// returns true on success.  returns false on failure and err_id is updated
bool AP_OADijkstra_StaticData::create_exclusion_polygon_with_margin(float margin_cm, AP_OADijkstra_Common::ErrorId &err_id)
{
    const AC_Fence *fence = AC_Fence::get_singleton();

    if (fence == nullptr) {
        err_id = AP_OADijkstra_Common::ErrorId::DIJKSTRA_ERROR_FENCE_DISABLED;
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
                err_id = AP_OADijkstra_Common::ErrorId::DIJKSTRA_ERROR_OVERLAPPING_POLYGON_POINTS;
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
                    err_id = AP_OADijkstra_Common::ErrorId::DIJKSTRA_ERROR_OVERLAPPING_POLYGON_LINES;
                    return false;
                }
            }

            // don't add points in corners
            if (fabsf(intermediate_pt.angle() - before_pt.angle()) < M_PI_2) {
                continue;
            }

            // expand array if required
            if (!_exclusion_polygon_pts.expand_to_hold(_exclusion_polygon_numpoints + new_points + 1)) {
                err_id = AP_OADijkstra_Common::ErrorId::DIJKSTRA_ERROR_OUT_OF_MEMORY;
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
bool AP_OADijkstra_StaticData::check_exclusion_circle_updated() const
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
bool AP_OADijkstra_StaticData::create_exclusion_circle_with_margin(float margin_cm, AP_OADijkstra_Common::ErrorId &err_id)
{
    // exit immediately if fence is not enabled
    const AC_Fence *fence = AC_Fence::get_singleton();
    if (fence == nullptr) {
        err_id = AP_OADijkstra_Common::ErrorId::DIJKSTRA_ERROR_FENCE_DISABLED;
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
        err_id = AP_OADijkstra_Common::ErrorId::DIJKSTRA_ERROR_OUT_OF_MEMORY;
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
uint16_t AP_OADijkstra_StaticData::total_numpoints() const
{
    return _inclusion_polygon_numpoints + _exclusion_polygon_numpoints + _exclusion_circle_numpoints;
}

// get a single point across the total list of points from all fence types
bool AP_OADijkstra_StaticData::get_point(uint16_t index, Vector2f &point) const
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
bool AP_OADijkstra_StaticData::intersects_fence(const Vector2f &seg_start, const Vector2f &seg_end) const
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
bool AP_OADijkstra_StaticData::create_fence_visgraph(AP_OADijkstra_Common::ErrorId &err_id)
{
    // exit immediately if fence is not enabled
    const AC_Fence *fence = AC_Fence::get_singleton();
    if (fence == nullptr) {
        err_id = AP_OADijkstra_Common::ErrorId::DIJKSTRA_ERROR_FENCE_DISABLED;
        return false;
    }

    // fail if more fence points than algorithm can handle
    if (total_numpoints() >= OA_DIJKSTRA_POLYGON_SHORTPATH_NOTSET_IDX) {
        err_id = AP_OADijkstra_Common::ErrorId::DIJKSTRA_ERROR_TOO_MANY_FENCE_POINTS;
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
                            err_id = AP_OADijkstra_Common::ErrorId::DIJKSTRA_ERROR_OUT_OF_MEMORY;
                            return false;
                        }
                    }
                }
            }
        }
    }

    return true;
}

#endif // AP_FENCE_ENABLED

#endif  // AP_OAPATHPLANNER_DIJKSTRA_ENABLED
