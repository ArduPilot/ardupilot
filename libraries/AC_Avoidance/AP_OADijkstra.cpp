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

#include "AP_OADijkstra.h"
#include "AP_OAPathPlanner.h"

#include <AC_Fence/AC_Fence.h>

#if AP_FENCE_ENABLED

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

#define OA_DIJKSTRA_EXPANDING_ARRAY_ELEMENTS_PER_CHUNK  32      // expanding arrays for fence points and paths to destination will grow in increments of 20 elements
#define OA_DIJKSTRA_POLYGON_SHORTPATH_NOTSET_IDX        255     // index use to indicate we do not have a tentative short path for a node

/// Constructor
AP_OADijkstra::AP_OADijkstra(const AP_Int16 &options) :
        _options(options),
        _static_data(_common, options),
        _calcpath(_static_data)
{
}

// calculate a destination to avoid fences
// returns DIJKSTRA_STATE_SUCCESS and populates origin_new, destination_new and next_destination_new if avoidance is required
// next_destination_new will be non-zero if there is a next destination
// dest_to_next_dest_clear will be set to true if the path from (the input) destination to (input) next_destination is clear
AP_OADijkstra::AP_OADijkstra_State AP_OADijkstra::update(const Location &current_loc,
                                                         const Location &destination,
                                                         const Location &next_destination,
                                                         Location& origin_new,
                                                         Location& destination_new,
                                                         Location& next_destination_new,
                                                         bool& dest_to_next_dest_clear)
{
    WITH_SEMAPHORE(AP::fence()->polyfence().get_loaded_fence_semaphore());

    // check static data (e.g. fences)
    AP_OADijkstra_Common::ErrorId error_id;
    switch (_static_data.update(error_id)) {
    case AP_OADijkstra_StaticData::UpdateState::NOT_REQUIRED:
        dest_to_next_dest_clear = _dest_to_next_dest_clear = true;
        Write_OADijkstra(DIJKSTRA_STATE_NOT_REQUIRED, 0, 0, 0, destination, destination);
        return DIJKSTRA_STATE_NOT_REQUIRED;
    case AP_OADijkstra_StaticData::UpdateState::READY:
        // all static data is already up-to-date
        break;
    case AP_OADijkstra_StaticData::UpdateState::UPDATED:
        // static data has changed so recalculate path
        _shortest_path_ok = false;
        break;
    case AP_OADijkstra_StaticData::UpdateState::ERROR:
        _common.report_error(error_id);
        Write_OADijkstra(DIJKSTRA_STATE_ERROR, (uint8_t)error_id, 0, 0, destination, destination);
        return DIJKSTRA_STATE_ERROR;
    }

    // no avoidance required if destination is same as current location
    if (current_loc.same_latlon_as(destination)) {
        // we do not check path to next destination so conservatively set to false
        dest_to_next_dest_clear = _dest_to_next_dest_clear = false;
        Write_OADijkstra(DIJKSTRA_STATE_NOT_REQUIRED, 0, 0, 0, destination, destination);
        return DIJKSTRA_STATE_NOT_REQUIRED;
    }

    // rebuild path if destination or next_destination has changed
    if (!destination.same_latlon_as(_destination_prev) || !next_destination.same_latlon_as(_next_destination_prev)) {
        _destination_prev = destination;
        _next_destination_prev = next_destination;
        _shortest_path_ok = false;
    }

    // calculate shortest path from current_loc to destination
    if (!_shortest_path_ok) {
        _shortest_path_ok = _calcpath.calc_shortest_path(current_loc, destination, _shortest_path, error_id);
        if (!_shortest_path_ok) {
            dest_to_next_dest_clear = _dest_to_next_dest_clear = false;
            _common.report_error(error_id);
            Write_OADijkstra(DIJKSTRA_STATE_ERROR, (uint8_t)error_id, 0, 0, destination, destination);
            return DIJKSTRA_STATE_ERROR;
        }
        // start from 2nd point on path (first is the original origin)
        _path_idx_returned = 1;

        // check if path from destination to next_destination intersects with a fence
        _dest_to_next_dest_clear = false;
        if (!next_destination.is_zero()) {
            Vector2f seg_start, seg_end;
            if (destination.get_vector_xy_from_origin_NE(seg_start) && next_destination.get_vector_xy_from_origin_NE(seg_end)) {
                _dest_to_next_dest_clear = !_static_data.intersects_fence(seg_start, seg_end);
            }
        }
    }

    // path has been created, return latest point
    Vector2f dest_pos;
    const uint8_t path_length = _shortest_path.num_pos > 0 ? (_shortest_path.num_pos - 1) : 0;
    if ((_path_idx_returned < path_length) && _shortest_path.get_position(_path_idx_returned, dest_pos)) {

        // for the first point return origin as current_loc
        Vector2f origin_pos;
        if ((_path_idx_returned > 0) && _shortest_path.get_position(_path_idx_returned-1, origin_pos)) {
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

        // provide next destination to allow smooth cornering
        next_destination_new.zero();
        Vector2f next_dest_pos;
        if ((_path_idx_returned + 1 < path_length) && _shortest_path.get_position(_path_idx_returned + 1, next_dest_pos)) {
            // convert offset from ekf origin to Location
            Location next_loc(Vector3f{next_dest_pos.x, next_dest_pos.y, 0.0}, Location::AltFrame::ABOVE_ORIGIN);
            next_destination_new = destination;
            next_destination_new.lat = next_loc.lat;
            next_destination_new.lng = next_loc.lng;
        } else {
            // return destination as next_destination
            next_destination_new = destination;
        }

        // path to next destination clear state is still valid from previous calcs (was calced along with shortest path)
        dest_to_next_dest_clear = _dest_to_next_dest_clear;

        // check if we should advance to next point for next iteration
        const bool near_oa_wp = current_loc.get_distance(destination_new) <= 2.0f;
        const bool past_oa_wp = current_loc.past_interval_finish_line(origin_new, destination_new);
        if (near_oa_wp || past_oa_wp) {
            _path_idx_returned++;
        }
        // log success
        Write_OADijkstra(DIJKSTRA_STATE_SUCCESS, 0, _path_idx_returned, _shortest_path.num_pos, destination, destination_new);
        return DIJKSTRA_STATE_SUCCESS;
    }

    // we have reached the destination so avoidance is no longer required
    // path to next destination clear state is still valid from previous calcs
    dest_to_next_dest_clear = _dest_to_next_dest_clear;
    Write_OADijkstra(DIJKSTRA_STATE_NOT_REQUIRED, 0, 0, 0, destination, destination);
    return DIJKSTRA_STATE_NOT_REQUIRED;
}

// calculate the length of a path between origin and destination in meters
// this calculation takes time and should only be run from a background thread
// returns true on success and fills in path_length argument
// called by AP_OAPathPlanner::get_path_length()
AP_OADijkstra::AP_OADijkstra_State AP_OADijkstra::get_path_length(const Location &origin, const Location& destination, float& path_length)
{
    // check static data (e.g. fences)
    AP_OADijkstra_Common::ErrorId error_id;
    switch (_static_data.update(error_id)) {
    case AP_OADijkstra_StaticData::UpdateState::NOT_REQUIRED:
        return DIJKSTRA_STATE_NOT_REQUIRED;
    case AP_OADijkstra_StaticData::UpdateState::READY:
    case AP_OADijkstra_StaticData::UpdateState::UPDATED:
        // all static data is already up-to-date
        break;
    case AP_OADijkstra_StaticData::UpdateState::ERROR:
        return DIJKSTRA_STATE_ERROR;
    }

    // no avoidance required if origin and destination are the same
    if (origin.same_latlon_as(destination)) {
        return DIJKSTRA_STATE_NOT_REQUIRED;
    }

    // calculate path length
    if (_calcpath.calc_shortest_path(origin, destination, _secondary_path, error_id)) {
        path_length = _secondary_path.length_cm * 0.01;
        return AP_OADijkstra_State::DIJKSTRA_STATE_SUCCESS;
    }
    return (error_id == AP_OADijkstra_Common::ErrorId::DIJKSTRA_ERROR_FENCE_DISABLED) ? AP_OADijkstra_State::DIJKSTRA_STATE_NOT_REQUIRED : AP_OADijkstra_State::DIJKSTRA_STATE_ERROR;
}

#endif // AP_FENCE_ENABLED


#endif  // AP_OAPATHPLANNER_DIJKSTRA_ENABLED
