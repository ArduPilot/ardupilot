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
#include <AC_Fence/AC_Fence.h>

#define OA_DIJKSTRA_POLYGON_VISGRAPH_PTS         (OA_DIJKSTRA_POLYGON_FENCE_PTS * OA_DIJKSTRA_POLYGON_FENCE_PTS / 2)
#define OA_DIJKSTRA_POLYGON_SHORTPATH_NOTSET_IDX 255     // index use to indicate we do not have a tentative short path for a node

/// Constructor
AP_OADijkstra::AP_OADijkstra() :
        _polyfence_visgraph(OA_DIJKSTRA_POLYGON_VISGRAPH_PTS),
        _source_visgraph(OA_DIJKSTRA_POLYGON_FENCE_PTS),
        _destination_visgraph(OA_DIJKSTRA_POLYGON_FENCE_PTS)
{
}

// calculate a destination to avoid the polygon fence
// returns true if avoidance is required and updates origin_new and destination_new
bool AP_OADijkstra::update(const Location &current_loc, const Location &destination, Location& origin_new, Location& destination_new)
{
    // require ekf origin to have been set
    struct Location ekf_origin {};
    if (!AP::ahrs().get_origin(ekf_origin)) {
        return false;
    }

    // no avoidance required if fence is disabled
    if (!polygon_fence_enabled()) {
        return false;
    }

    // check for fence updates
    if (check_polygon_fence_updated()) {
        _polyfence_with_margin_ok = false;
    }

    // create inner polygon fence
    if (!_polyfence_with_margin_ok) {
        _polyfence_with_margin_ok = create_polygon_fence_with_margin(_polyfence_margin * 100.0f);
        if (!_polyfence_with_margin_ok) {
            _polyfence_visgraph_ok = false;
            _shortest_path_ok = false;
            return false;
        }
    }

    // create visgraph for inner polygon fence
    if (!_polyfence_visgraph_ok) {
        _polyfence_visgraph_ok = create_polygon_fence_visgraph();
        if (!_polyfence_visgraph_ok) {
            _shortest_path_ok = false;
            return false;
        }
    }

    // rebuild path if destination has changed
    if (!destination.same_latlon_as(_destination_prev)) {
        _destination_prev = destination;
        _shortest_path_ok = false;
    }

    // calculate shortest path from current_loc to destination
    if (!_shortest_path_ok) {
        _shortest_path_ok = calc_shortest_path(current_loc, destination);
        if (!_shortest_path_ok) {
            return false;
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
            Location temp_loc(Vector3f(origin_pos.x, origin_pos.y, 0.0f));
            origin_new = temp_loc;
        } else {
            // for first point use current loc as origin
            origin_new = current_loc;
        }

        // convert offset from ekf origin to Location
        Location temp_loc(Vector3f(dest_pos.x, dest_pos.y, 0.0f));
        destination_new = destination;
        destination_new.lat = temp_loc.lat;
        destination_new.lng = temp_loc.lng;

        // check if we should advance to next point for next iteration
        const bool near_oa_wp = current_loc.get_distance(destination_new) <= 2.0f;
        const bool past_oa_wp = current_loc.past_interval_finish_line(origin_new, destination_new);
        if (near_oa_wp || past_oa_wp) {
            _path_idx_returned++;
        }
        return true;
    }

    return false;
}

// returns true if polygon fence is enabled
bool AP_OADijkstra::polygon_fence_enabled() const
{
    const AC_Fence *fence = AC_Fence::get_singleton();
    if (fence == nullptr) {
        return false;
    }
    return ((fence->get_enabled_fences() & AC_FENCE_TYPE_POLYGON) > 0);
}

// check if polygon fence has been updated since we created the inner fence. returns true if changed
bool AP_OADijkstra::check_polygon_fence_updated() const
{
    // exit immediately if polygon fence is not enabled
    const AC_Fence *fence = AC_Fence::get_singleton();
    if (fence == nullptr) {
        return false;
    }
    return (_polyfence_update_ms != fence->get_boundary_update_ms());
}

// create a smaller polygon fence within the existing polygon fence
// returns true on success
bool AP_OADijkstra::create_polygon_fence_with_margin(float margin_cm)
{
    // exit immediately if polygon fence is not enabled
    const AC_Fence *fence = AC_Fence::get_singleton();
    if (fence == nullptr) {
        return false;
    }

    // get polygon boundary
    uint16_t num_points;
    const Vector2f* boundary = fence->get_boundary_points(num_points);
    if ((boundary == nullptr) || (num_points < 3)) {
        return false;
    }

    // fail if fence is too large
    if (num_points > OA_DIJKSTRA_POLYGON_FENCE_PTS) {
        return false;
    }

    // for each point on polygon fence
    // Note: boundary is "unclosed" meaning the last point is *not* the same as the first
    for (uint8_t i=0; i<num_points; i++) {

        // find points before and after current point (relative to current point)
        const uint8_t before_idx = (i == 0) ? num_points-1 : i-1;
        const uint8_t after_idx = (i == num_points-1) ? 0 : i+1;
        Vector2f before_pt = boundary[before_idx] - boundary[i];
        Vector2f after_pt = boundary[after_idx] - boundary[i];

        // if points are overlapping fail
        if (before_pt.is_zero() || after_pt.is_zero() || (before_pt == after_pt)) {
            return false;
        }

        // scale points to be unit vectors
        before_pt.normalize();
        after_pt.normalize();

        // calculate intermediate point and scale to margin
        Vector2f intermediate_pt = (after_pt + before_pt) * 0.5f;
        float intermediate_len = intermediate_pt.length();
        intermediate_pt *= (margin_cm / intermediate_len);

        // find final point which is inside the original polygon
        _polyfence_pts[i] = boundary[i] + intermediate_pt;
        if (Polygon_outside(_polyfence_pts[i], boundary, num_points)) {
            _polyfence_pts[i] = boundary[i] - intermediate_pt;
            if (Polygon_outside(_polyfence_pts[i], boundary, num_points)) {
                // could not find a point on either side that was within the fence so fail
                // this can happen if fence lines are closer than margin_cm
                return false;
            }
        }
    }

    // update number of fence points
    _polyfence_numpoints = num_points;

    // record fence update time so we don't process this exact fence again
    _polyfence_update_ms = fence->get_boundary_update_ms();

    return true;
}

// create a visibility graph of the polygon fence
// returns true on success
// requires create_polygon_fence_with_margin to have been run
bool AP_OADijkstra::create_polygon_fence_visgraph()
{
    // exit immediately if no polygon fence (with margin)
    if (_polyfence_numpoints == 0) {
        return false;
    }

    // exit immediately if polygon fence is not enabled
    const AC_Fence *fence = AC_Fence::get_singleton();
    if (fence == nullptr) {
        return false;
    }

    // get polygon boundary
    uint16_t num_points;
    const Vector2f* boundary = fence->get_boundary_points(num_points);
    if ((boundary == nullptr) || (num_points < 3)) {
        return false;
    }

    // clear polygon visibility graph
    _polyfence_visgraph.clear();

    // calculate distance from each polygon fence point to all other points
    for (uint8_t i=0; i<_polyfence_numpoints-1; i++) {
        const Vector2f &start1 = _polyfence_pts[i];
        for (uint8_t j=i+1; j<_polyfence_numpoints-1; j++) {
            const Vector2f &end1 = _polyfence_pts[j];
            Vector2f intersection;
            // ToDo: calculation below could be sped up by removing unused intersection and
            //       skipping segments we know are parallel to our fence-with-margin segments
            if (!Polygon_intersects(boundary, num_points, start1, end1, intersection)) {
                // line segment does not intersect with original fence so add to visgraph
                _polyfence_visgraph.add_item({AP_OAVisGraph::OATYPE_FENCE_POINT, i},
                                             {AP_OAVisGraph::OATYPE_FENCE_POINT, j},
                                             (_polyfence_pts[i] - _polyfence_pts[j]).length());
            }
            // ToDo: store infinity when there is no clear path between points to allow faster search later
        }
    }

    return true;
}

// updates visibility graph for a given position which is an offset (in cm) from the ekf origin
// to add an additional position (i.e. the destination) set add_extra_position = true and provide the position in the extra_position argument
// requires create_polygon_fence_with_margin to have been run
// returns true on success
bool AP_OADijkstra::update_visgraph(AP_OAVisGraph& visgraph, const AP_OAVisGraph::OAItemID& oaid, const Vector2f &position, bool add_extra_position, Vector2f extra_position)
{
    // exit immediately if no polygon fence (with margin)
    if (_polyfence_numpoints == 0) {
        return false;
    }

    // exit immediately if polygon fence is not enabled
    const AC_Fence *fence = AC_Fence::get_singleton();
    if (fence == nullptr) {
        return false;
    }

    // get polygon boundary
    uint16_t num_points;
    const Vector2f* boundary = fence->get_boundary_points(num_points);
    if ((boundary == nullptr) || (num_points < 3)) {
        return false;
    }

    // clear visibility graph
    visgraph.clear();

    // calculate distance from position to all fence points
    for (uint8_t i=0; i<_polyfence_numpoints-1; i++) {
        Vector2f intersection;
        if (!Polygon_intersects(boundary, num_points, position, _polyfence_pts[i], intersection)) {
            // line segment does not intersect with original fence so add to visgraph
            visgraph.add_item(oaid, {AP_OAVisGraph::OATYPE_FENCE_POINT, i}, (position - _polyfence_pts[i]).length());
        }
        // ToDo: store infinity when there is no clear path between points to allow faster search later
    }

    // add extra point to visibility graph if it doesn't intersect with polygon fence
    if (add_extra_position) {
        Vector2f intersection;
        if (!Polygon_intersects(boundary, num_points, position, extra_position, intersection)) {
            visgraph.add_item(oaid, {AP_OAVisGraph::OATYPE_DESTINATION, 0}, (position - extra_position).length());
        }
    }

    return true;
}

// update total distance for all nodes visible from current node
// curr_node_idx is an index into the _short_path_data array
void AP_OADijkstra::update_visible_node_distances(node_index curr_node_idx)
{
    // sanity check
    if (curr_node_idx > _short_path_data_numpoints) {
        return;
    }

    // get current node for convenience
    const ShortPathNode &curr_node = _short_path_data[curr_node_idx];

    // for each visibility graph
    const AP_OAVisGraph* visgraphs[] = {&_polyfence_visgraph, &_destination_visgraph};
    for (uint8_t v=0; v<ARRAY_SIZE(visgraphs); v++) {

        // skip if empty
        const AP_OAVisGraph &curr_visgraph = *visgraphs[v];
        if (curr_visgraph.num_items() == 0) {
            continue;
        }

        // search visibility graph for items visible from current_node
        for (uint8_t i=0; i<curr_visgraph.num_items(); i++) {
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
    case AP_OAVisGraph::OATYPE_FENCE_POINT:
        // must be a fence node which start from 3rd node
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
        if (!node.visited && (node.distance_cm < lowest_dist)) {
            lowest_idx = i;
            lowest_dist = node.distance_cm;
        }
    }

    if (lowest_dist < FLT_MAX) {
        node_idx = lowest_idx;
        return true;
    }
    return false;
}

// calculate shortest path from origin to destination
// returns true on success
// requires create_polygon_fence_with_margin and create_polygon_fence_visgraph to have been run
// resulting path is stored in _shortest_path array as vector offsets from EKF origin
bool AP_OADijkstra::calc_shortest_path(const Location &origin, const Location &destination)
{
    // convert origin and destination to offsets from EKF origin
    Vector2f origin_NE, destination_NE;
    if (!origin.get_vector_xy_from_origin_NE(origin_NE) || !destination.get_vector_xy_from_origin_NE(destination_NE)) {
        return false;
    }

    // create origin and destination visgraphs of polygon points
    update_visgraph(_source_visgraph, {AP_OAVisGraph::OATYPE_SOURCE, 0}, origin_NE, true, destination_NE);
    update_visgraph(_destination_visgraph, {AP_OAVisGraph::OATYPE_DESTINATION, 0}, destination_NE);

    // check OA_DIJKSTRA_POLYGON_SHORTPATH_NOTSET_IDX is defined correct
    static_assert(OA_DIJKSTRA_POLYGON_SHORTPATH_NOTSET_IDX > OA_DIJKSTRA_POLYGON_FENCE_PTS, "check OA_DIJKSTRA_POLYGON_SHORTPATH_NOTSET_IDX > OA_DIJKSTRA_POLYGON_FENCE_PTS");

    // add origin and destination (node_type, id, visited, distance_from_idx, distance_cm) to short_path_data array
    _short_path_data[0] = {{AP_OAVisGraph::OATYPE_SOURCE, 0}, false, 0, 0};
    _short_path_data[1] = {{AP_OAVisGraph::OATYPE_DESTINATION, 0}, false, OA_DIJKSTRA_POLYGON_SHORTPATH_NOTSET_IDX, FLT_MAX};
    _short_path_data_numpoints = 2;

    // add fence points to short_path_data array (node_type, id, visited, distance_from_idx, distance_cm)
    // skip last fence point because it is the same as the first
    for (uint8_t i=0; i<_polyfence_numpoints-1; i++) {
        _short_path_data[_short_path_data_numpoints++] = {{AP_OAVisGraph::OATYPE_FENCE_POINT, i}, false, OA_DIJKSTRA_POLYGON_SHORTPATH_NOTSET_IDX, FLT_MAX};
    }

    // start algorithm from source point
    node_index current_node_idx = 0;

    // update nodes visible from source point
    for (uint8_t i=0; i<_source_visgraph.num_items(); i++) {
        node_index node_idx;
        if (find_node_from_id(_source_visgraph[i].id2, node_idx)) {
            _short_path_data[node_idx].distance_cm = _source_visgraph[i].distance_cm;
            _short_path_data[node_idx].distance_from_idx = current_node_idx;
        } else {
            return false;
        }
    }
    // mark source node as visited
    _short_path_data[current_node_idx].visited = true;

    // move current_node_idx to node with lowest distance
    while (find_closest_node_idx(current_node_idx)) {
        // update distances to all neighbours of current node
        update_visible_node_distances(current_node_idx);

        // mark current node as visited
        _short_path_data[current_node_idx].visited = true;
    }

    // extract path starting from destination
    bool success = false;
    node_index nidx;
    if (!find_node_from_id({AP_OAVisGraph::OATYPE_DESTINATION,0}, nidx)) {
        return false;
    }
    _path_numpoints = 0;
    while (true) {
        // fail if out of space or newest node has invalid distance or from index
        if ((_path_numpoints >= OA_DIJKSTRA_POLYGON_SHORTPATH_PTS) ||
            (_short_path_data[nidx].distance_from_idx == OA_DIJKSTRA_POLYGON_SHORTPATH_NOTSET_IDX) ||
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
    // update source and destination for by get_shortest_path_point
    if (success) {
        _path_source = origin_NE;
        _path_destination = destination_NE;
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

    // convert id to a position offset from EKF origin
    switch (id.id_type) {
    case AP_OAVisGraph::OATYPE_SOURCE:
        pos = _path_source;
        return true;
    case AP_OAVisGraph::OATYPE_DESTINATION:
        pos = _path_destination;
        return true;
    case AP_OAVisGraph::OATYPE_FENCE_POINT:
        // sanity check polygon fence has the point
        if (id.id_num < _polyfence_numpoints) {
            pos = _polyfence_pts[id.id_num];
            return true;
        }
        return false;
    }

    // we should never reach here but just in case
    return false;
}
