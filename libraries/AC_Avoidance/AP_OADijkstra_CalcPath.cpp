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

#include "AP_OADijkstra_CalcPath.h"
#include "AP_OAPathPlanner.h"

#include <AC_Fence/AC_Fence.h>

#if AP_FENCE_ENABLED

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

#define OA_DIJKSTRA_EXPANDING_ARRAY_ELEMENTS_PER_CHUNK  32      // expanding arrays for fence points and paths to destination will grow in increments of 20 elements
#define OA_DIJKSTRA_POLYGON_SHORTPATH_NOTSET_IDX        255     // index use to indicate we do not have a tentative short path for a node

/// constructor
AP_OADijkstra_CalcPath::AP_OADijkstra_CalcPath(const AP_OADijkstra_StaticData& static_data) :
    _short_path_data(OA_DIJKSTRA_EXPANDING_ARRAY_ELEMENTS_PER_CHUNK),
    _static_data(static_data)
{
}

    // path structure
AP_OADijkstra_CalcPath::Path::Path() :
    pos(OA_DIJKSTRA_EXPANDING_ARRAY_ELEMENTS_PER_CHUNK)
{
}

// return a position from the path as an offset (in cm) from the ekf origin
// index 0 is the first point (e.g the source), index of nums_pos-1 is the destination
// returns true on success and fills in the pos_ne argument
bool AP_OADijkstra_CalcPath::Path::get_position(uint8_t index, Vector2f& pos_ne) const
{
    // sanitcy check index
    if (index >= num_pos) {
        return false;
    }
    pos_ne = pos[num_pos - index - 1];
    return true;
}

// calculate shortest path from origin to destination
// returns true on success.  returns false on failure and err_id is updated
// requires these functions to have been run: create_inclusion_polygon_with_margin, create_exclusion_polygon_with_margin, create_exclusion_circle_with_margin, create_polygon_fence_visgraph
// resulting path is stored in _shortest_path array as vector offsets from EKF origin
bool AP_OADijkstra_CalcPath::calc_shortest_path(const Location &origin, const Location &destination, Path& path, AP_OADijkstra_Common::ErrorId &err_id)
{
    // convert origin and destination to offsets from EKF origin
    if (!origin.get_vector_xy_from_origin_NE(path.source) || !destination.get_vector_xy_from_origin_NE(path.destination)) {
        err_id = AP_OADijkstra_Common::ErrorId::DIJKSTRA_ERROR_NO_POSITION_ESTIMATE;
        return false;
    }

    // create visgraphs of origin and destination to fence points
    if (!update_visgraph(_source_visgraph, {AP_OAVisGraph::OATYPE_SOURCE, 0}, path.source, true, path.destination)) {
        err_id = AP_OADijkstra_Common::ErrorId::DIJKSTRA_ERROR_OUT_OF_MEMORY;
        return false;
    }
    if (!update_visgraph(_destination_visgraph, {AP_OAVisGraph::OATYPE_DESTINATION, 0}, path.destination)) {
        err_id = AP_OADijkstra_Common::ErrorId::DIJKSTRA_ERROR_OUT_OF_MEMORY;
        return false;
    }

    // expand _short_path_data if necessary
    if (!_short_path_data.expand_to_hold(2 + _static_data.total_numpoints())) {
        err_id = AP_OADijkstra_Common::ErrorId::DIJKSTRA_ERROR_OUT_OF_MEMORY;
        return false;
    }

    // add origin and destination (node_type, id, visited, distance_from_idx, distance_cm) to short_path_data array
    _short_path_data[0] = {{AP_OAVisGraph::OATYPE_SOURCE, 0}, false, 0, 0};
    _short_path_data[1] = {{AP_OAVisGraph::OATYPE_DESTINATION, 0}, false, OA_DIJKSTRA_POLYGON_SHORTPATH_NOTSET_IDX, FLT_MAX};
    _short_path_data_numpoints = 2;

    // add all inclusion and exclusion fence points to short_path_data array (node_type, id, visited, distance_from_idx, distance_cm)
    for (uint8_t i=0; i<_static_data.total_numpoints(); i++) {
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
            err_id = AP_OADijkstra_Common::ErrorId::DIJKSTRA_ERROR_COULD_NOT_FIND_PATH;
            return false;
        }
    }
    // mark source node as visited
    _short_path_data[current_node_idx].visited = true;

    // move current_node_idx to node with lowest distance
   while (find_closest_node_idx(path, current_node_idx)) {
        node_index dest_node;
        // see if this next "closest" node is actually the destination
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
        err_id = AP_OADijkstra_Common::ErrorId::DIJKSTRA_ERROR_COULD_NOT_FIND_PATH;
        return false;
    }
    path.num_pos = 0;
    path.length_cm = _short_path_data[nidx].distance_cm;
    while (true) {
        if (!path.pos.expand_to_hold(path.num_pos + 1)) {
            err_id = AP_OADijkstra_Common::ErrorId::DIJKSTRA_ERROR_OUT_OF_MEMORY;
            return false;
        }
        // fail if newest node has invalid distance_from_index
        if ((_short_path_data[nidx].distance_from_idx == OA_DIJKSTRA_POLYGON_SHORTPATH_NOTSET_IDX) ||
            (_short_path_data[nidx].distance_cm >= FLT_MAX)) {
            break;
        } else {
            // add position to path array
            Vector2f next_pos;
            if (!convert_node_to_point(path, _short_path_data[nidx].id, next_pos)) {
                // this should never happen
                err_id = AP_OADijkstra_Common::ErrorId::DIJKSTRA_ERROR_COULD_NOT_FIND_PATH;
                return false;
            }
            path.pos[path.num_pos] = next_pos;
            path.num_pos++;

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
    // report error in case path not found
    if (!success) {
        err_id = AP_OADijkstra_Common::ErrorId::DIJKSTRA_ERROR_COULD_NOT_FIND_PATH;
    }

    return success;
}

// updates visibility graph for a given position which is an offset (in cm) from the ekf origin
// to add an additional position (i.e. the destination) set add_extra_position = true and provide the position in the extra_position argument
// requires create_inclusion_polygon_with_margin to have been run
// returns true on success
bool AP_OADijkstra_CalcPath::update_visgraph(AP_OAVisGraph& visgraph, const AP_OAVisGraph::OAItemID& oaid, const Vector2f &position, bool add_extra_position, Vector2f extra_position)
{
    // clear visibility graph
    visgraph.clear();

    // calculate distance from position to all inclusion/exclusion fence points
    for (uint8_t i = 0; i < _static_data.total_numpoints(); i++) {
        Vector2f seg_end;
        if (_static_data.get_point(i, seg_end)) {
            if (!_static_data.intersects_fence(position, seg_end)) {
                // line segment does not intersect with fences so add to visgraph
                if (!visgraph.add_item(oaid, {AP_OAVisGraph::OATYPE_INTERMEDIATE_POINT, i}, (position - seg_end).length())) {
                    return false;
                }
            }
        }
    }

    // add extra point to visibility graph if it doesn't intersect with polygon fence or exclusion polygons
    if (add_extra_position) {
        if (!_static_data.intersects_fence(position, extra_position)) {
            if (!visgraph.add_item(oaid, {AP_OAVisGraph::OATYPE_DESTINATION, 0}, (position - extra_position).length())) {
                return false;
            }
        }
    }

    return true;
}

// update total distance for all nodes visible from current node
// curr_node_idx is an index into the _short_path_data array
void AP_OADijkstra_CalcPath::update_visible_node_distances(node_index curr_node_idx)
{
    // sanity check
    if (curr_node_idx >= _short_path_data_numpoints) {
        return;
    }

    // get current node for convenience
    const ShortPathNode &curr_node = _short_path_data[curr_node_idx];

    // for each visibility graph
    const AP_OAVisGraph* visgraphs[] = {&_static_data.get_fence_visgraph(), &_destination_visgraph};
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
bool AP_OADijkstra_CalcPath::find_node_from_id(const AP_OAVisGraph::OAItemID &id, node_index &node_idx) const
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
bool AP_OADijkstra_CalcPath::find_closest_node_idx(const Path& path, node_index &node_idx) const
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
        if (convert_node_to_point(path, node.id, node_pos)) {
            // heuristics is is simple Euclidean distance from the node to the destination
            // This should be admissible, therefore optimal path is guaranteed
            const float heuristics = (node_pos-path.destination).length();
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

// find the position of a node as an offset (in cm) from the ekf origin
bool AP_OADijkstra_CalcPath::convert_node_to_point(const Path& path, const AP_OAVisGraph::OAItemID& id, Vector2f& pos) const
{
    // convert id to a position offset from EKF origin
    switch (id.id_type) {
    case AP_OAVisGraph::OATYPE_SOURCE:
        pos = path.source;
        return true;
    case AP_OAVisGraph::OATYPE_DESTINATION:
        pos = path.destination;
        return true;
    case AP_OAVisGraph::OATYPE_INTERMEDIATE_POINT:
        return _static_data.get_point(id.id_num, pos);
    }

    // we should never reach here but just in case
    return false;
}
#endif // AP_FENCE_ENABLED

#endif  // AP_OAPATHPLANNER_DIJKSTRA_ENABLED
