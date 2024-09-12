#pragma once

#include "AC_Avoidance_config.h"

#if AP_OAPATHPLANNER_DIJKSTRA_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>
#include "AP_OADijkstra_Common.h"
#include "AP_OAVisGraph.h"
#include "AP_OADijkstra_StaticData.h"
#include <AP_Logger/AP_Logger_config.h>

/*
 * Dijkstra's algorithm for path planning around polygon fence
 */

class AP_OADijkstra_CalcPath {
public:

    // constructor
    AP_OADijkstra_CalcPath(const AP_OADijkstra_StaticData& static_data);

    CLASS_NO_COPY(AP_OADijkstra_CalcPath);  /* Do not allow copies */

    // path structure
    class Path {
        public:
        // constuctor
        Path();

        CLASS_NO_COPY(Path);                 /* Do not allow copies */
        uint8_t num_pos;                    // number of position on return path
        Vector2f source;                    // source point used in shortest path calculations (offset in cm from EKF origin)
        Vector2f destination;               // destination position used in shortest path calculations (offset in cm from EKF origin)
        float length_cm;                    // length of path to destination in cm
        AP_ExpandingArray<Vector2f> pos;    // positions on return path in reverse order (i.e. destination is first element)

        // return a position from the path as an offset (in cm) from the ekf origin
        // index 0 is the first point (e.g the source), index of nums_pos-1 is the destination
        // returns true on success and fills in the pos_ne argument
        bool get_position(uint8_t index, Vector2f& pos_ne) const;
    };

    // calculate shortest path from origin to destination
    // returns true on success.  returns false on failure and err_id is updated
    // requires create_polygon_fence_with_margin and create_polygon_fence_visgraph to have been run
    // path argument is updated with resulting path, an array of vector offsets from EKF origin
    bool calc_shortest_path(const Location &origin, const Location &destination, Path& path, AP_OADijkstra_Common::ErrorId &err_id);

private:

    //
    // other methods
    //

    // visibility graphs
    AP_OAVisGraph _source_visgraph;         // holds distances from source point to all other nodes
    AP_OAVisGraph _destination_visgraph;    // holds distances from the destination to all other nodes

    // updates visibility graph for a given position which is an offset (in cm) from the ekf origin
    // to add an additional position (i.e. the destination) set add_extra_position = true and provide the position in the extra_position argument
    // requires create_polygon_fence_with_margin to have been run
    // returns true on success
    bool update_visgraph(AP_OAVisGraph& visgraph, const AP_OAVisGraph::OAItemID& oaid, const Vector2f &position, bool add_extra_position = false, Vector2f extra_position = Vector2f(0,0));

    typedef uint8_t node_index;         // indices into short path data
    struct ShortPathNode {
        AP_OAVisGraph::OAItemID id;     // unique id for node (combination of type and id number)
        bool visited;                   // true if all this node's neighbour's distances have been updated
        node_index distance_from_idx;   // index into _short_path_data from where distance was updated (or 255 if not set)
        float distance_cm;              // distance from source (number is tentative until this node is the current node and/or visited = true)
    };
    AP_ExpandingArray<ShortPathNode> _short_path_data;
    node_index _short_path_data_numpoints;  // number of elements in _short_path_data array

    // update total distance for all nodes visible from current node
    // curr_node_idx is an index into the _short_path_data array
    void update_visible_node_distances(node_index curr_node_idx);

    // find a node's index into _short_path_data array from it's id (i.e. id type and id number)
    // returns true if successful and node_idx is updated
    bool find_node_from_id(const AP_OAVisGraph::OAItemID &id, node_index &node_idx) const;

    // find index of node with lowest tentative distance (ignore visited nodes)
    // path is required because destination is used
    // returns true if successful and node_idx argument is updated
    bool find_closest_node_idx(const Path& path, node_index &node_idx) const;

    // find the position of a node as an offset (in cm) from the ekf origin
    // path is required because source or destination may be returned
    // returns true if successful and pos is updated
    bool convert_node_to_point(const Path& path, const AP_OAVisGraph::OAItemID& id, Vector2f& pos) const;

    // references
    const AP_OADijkstra_StaticData& _static_data;
};

#endif  // AP_OAPATHPLANNER_DIJKSTRA_ENABLED
