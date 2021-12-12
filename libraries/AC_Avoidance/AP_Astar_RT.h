/*
This library calculates safe path around proximity based obstacles using A-Star in realtime. The summary of work flow is given below.
1. Obstacles are read as "spheres" from the obstacle database (see function "create_oa_database_nodes")
2. Closely located obstacles are grouped together to form one bigger obstacle. This is done using a graph implementation, where each each obstacle is a member of a graph, and to find close obstacles, we just find connected items of the graph using Depth First Search.
    The grouping and DFS is done in "group_oa_items" function.
3. Nodes are constructed from each of the grouped obstacles. In the current implementation, nodes are projected in 8 separate directions proportional to the required safe distance.
4. For grouped obstacles, nodes are placed in 8 extremeties of the larger figure formed. See "expand_nodes" function
5. Now that nodes are constructed, visability graph is made from these nodes. This is a naive implementation, which is highly inefficent and should be improved in the future.
6. A-Star is run on the constructed visiblity graph. See "find_shortest_path" function
 */

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_OAVisGraph.h"
#include <AC_Avoidance/AP_OADatabase.h>
#include "AP_OAGraph.h"

#define OA_RTASTAR_NUM_POINTS_PER_OBJECT 8
#define OA_RTASTAR_DEBUG_ENABLE 0

/*
 * Real Time A* avoidance algorithm for avoiding the polygon and circular fence and dynamic objects detected by the proximity sensor
 */
class AP_OART_AStar {
public:
    AP_OART_AStar();

    /* Do not allow copies */
    AP_OART_AStar(const AP_OART_AStar &other) = delete;
    AP_OART_AStar &operator=(const AP_OART_AStar&) = delete;

    // send configuration info stored in front end parameters
    void set_config(float margin_max) { _margin_max_cm = MAX(margin_max, 0.0f)*100.0f;  }

    // update return status enum
    enum AP_OART_Astar_State : uint8_t {
        RT_ASTAR_STATE_NOT_REQUIRED = 0,
        RT_ASTAR_STATE_ERROR,
        RT_ASTAR_STATE_SUCCESS
    };

    // run background task to find best path and update avoidance_results
    // returns true and populates origin_new and destination_new if OA is required.  returns false if OA is not required
    AP_OART_Astar_State update(const Location& current_loc, const Location& destination, const Vector2f &ground_speed_vec, Location &origin_new, Location &destination_new, bool proximity_only);

    // get singleton instance
    static AP_OART_AStar *get_singleton() {
        return _singleton;
    }

#if OA_RTASTAR_DEBUG_ENABLE
    void send_debug_info(mavlink_channel_t chan, uint16_t interval_ms);
#endif

private:

    // various types of errors that can occur
    enum class AP_OA_RTAStar_Error : uint8_t {
        RT_ASTAR_ERROR_NONE = 0,             // No error
        RT_ASTAR_ERROR_OUT_OF_MEMORY,        // Out of memory (mostly happens when arrays can't be expanded)
        RT_ASTAR_ERROR_DB_ERROR,             // Some error with OA DB
        RT_ASTAR_ERROR_TOO_MANY_POINTS,      // Too many OA DB obstacles
        RT_ASTAR_ERROR_NO_POSITION_ESTIMATE, // No position estimate
        RT_ASTAR_ERROR_TIMEOUT,              // Can't find path in a reasonable time
        RT_ASTAR_ERROR_COULD_NOT_FIND_PATH   // Can't find a safe path at all
    };

    // create nodes from OA DB obstacles. This will attempt to group closely located obstacles.  Returns false on error or DB is empty
    bool create_oa_database_nodes();

    // group closesly packed obstacles together. Returns false on error
    bool group_oa_items(Graph &grouping_graph);

    // do depth first search to figure out connected obstacles. This is a recursive function. Returns false on error
    bool find_connected_obstacles(uint16_t &iteration, uint16_t start, Vector3f grouped_nodes[OA_RTASTAR_NUM_POINTS_PER_OBJECT], Graph &grouping_graph, bool visited[]);

    // create nodes from obstacles
    void create_nodes_from_objects(const AP_OADatabase::OA_DbItem& item, Vector3f grouped_nodes[OA_RTASTAR_NUM_POINTS_PER_OBJECT]);

    // Expand current set of nodes to a bigger set
    void expand_nodes(Vector3f new_nodes[OA_RTASTAR_NUM_POINTS_PER_OBJECT], Vector3f grouped_nodes[OA_RTASTAR_NUM_POINTS_PER_OBJECT]);

    // create visibility graph from OA DB nodes
    bool create_oa_db_visgraph();

    // returns true if line segment between seg_start and seg_ends is close to an obstacle
    bool intersects_proximity_obstacle(const Vector3f &seg_start, const Vector3f &seg_end) const;

    //  Creates visibility graph for goal and destination. Returns false on error
    bool create_goal_and_destination_visgraph(const Vector3f &origin, const Vector3f &destination);

    // Create visiblity graph from a point. Returns false on error
    bool create_visgraph_from_point(AP_OAVisGraph& visgraph, const AP_OAVisGraph::OAItemID& oaid, const Vector3f &position, bool add_extra_position = false, Vector3f extra_position = Vector3f(0,0,0));

    // Find shortest path via A-Star Search. Returns false on error
    bool find_shortest_path();

    // Log status
    void Write_RTAstar(const AP_OART_Astar_State& state, const AP_OA_RTAStar_Error& error_id, const Location &next_loc, uint64_t nodes_time, uint64_t visgraph_time, uint64_t path_time, uint64_t total_time);

    typedef uint16_t node_index;         // indices into short path data
    struct ShortPathNode {
        AP_OAVisGraph::OAItemID id;     // unique id for node (combination of type and id number)
        bool visited;                   // true if all this node's neighbour's distances have been updated
        node_index distance_from_idx;   // index into _short_path_data from where distance was updated (or 255 if not set)
        float distance_cm;              // distance from source (number is tentative until this node is the current node and/or visited = true)
    };

    // find a node's index into _short_path_data array from it's id (i.e. id type and id number)
    // returns true if successful and node_idx is updated
    bool find_node_from_id(const AP_OAVisGraph::OAItemID &id, node_index &node_idx) const;

    // find index of node with lowest tentative distance (ignore visited nodes)
    // returns true if successful and node_idx argument is updated
    bool find_closest_node_idx(node_index &node_idx) const;

    // find the position of a node as an offset (in cm) from the ekf origin
    // returns true if successful and pos is updated
    bool convert_node_to_point(const AP_OAVisGraph::OAItemID& id, Vector3f& pos) const;

    // update total distance for all nodes visible from current node
    // curr_node_idx is an index into the _short_path_data array
    void update_visible_node_distances(node_index curr_node_idx);

    // return point from final path as an offset (in cm) from the ekf origin
    bool get_shortest_path_point(uint16_t point_num, Vector3f& pos);

    AP_OAVisGraph _oa_db_visgraph;          // holds distances from all OA DB points to each other
    AP_OAVisGraph _source_visgraph;         // holds distances from source point to all other nodes
    AP_OAVisGraph _destination_visgraph;    // holds distances from the destination to all other nodes

    Vector3f _path_destination;  // Current goal of path
    Vector3f _path_source;       // Current source of path

    uint16_t _path_numpoints; // number of points in the _path array
    AP_ExpandingArray<AP_OAVisGraph::OAItemID> _path;   // ids of points on return path in reverse order (i.e. destination is first element)

    Vector3f unit_offsets[OA_RTASTAR_NUM_POINTS_PER_OBJECT];  // offset directions when nodes are created from obstacles

    float _margin_max_cm;    // margin to be maintained from obstacles

    AP_ExpandingArray<ShortPathNode> _short_path_data;
    node_index _short_path_data_numpoints;  // number of elements in _short_path_data array

    AP_ExpandingArray<Vector3f> oa_db_nodes; // array of nodes corresponding to inclusion polygon points plus a margin
    uint16_t _total_oa_db_numpoints;   // number of points held in above array
    uint32_t _oa_db_nodes_update_ms;  // system time of boundary update from AC_Fence (used to detect changes to polygon fence)

    AP_OA_RTAStar_Error _error_last_id;  // Latest error

    static AP_OART_AStar *_singleton;
};

namespace AP {
    AP_OART_AStar *oa_astarRT();
};
