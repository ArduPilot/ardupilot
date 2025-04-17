#pragma once

#include "AC_Avoidance_config.h"

#if AP_OAPATHPLANNER_DIJKSTRA_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>
#include "AP_OADijkstra_Common.h"
#include "AP_OADijkstra_StaticData.h"
#include "AP_OADijkstra_CalcPath.h"
#include <AP_Logger/AP_Logger_config.h>

/*
 * Dijkstra's algorithm for path planning around polygon fence
 */

class AP_OADijkstra {
public:

    AP_OADijkstra(const AP_Int16 &options);

    CLASS_NO_COPY(AP_OADijkstra);  /* Do not allow copies */

    // set fence margin (in meters) used when creating "safe positions" within the polygon fence
    void set_fence_margin(float margin) { _static_data.set_fence_margin(margin); }

    // trigger Dijkstra's to recalculate shortest path based on current location 
    void recalculate_path() { _shortest_path_ok = false; }

    // update return status enum
    enum AP_OADijkstra_State : uint8_t {
        DIJKSTRA_STATE_NOT_REQUIRED = 0,
        DIJKSTRA_STATE_ERROR,
        DIJKSTRA_STATE_SUCCESS
    };

    // calculate a destination to avoid the polygon fence
    // returns DIJKSTRA_STATE_SUCCESS and populates origin_new, destination_new and next_destination_new if avoidance is required
    // next_destination_new will be non-zero if there is a next destination
    // dest_to_next_dest_clear will be set to true if the path from (the input) destination to (input) next_destination is clear
    AP_OADijkstra_State update(const Location &current_loc,
                               const Location &destination,
                               const Location &next_destination,
                               Location& origin_new,
                               Location& destination_new,
                               Location& next_destination_new,
                               bool& dest_to_next_dest_clear);

    // return the path length in meters
    float get_path_length() const { return _shortest_path_ok ? _shortest_path.length_cm * 0.01 : 0; }

    // calculate the length of a path between origin and destination in meters
    // this calculation takes time and should only be run from a background thread
    // returns true on success and fills in path_length argument
    // called by AP_OAPathPlanner::get_path_length()
    AP_OADijkstra_State get_path_length(const Location &origin, const Location& destination, float& path_length);

private:

    bool _shortest_path_ok;                         // shortest path is up-to-date
    AP_OADijkstra_CalcPath::Path _shortest_path;    // shortest path from source to destination
    AP_OADijkstra_CalcPath::Path _secondary_path;   // secondary path used to reply to get_path_length requests

    Location _destination_prev;     // destination of previous iterations (used to determine if path should be re-calculated)
    Location _next_destination_prev;// next_destination of previous iterations (used to determine if path should be re-calculated)
    uint8_t _path_idx_returned;     // index into _path array which gives location vehicle should be currently moving towards
    bool _dest_to_next_dest_clear;  // true if path from dest to next_dest is clear (i.e. does not intersects a fence)

#if HAL_LOGGING_ENABLED
    // Logging functions
    void Write_OADijkstra(const uint8_t state, const uint8_t error_id, const uint8_t curr_point, const uint8_t tot_points, const Location &final_dest, const Location &oa_dest) const;
#else
    void Write_OADijkstra(const uint8_t state, const uint8_t error_id, const uint8_t curr_point, const uint8_t tot_points, const Location &final_dest, const Location &oa_dest) const {}
#endif

    // reference to AP_OAPathPlanner options param, static data and calc path
    const AP_Int16 &_options;
    AP_OADijkstra_Common _common;
    AP_OADijkstra_StaticData _static_data;
    AP_OADijkstra_CalcPath _calcpath;
};

#endif  // AP_OAPATHPLANNER_DIJKSTRA_ENABLED
