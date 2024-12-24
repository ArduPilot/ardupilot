#pragma once

#include "AC_Avoidance_config.h"

#if AP_OAPATHPLANNER_DIJKSTRA_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>
#include "AP_OADijkstra_Common.h"
#include "AP_OAVisGraph.h"
#include <AP_Logger/AP_Logger_config.h>

/*
 * Dijkstra's algorithm's static data handler (e.g fences)
 */

class AP_OADijkstra_StaticData {
public:

    AP_OADijkstra_StaticData(AP_OADijkstra_Common& common, const AP_Int16 &options);

    CLASS_NO_COPY(AP_OADijkstra_StaticData);  /* Do not allow copies */

    // returns true if at least one inclusion or exclusion zone is enabled
    bool some_fences_enabled() const;

    // set fence margin (in meters) used when creating "safe positions" within the polygon fence
    void set_fence_margin(float margin) { _polyfence_margin = MAX(margin, 0.0f); }

    // updates static data required for Dijkstra's algorithm
    // returns READY on success, UPDATED if change in static data since previous call means path should be re-calculated
    enum class UpdateState : uint8_t {
        NOT_REQUIRED = 0,
        READY,
        UPDATED,
        ERROR
    };
    UpdateState update(AP_OADijkstra_Common::ErrorId& error_id);

    // returns total number of points across all fence types
    uint16_t total_numpoints() const;

    // get a single point across the total list of points from all fence types
    // also returns the type of point
    bool get_point(uint16_t index, Vector2f& point) const;

    // returns true if line segment intersects polygon or circular fence
    bool intersects_fence(const Vector2f &seg_start, const Vector2f &seg_end) const;

    // return fence visibility graph
    const AP_OAVisGraph& get_fence_visgraph() const { return _fence_visgraph; }

private:

    //
    // inclusion polygon methods
    //

    // check if inclusion polygons have been updated since create_inclusion_polygon_with_margin was run
    // returns true if changed
    bool check_inclusion_polygon_updated() const;

    // create polygons inside the existing inclusion polygons
    // returns true on success.  returns false on failure and err_id is updated
    bool create_inclusion_polygon_with_margin(float margin_cm, AP_OADijkstra_Common::ErrorId &err_id);

    //
    // exclusion polygon methods
    //

    // check if exclusion polygons have been updated since create_exclusion_polygon_with_margin was run
    // returns true if changed
    bool check_exclusion_polygon_updated() const;

    // create polygons around existing exclusion polygons
    // returns true on success.  returns false on failure and err_id is updated
    bool create_exclusion_polygon_with_margin(float margin_cm, AP_OADijkstra_Common::ErrorId &err_id);

    //
    // exclusion circle methods
    //

    // check if exclusion circles have been updated since create_exclusion_circle_with_margin was run
    // returns true if changed
    bool check_exclusion_circle_updated() const;

    // create polygons around existing exclusion circles
    // returns true on success.  returns false on failure and err_id is updated
    bool create_exclusion_circle_with_margin(float margin_cm, AP_OADijkstra_Common::ErrorId &err_id);

    //
    // other methods
    //

    // create visibility graph for all fence (with margin) points
    // returns true on success.  returns false on failure and err_id is updated
    bool create_fence_visgraph(AP_OADijkstra_Common::ErrorId &err_id);

    // shortest path state variables
    bool _polyfence_visgraph_ok;

    // inclusion polygon (with margin) related variables
    float _polyfence_margin = 10;           // margin around polygon defaults to 10m but is overriden with set_fence_margin
    AP_ExpandingArray<Vector2f> _inclusion_polygon_pts; // array of nodes corresponding to inclusion polygon points plus a margin
    uint8_t _inclusion_polygon_numpoints;   // number of points held in above array
    uint32_t _inclusion_polygon_update_ms;  // system time of boundary update from AC_Fence (used to detect changes to polygon fence)

    // exclusion polygon related variables
    AP_ExpandingArray<Vector2f> _exclusion_polygon_pts; // array of nodes corresponding to exclusion polygon points plus a margin
    uint8_t _exclusion_polygon_numpoints;   // number of points held in above array
    uint32_t _exclusion_polygon_update_ms;  // system time exclusion polygon was updated (used to detect changes)

    // exclusion circle related variables
    AP_ExpandingArray<Vector2f> _exclusion_circle_pts; // array of nodes surrounding exclusion circles plus a margin
    uint8_t _exclusion_circle_numpoints;    // number of points held in above array
    uint32_t _exclusion_circle_update_ms;   // system time exclusion circles were updated (used to detect changes)

    // visibility graphs
    AP_OAVisGraph _fence_visgraph;          // holds distances between all inclusion/exclusion fence points (with margin)

#if HAL_LOGGING_ENABLED
    // Logging functions
    void Write_Visgraph_point(const uint8_t version, const uint8_t point_num, const int32_t Lat, const int32_t Lon) const;
#else
    void Write_Visgraph_point(const uint8_t version, const uint8_t point_num, const int32_t Lat, const int32_t Lon) const {}
#endif
    uint8_t _log_num_points;
    uint8_t _log_visgraph_version;

    // references
    AP_OADijkstra_Common& _common;
    const AP_Int16 &_options;
};

#endif  // AP_OAPATHPLANNER_DIJKSTRA_ENABLED
