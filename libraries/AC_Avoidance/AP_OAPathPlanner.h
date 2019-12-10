#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Param/AP_Param.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_OABendyRuler.h"
#include "AP_OADijkstra.h"
#include "AP_OADatabase.h"

/*
 * This class provides path planning around fence, stay-out zones and moving obstacles
 */
class AP_OAPathPlanner {

public:
    AP_OAPathPlanner();

    /* Do not allow copies */
    AP_OAPathPlanner(const AP_OAPathPlanner &other) = delete;
    AP_OAPathPlanner &operator=(const AP_OAPathPlanner&) = delete;

    // get singleton instance
    static AP_OAPathPlanner *get_singleton() {
        return _singleton;
    }

    // perform any required initialisation
    void init();

    /// returns true if all pre-takeoff checks have completed successfully
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const;

    // object avoidance processing return status enum
    enum OA_RetState : uint8_t {
        OA_NOT_REQUIRED = 0,            // object avoidance is not required
        OA_PROCESSING,                  // still calculating alternative path
        OA_ERROR,                       // error during calculation
        OA_SUCCESS                      // success
    };

    // provides an alternative target location if path planning around obstacles is required
    // returns true and updates result_origin and result_destination with an intermediate path
    OA_RetState mission_avoidance(const Location &current_loc,
                           const Location &origin,
                           const Location &destination,
                           Location &result_origin,
                           Location &result_destination) WARN_IF_UNUSED;

    // enumerations for _TYPE parameter
    enum OAPathPlanTypes {
        OA_PATHPLAN_DISABLED = 0,
        OA_PATHPLAN_BENDYRULER = 1,
        OA_PATHPLAN_DIJKSTRA = 2
    };

    static const struct AP_Param::GroupInfo var_info[];

private:

    // avoidance thread that continually updates the avoidance_result structure based on avoidance_request
    void avoidance_thread();
    bool start_thread();

    // an avoidance request from the navigation code
    struct avoidance_info {
        Location current_loc;
        Location origin;
        Location destination;
        Vector2f ground_speed_vec;
        uint32_t request_time_ms;
    } avoidance_request, avoidance_request2;

    // an avoidance result from the avoidance thread
    struct {
        Location destination;       // destination vehicle is trying to get to (also used to verify the result matches a recent request)
        Location origin_new;        // intermediate origin.  The start of line segment that vehicle should follow
        Location destination_new;   // intermediate destination vehicle should move towards
        uint32_t result_time_ms;    // system time the result was calculated (used to verify the result is recent)
        OA_RetState ret_state;      // OA_SUCCESS if the vehicle should move along the path from origin_new to destination_new
    } avoidance_result;

    // parameters
    AP_Int8 _type;                  // avoidance algorith to be used
    AP_Float _lookahead;            // object avoidance will look this many meters ahead of vehicle
    AP_Float _margin_max;           // object avoidance will ignore objects more than this many meters from vehicle

    // internal variables used by front end
    HAL_Semaphore_Recursive _rsem;  // semaphore for multi-thread use of avoidance_request and avoidance_result
    bool _thread_created;           // true once background thread has been created
    AP_OABendyRuler *_oabendyruler; // Bendy Ruler algorithm
    AP_OADijkstra *_oadijkstra;     // Dijkstra's algorithm
    AP_OADatabase _oadatabase;      // Database of dynamic objects to avoid
    uint32_t avoidance_latest_ms;   // last time Dijkstra's or BendyRuler algorithms ran

    static AP_OAPathPlanner *_singleton;
};

namespace AP {
    AP_OAPathPlanner *ap_oapathplanner();
};
