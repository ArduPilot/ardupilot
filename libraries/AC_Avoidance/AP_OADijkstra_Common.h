#pragma once

#include "AC_Avoidance_config.h"

#if AP_OAPATHPLANNER_DIJKSTRA_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>

/*
 * Dijkstra's algorithm for path planning around polygon fence
 */

class AP_OADijkstra_Common {
public:

    // constructor
    AP_OADijkstra_Common() {}

    CLASS_NO_COPY(AP_OADijkstra_Common);  /* Do not allow copies */

    // update return status enum
    enum class State : uint8_t {
        DIJKSTRA_STATE_NOT_REQUIRED = 0,
        DIJKSTRA_STATE_ERROR,
        DIJKSTRA_STATE_SUCCESS
    };

    enum class ErrorId : uint8_t {
        DIJKSTRA_ERROR_NONE = 0,
        DIJKSTRA_ERROR_OUT_OF_MEMORY,
        DIJKSTRA_ERROR_OVERLAPPING_POLYGON_POINTS,
        DIJKSTRA_ERROR_FAILED_TO_BUILD_INNER_POLYGON,
        DIJKSTRA_ERROR_OVERLAPPING_POLYGON_LINES,
        DIJKSTRA_ERROR_FENCE_DISABLED,
        DIJKSTRA_ERROR_TOO_MANY_FENCE_POINTS,
        DIJKSTRA_ERROR_NO_POSITION_ESTIMATE,
        DIJKSTRA_ERROR_COULD_NOT_FIND_PATH
    };

    // return error message for a given error id
    const char* get_error_msg(ErrorId error_id) const;

    // report error to ground station
    void report_error(ErrorId error_id);

    private:

    ErrorId _error_last_id;         // last error id sent to GCS
    uint32_t _error_last_report_ms; // last time an error message was sent to GCS
};

#endif  // AP_OAPATHPLANNER_DIJKSTRA_ENABLED
