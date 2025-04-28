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
#if AP_FENCE_ENABLED

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_OADijkstra_Common.h"

#define OA_DIJKSTRA_ERROR_REPORTING_INTERVAL_MS         5000    // failure messages sent to GCS every 5 seconds

// return error message for a given error id
const char* AP_OADijkstra_Common::get_error_msg(ErrorId error_id) const
{
    switch (error_id) {
    case ErrorId::DIJKSTRA_ERROR_NONE:
        return "no error";
        break;
    case ErrorId::DIJKSTRA_ERROR_OUT_OF_MEMORY:
        return "out of memory";
        break;
    case ErrorId::DIJKSTRA_ERROR_OVERLAPPING_POLYGON_POINTS:
        return "overlapping polygon points";
        break;
    case ErrorId::DIJKSTRA_ERROR_FAILED_TO_BUILD_INNER_POLYGON:
        return "failed to build inner polygon";
        break;
    case ErrorId::DIJKSTRA_ERROR_OVERLAPPING_POLYGON_LINES:
        return "overlapping polygon lines";
        break;
    case ErrorId::DIJKSTRA_ERROR_FENCE_DISABLED:
        return "fence disabled";
        break;
    case ErrorId::DIJKSTRA_ERROR_TOO_MANY_FENCE_POINTS:
        return "too many fence points";
        break;
    case ErrorId::DIJKSTRA_ERROR_NO_POSITION_ESTIMATE:
        return "no position estimate";
        break;
    case ErrorId::DIJKSTRA_ERROR_COULD_NOT_FIND_PATH:
        return "could not find path";
        break;
    }

    // we should never reach here but just in case
    return "unknown error";
}

void AP_OADijkstra_Common::report_error(ErrorId error_id)
{
    // report errors to GCS every 5 seconds
    uint32_t now_ms = AP_HAL::millis();
    if ((error_id != ErrorId::DIJKSTRA_ERROR_NONE) &&
        ((error_id != _error_last_id) || ((now_ms - _error_last_report_ms) > OA_DIJKSTRA_ERROR_REPORTING_INTERVAL_MS))) {
        const char* error_msg = get_error_msg(error_id);
        (void)error_msg;  // in case !HAL_GCS_ENABLED
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Dijkstra: %s", error_msg);
        _error_last_id = error_id;
        _error_last_report_ms = now_ms;
    }
}

#endif // AP_FENCE_ENABLED
#endif // AP_OAPATHPLANNER_DIJKSTRA_ENABLED
