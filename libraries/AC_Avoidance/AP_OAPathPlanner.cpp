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

#include "AP_OAPathPlanner.h"
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AC_Fence/AC_Fence.h>
#include <AP_Logger/AP_Logger.h>
#include "AP_OABendyRuler.h"
#include "AP_OADijkstra.h"

extern const AP_HAL::HAL &hal;

// parameter defaults
const float OA_LOOKAHEAD_DEFAULT = 15;
const float OA_MARGIN_MAX_DEFAULT = 5;

const int16_t OA_TIMEOUT_MS = 2000;             // avoidance results over 2 seconds old are ignored

const AP_Param::GroupInfo AP_OAPathPlanner::var_info[] = {

    // @Param: TYPE
    // @DisplayName: Object Avoidance Path Planning algorithm to use
    // @Description: Enabled/disable path planning around obstacles
    // @Values: 0:Disabled,1:BendyRuler,2:Dijkstra
    // @User: Standard
    AP_GROUPINFO_FLAGS("TYPE", 1,  AP_OAPathPlanner, _type, OA_PATHPLAN_DISABLED, AP_PARAM_FLAG_ENABLE),

    // @Param: LOOKAHEAD
    // @DisplayName: Object Avoidance look ahead distance maximum
    // @Description: Object Avoidance will look this many meters ahead of vehicle
    // @Units: m
    // @Range: 1 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("LOOKAHEAD", 2, AP_OAPathPlanner, _lookahead, OA_LOOKAHEAD_DEFAULT),

    // @Param: MARGIN_MAX
    // @DisplayName: Object Avoidance wide margin distance
    // @Description: Object Avoidance will ignore objects more than this many meters from vehicle
    // @Units: m
    // @Range: 0.1 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("MARGIN_MAX", 3, AP_OAPathPlanner, _margin_max, OA_MARGIN_MAX_DEFAULT),

#if !HAL_MINIMIZE_FEATURES
    // @Group: DB_
    // @Path: AP_OADatabase.cpp
    AP_SUBGROUPINFO(_oadatabase, "DB_", 4, AP_OAPathPlanner, AP_OADatabase),
#endif

    AP_GROUPEND
};

/// Constructor
AP_OAPathPlanner::AP_OAPathPlanner()
{
    _singleton = this;

    AP_Param::setup_object_defaults(this, var_info);
}

// perform any required initialisation
void AP_OAPathPlanner::init()
{
    // run background task looking for best alternative destination
    switch (_type) {
    case OA_PATHPLAN_DISABLED:
        // do nothing
        return;
    case OA_PATHPLAN_BENDYRULER:
        if (_oabendyruler == nullptr) {
            _oabendyruler = new AP_OABendyRuler();
        }
        break;
    case OA_PATHPLAN_DIJKSTRA:
        if (_oadijkstra == nullptr) {
            _oadijkstra = new AP_OADijkstra();
        }
        break;
    }

    _oadatabase.init();
    start_thread();
}

// pre-arm checks that algorithms have been initialised successfully
bool AP_OAPathPlanner::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    // check if initialisation has succeeded
    switch (_type) {
    case OA_PATHPLAN_DISABLED:
        // do nothing
        break;
    case OA_PATHPLAN_BENDYRULER:
        if (_oabendyruler == nullptr) {
            hal.util->snprintf(failure_msg, failure_msg_len, "BendyRuler OA requires reboot");
            return false;
        }
        break;
    case OA_PATHPLAN_DIJKSTRA:
        if (_oadijkstra == nullptr) {
            hal.util->snprintf(failure_msg, failure_msg_len, "Dijkstra OA requires reboot");
            return false;
        }
        break;
    }
    return true;
}

bool AP_OAPathPlanner::start_thread()
{
    WITH_SEMAPHORE(_rsem);

    if (_thread_created) {
        return true;
    }
    if (_type == OA_PATHPLAN_DISABLED) {
        return false;
    }

    // create the avoidance thread as low priority. It should soak
    // up spare CPU cycles to fill in the avoidance_result structure based
    // on requests in avoidance_request
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_OAPathPlanner::avoidance_thread, void),
                                      "avoidance",
                                      8192, AP_HAL::Scheduler::PRIORITY_IO, -1)) {
        return false;
    }
    _thread_created = true;
    return true;
}

// provides an alternative target location if path planning around obstacles is required
// returns true and updates result_loc with an intermediate location
AP_OAPathPlanner::OA_RetState AP_OAPathPlanner::mission_avoidance(const Location &current_loc,
                                         const Location &origin,
                                         const Location &destination,
                                         Location &result_origin,
                                         Location &result_destination)
{
    // exit immediately if disabled or thread is not running from a failed init
    if (_type == OA_PATHPLAN_DISABLED || !_thread_created) {
        return OA_NOT_REQUIRED;
    }

    const uint32_t now = AP_HAL::millis();
    WITH_SEMAPHORE(_rsem);

    // place new request for the thread to work on
    avoidance_request.current_loc = current_loc;
    avoidance_request.origin = origin;
    avoidance_request.destination = destination;
    avoidance_request.ground_speed_vec = AP::ahrs().groundspeed_vector();
    avoidance_request.request_time_ms = now;

    // check result's destination matches our request
    const bool destination_matches = (destination.lat == avoidance_result.destination.lat) && (destination.lng == avoidance_result.destination.lng);

    // check results have not timed out
    const bool timed_out = now - avoidance_result.result_time_ms > OA_TIMEOUT_MS;

    // return results from background thread's latest checks
    if (destination_matches && !timed_out) {
        // we have a result from the thread
        result_origin = avoidance_result.origin_new;
        result_destination = avoidance_result.destination_new;
        return avoidance_result.ret_state;
    }

    // if timeout then path planner is taking too long to respond
    if (timed_out) {
        return OA_ERROR;
    }

    // background thread is working on a new destination
    return OA_PROCESSING;
}

// avoidance thread that continually updates the avoidance_result structure based on avoidance_request
void AP_OAPathPlanner::avoidance_thread()
{
    while (true) {
#if !HAL_MINIMIZE_FEATURES

        // if database queue needs attention, service it faster
        if (_oadatabase.process_queue()) {
            hal.scheduler->delay(1);
        } else {
            hal.scheduler->delay(20);
        }

        const uint32_t now = AP_HAL::millis();
        if (now - avoidance_latest_ms < 100) {
            continue;
        }
        avoidance_latest_ms = now;

        _oadatabase.update();
#else
        hal.scheduler->delay(100);
        const uint32_t now = AP_HAL::millis();
#endif

        Location origin_new;
        Location destination_new;
        {
            WITH_SEMAPHORE(_rsem);
            if (now - avoidance_request.request_time_ms > OA_TIMEOUT_MS) {
                // this is a very old request, don't process it
                continue;
            }

            // copy request to avoid conflict with main thread
            avoidance_request2 = avoidance_request;

            // store passed in origin and destination so we can return it if object avoidance is not required
            origin_new = avoidance_request.origin;
            destination_new = avoidance_request.destination;
        }

        // run background task looking for best alternative destination
        OA_RetState res = OA_NOT_REQUIRED;
        switch (_type) {
        case OA_PATHPLAN_DISABLED:
            continue;
        case OA_PATHPLAN_BENDYRULER:
            if (_oabendyruler == nullptr) {
                continue;
            }
            _oabendyruler->set_config(_lookahead, _margin_max);
            if (_oabendyruler->update(avoidance_request2.current_loc, avoidance_request2.destination, avoidance_request2.ground_speed_vec, origin_new, destination_new)) {
                res = OA_SUCCESS;
            }
            break;

        case OA_PATHPLAN_DIJKSTRA:
            if (_oadijkstra == nullptr) {
                continue;
            }
            _oadijkstra->set_fence_margin(_margin_max);
            const AP_OADijkstra::AP_OADijkstra_State dijkstra_state = _oadijkstra->update(avoidance_request2.current_loc, avoidance_request2.destination, origin_new, destination_new);
            switch (dijkstra_state) {
            case AP_OADijkstra::DIJKSTRA_STATE_NOT_REQUIRED:
                res = OA_NOT_REQUIRED;
                break;
            case AP_OADijkstra::DIJKSTRA_STATE_ERROR:
                res = OA_ERROR;
                break;
            case AP_OADijkstra::DIJKSTRA_STATE_SUCCESS:
                res = OA_SUCCESS;
                break;
            }
            break;
        }

        {
            // give the main thread the avoidance result
            WITH_SEMAPHORE(_rsem);
            avoidance_result.destination = avoidance_request2.destination;
            avoidance_result.origin_new = (res == OA_SUCCESS) ? origin_new : avoidance_result.origin_new;
            avoidance_result.destination_new = (res == OA_SUCCESS) ? destination_new : avoidance_result.destination;
            avoidance_result.result_time_ms = AP_HAL::millis();
            avoidance_result.ret_state = res;
        }
    }
}

// singleton instance
AP_OAPathPlanner *AP_OAPathPlanner::_singleton;

namespace AP {

AP_OAPathPlanner *ap_oapathplanner()
{
    return AP_OAPathPlanner::get_singleton();
}

}
