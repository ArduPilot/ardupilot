#include "AP_Rally_config.h"

#if HAL_RALLY_ENABLED

#include "AP_Rally.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>
#include <StorageManager/StorageManager.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AC_Avoidance/AC_Avoidance_config.h>
#include <AC_Avoidance/AP_OAPathPlanner.h>
#include <AP_InternalError/AP_InternalError.h>
#include <GCS_MAVLink/GCS.h>

// storage object
StorageAccess AP_Rally::_storage(StorageManager::StorageRally);

#if APM_BUILD_COPTER_OR_HELI
  #define RALLY_LIMIT_KM_DEFAULT 0.3f
  #define RALLY_INCLUDE_HOME_DEFAULT 1
#elif APM_BUILD_TYPE(APM_BUILD_ArduPlane)
  #define RALLY_LIMIT_KM_DEFAULT 5.0f
  #define RALLY_INCLUDE_HOME_DEFAULT 0
#elif APM_BUILD_TYPE(APM_BUILD_Rover)
  #define RALLY_LIMIT_KM_DEFAULT 0.5f
  #define RALLY_INCLUDE_HOME_DEFAULT 1
#else
  #define RALLY_LIMIT_KM_DEFAULT 1.0f
  #define RALLY_INCLUDE_HOME_DEFAULT 0
#endif

const AP_Param::GroupInfo AP_Rally::var_info[] = {
    // @Param: TOTAL
    // @DisplayName: Rally Total
    // @Description: Number of rally points currently loaded
    // @User: Advanced
    AP_GROUPINFO("TOTAL", 0, AP_Rally, _rally_point_total_count, 0),

    // @Param: LIMIT_KM
    // @DisplayName: Rally Limit
    // @Description: Maximum distance to rally point. If the closest rally point is more than this number of kilometers from the current position and the home location is closer than any of the rally points from the current position then do RTL to home rather than to the closest rally point. This prevents a leftover rally point from a different airfield being used accidentally. If this is set to 0 then the closest rally point is always used.
    // @User: Advanced
    // @Units: km
    // @Increment: 0.1
    AP_GROUPINFO("LIMIT_KM", 1, AP_Rally, _rally_limit_km, RALLY_LIMIT_KM_DEFAULT),

    // @Param: INCL_HOME
    // @DisplayName: Rally Include Home
    // @Description: Controls if Home is included as a Rally point (i.e. as a safe landing place) for RTL
    // @User: Standard
    // @Values: 0:DoNotIncludeHome,1:IncludeHome
    AP_GROUPINFO("INCL_HOME", 2, AP_Rally, _rally_incl_home, RALLY_INCLUDE_HOME_DEFAULT),

    // @Param: OPTIONS
    // @DisplayName: Rally Options
    // @Description: Options to Rally point usage
    // @User: Standard
    // @Values: 0:Use Dijkstras to decide closest point
    AP_GROUPINFO("OPTIONS", 3, AP_Rally, _options, 0),

    AP_GROUPEND
};

// constructor
AP_Rally::AP_Rally()
{
    ASSERT_STORAGE_SIZE(RallyLocation, 15);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("Rally must be singleton");
    }
#endif
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

// get a rally point from EEPROM
bool AP_Rally::get_rally_point_with_index(uint8_t i, RallyLocation &ret) const
{
    if (i >= (uint8_t) _rally_point_total_count) {
        return false;
    }

    _storage.read_block(&ret, i * sizeof(RallyLocation), sizeof(RallyLocation));

    if (ret.lat == 0 && ret.lng == 0) {
        return false; // sanity check
    }

    return true; 
}

void AP_Rally::truncate(uint8_t num)
{
    if (num > _rally_point_total_count) {
        // we never make the space larger this way
        return;
    }
    _rally_point_total_count.set_and_save_ifchanged(num);
}

bool AP_Rally::append(const RallyLocation &loc)
{
    const uint8_t current_total = get_rally_total();
    _rally_point_total_count.set_and_save_ifchanged(current_total + 1);
    if (!set_rally_point_with_index(current_total, loc)) {
        _rally_point_total_count.set_and_save_ifchanged(current_total);
        return false;
    }
    return true;
}

// save a rally point to EEPROM - this assumes that the RALLY_TOTAL param has been incremented beforehand, which is the case in Mission Planner
bool AP_Rally::set_rally_point_with_index(uint8_t i, const RallyLocation &rallyLoc)
{
    if (i >= (uint8_t) _rally_point_total_count) {
        return false;
    }

    if (i >= get_rally_max()) {
        return false;
    }

    _storage.write_block(i * sizeof(RallyLocation), &rallyLoc, sizeof(RallyLocation));

    _last_change_time_ms = AP_HAL::millis();

#if HAL_LOGGING_ENABLED
    AP::logger().Write_RallyPoint(_rally_point_total_count, i, rallyLoc);
#endif

    return true;
}

// helper function to translate a RallyLocation to a Location
Location AP_Rally::rally_location_to_location(const RallyLocation &rally_loc) const
{
    //Relative altitudes are relative to HOME point's altitude:
    Location ret {
        rally_loc.lat,
        rally_loc.lng,
        rally_loc.alt * 100,
        (rally_loc.alt_frame_valid == 1) ? Location::AltFrame(rally_loc.alt_frame) : Location::AltFrame::ABOVE_HOME
    };

    return ret;
}

// returns true if a valid rally point is found, otherwise returns false to indicate home position should be used
bool AP_Rally::find_nearest_rally_point(const Location &current_loc, RallyLocation &return_loc) const
{
    float min_dis = -1;

    for (uint8_t i = 0; i < (uint8_t) _rally_point_total_count; i++) {
        RallyLocation next_rally;
        if (!get_rally_point_with_index(i, next_rally)) {
            continue;
        }
        Location rally_loc = rally_location_to_location(next_rally);
        float dis = current_loc.get_distance(rally_loc);

        if (is_valid(rally_loc) && (dis < min_dis || min_dis < 0)) {
            min_dis = dis;
            return_loc = next_rally;
        }
    }

    // if a limit is defined and all rally points are beyond that limit, use home if it is closer
    if ((_rally_limit_km > 0) && (min_dis > _rally_limit_km*1000.0f)) {
        return false; // use home position
    }

    // use home if no rally points found
    return min_dis >= 0;
}

// return best RTL location from current position
Location AP_Rally::calc_best_rally_or_home_location(const Location &current_loc, float rtl_home_alt_amsl_cm) const
{
    // if no valid rally point, return home position:
    Location return_loc { AP::ahrs().get_home() };
    return_loc.set_alt_cm(rtl_home_alt_amsl_cm, Location::AltFrame::ABSOLUTE);

    RallyLocation ral_loc;
    if (find_nearest_rally_point(current_loc, ral_loc)) {
        Location loc = rally_location_to_location(ral_loc);
        // use the rally point if it's closer then home, or we aren't generally considering home as acceptable
        if (!_rally_incl_home  || (current_loc.get_distance(loc) < current_loc.get_distance(return_loc))) {
            return_loc = rally_location_to_location(ral_loc);
        }
    }

    return return_loc;
}

// find rally point or home with shortest flight path distance calculated using Dijkstras
// returns true on completion and fills in ret with the Location of the closest rally point or home
// returns false if the calculation has not yet completed
// should be called continuously until it returns true.  only one caller is supported at a time
// if dijkstras is disabled or "Use Dikstras" options bit is not set then it will fall back to calc_best_rally_or_home_location()
bool AP_Rally::find_nearest_rally_or_home_with_dijkstras(const Location &current_loc, float rtl_home_alt_amsl_cm, Location& ret)
{
#if AP_OAPATHPLANNER_ENABLED
    // get oa singleton
    AP_OAPathPlanner *oa_ptr = AP_OAPathPlanner::get_singleton();
    
    // fallback to calc_best_rally_or_home_location if dijkstras is disabled or not rally points
    if (!option_is_set(Options::USE_DIJKSTRAS) || (oa_ptr == nullptr) || (get_rally_total() == 0)) {
        ret = calc_best_rally_or_home_location(current_loc, rtl_home_alt_amsl_cm);
        return true;
    }

    // initialise state
    if (_find_with_dijkstras.state != FindWithDijkstras::State::PROCESSING) {
        _find_with_dijkstras.state = FindWithDijkstras::State::PROCESSING;
        _find_with_dijkstras.shortest_path_valid = false;
        _find_with_dijkstras.rally_index = 0;
    }

    // get destination location (e.g. home or next rally point)
    Location search_loc;
    bool searching_for_path_home = false;
    if (_find_with_dijkstras.rally_index < get_rally_total()) {
        // calculating path length to a rally point
        RallyLocation rallyloc;
        if (!get_rally_point_with_index(_find_with_dijkstras.rally_index, rallyloc)) {
            // unexpected failure to get rally point, return home location
            // this should never happen
            _find_with_dijkstras.state = FindWithDijkstras::State::COMPLETED;
            ret = calc_best_rally_or_home_location(current_loc, rtl_home_alt_amsl_cm);
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            return true;
        }

        // convert rally location to Location
        search_loc = rally_location_to_location(rallyloc);
    } else {
        // calculating path length to home
        search_loc = AP::ahrs().get_home();
        searching_for_path_home = true;
    }

    // get path length to search_loc
    float path_length = 0;
    AP_OAPathPlanner::OA_RetState oa_ret = oa_ptr->get_path_length(current_loc, search_loc, path_length);

    // check response
    switch (oa_ret) {
    case AP_OAPathPlanner::OA_NOT_REQUIRED:
    case AP_OAPathPlanner::OA_ERROR:
        // object avoidance is not required or unrecoverable error during calculation
        if (!searching_for_path_home) {
            // display warning message to user and advance to next rally
            if (oa_ret == AP_OAPathPlanner::OA_ERROR) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Rally: failed to calc path to rally point %u", (unsigned)_find_with_dijkstras.rally_index + 1);
            }
            _find_with_dijkstras.rally_index++;
            return false;
        }
        // display warning message to user, complete calculation and break to send results to user
        if (oa_ret == AP_OAPathPlanner::OA_ERROR) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Rally: failed to calc path home");
        }
        break;
    case AP_OAPathPlanner::OA_PROCESSING:
        // still calculating path length
        return false;
    case AP_OAPathPlanner::OA_SUCCESS:
        // path length calculated, update shortest path
        if (!_find_with_dijkstras.shortest_path_valid || path_length < _find_with_dijkstras.shortest_path_length) {
            _find_with_dijkstras.shortest_path_loc = search_loc;
            _find_with_dijkstras.shortest_path_length = path_length;
            _find_with_dijkstras.shortest_path_valid = true;
        }
        if (!searching_for_path_home) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Rally: %4.0fm to rally point %u", (double)path_length, (unsigned)_find_with_dijkstras.rally_index + 1);
            // advance to the next rally point
            _find_with_dijkstras.rally_index++;
            return false;
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Rally: %4.0fm to home", (double)path_length);
        }
        // the path home has been calculated so the search is complete
        // break to return results
        break;
    }

    // we should only reach here if the search has completed
    _find_with_dijkstras.state = FindWithDijkstras::State::COMPLETED;
    if (_find_with_dijkstras.shortest_path_valid) {
        ret = _find_with_dijkstras.shortest_path_loc;
        return true;
    }

    // fall back to using closest rally point or home location
    ret = calc_best_rally_or_home_location(current_loc, rtl_home_alt_amsl_cm);
    return true;
#else
    ret = calc_best_rally_or_home_location(current_loc, rtl_home_alt_amsl_cm);
    return true;
#endif
}

// singleton instance
AP_Rally *AP_Rally::_singleton;

namespace AP {

AP_Rally *rally()
{
    return AP_Rally::get_singleton();
}

}
#endif //HAL_RALLY_ENABLED
