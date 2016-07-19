// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file    AP_Rally.h
/// @brief   Handles rally point storage and retrieval.
#include "AP_Rally.h"

#include <AP_HAL/AP_HAL.h>
extern const AP_HAL::HAL& hal;

// storage object
StorageAccess AP_Rally::_storage(StorageManager::StorageRally);

// ArduCopter/defines.h sets this, and this definition will be moved into ArduPlane/defines.h when that is patched to use the lib
#ifdef APM_BUILD_DIRECTORY
  #if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
    #define RALLY_LIMIT_KM_DEFAULT 0.3f
    #define RALLY_INCLUDE_HOME_DEFAULT 1
  #elif APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    #define RALLY_LIMIT_KM_DEFAULT 5.0f
    #define RALLY_INCLUDE_HOME_DEFAULT 0
  #elif APM_BUILD_TYPE(APM_BUILD_APMrover2)
    #define RALLY_LIMIT_KM_DEFAULT 0.5f
    #define RALLY_INCLUDE_HOME_DEFAULT 0
  #endif
#endif  // APM_BUILD_DIRECTORY

#ifndef RALLY_LIMIT_KM_DEFAULT
#define RALLY_LIMIT_KM_DEFAULT 1.0f
#endif

#ifndef RALLY_INCLUDE_HOME_DEFAULT
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
    // @Units: kilometers
    // @Increment: 0.1
    AP_GROUPINFO("LIMIT_KM", 1, AP_Rally, _rally_limit_km, RALLY_LIMIT_KM_DEFAULT),

    // @Param: INCL_HOME
    // @DisplayName: Rally Include Home
    // @Description: Controls if Home is included as a Rally point (i.e. as a safe landing place) for RTL
    // @User: Standard
    // @Values: 0:DoNotIncludeHome,1:IncludeHome
    AP_GROUPINFO("INCL_HOME", 2, AP_Rally, _rally_incl_home, RALLY_INCLUDE_HOME_DEFAULT),

    AP_GROUPEND
};

// constructor
AP_Rally::AP_Rally(AP_AHRS &ahrs) 
    : _ahrs(ahrs)
    , _last_change_time_ms(0xFFFFFFFF)
{
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

    return true;
}

// helper function to translate a RallyLocation to a Location
Location AP_Rally::rally_location_to_location(const RallyLocation &rally_loc) const
{
    Location ret = {};

    // we return an absolute altitude, as we add homeloc.alt below
    ret.flags.relative_alt = false;

    //Currently can't do true AGL on the APM.  Relative altitudes are
    //relative to HOME point's altitude.  Terrain on the board is inbound
    //for the PX4, though.  This line will need to be updated when that happens:
    ret.alt = (rally_loc.alt*100UL) + _ahrs.get_home().alt;

    ret.lat = rally_loc.lat;
    ret.lng = rally_loc.lng;

    return ret;
}

// returns true if a valid rally point is found, otherwise returns false to indicate home position should be used
bool AP_Rally::find_nearest_rally_point(const Location &current_loc, RallyLocation &return_loc) const
{
    float min_dis = -1;
    const struct Location &home_loc = _ahrs.get_home();

    for (uint8_t i = 0; i < (uint8_t) _rally_point_total_count; i++) {
        RallyLocation next_rally;
        if (!get_rally_point_with_index(i, next_rally)) {
            continue;
        }
        Location rally_loc = rally_location_to_location(next_rally);
        float dis = get_distance(current_loc, rally_loc);

        if (is_valid(rally_loc) && (dis < min_dis || min_dis < 0)) {
            min_dis = dis;
            return_loc = next_rally;
        }
    }

    // if home is included, return false (meaning use home) if it is closer than all rally points
    if (_rally_incl_home && (get_distance(current_loc, home_loc) < min_dis)) {
        return false;
    }

    // if a limit is defined and all rally points are beyond that limit, use home if it is closer
    if ((_rally_limit_km > 0) && (min_dis > _rally_limit_km*1000.0f) && (get_distance(current_loc, home_loc) < min_dis)) {
        return false; // use home position
    }

    // use home if no rally points found
    return min_dis >= 0;
}

// return best RTL location from current position
Location AP_Rally::calc_best_rally_or_home_location(const Location &current_loc, float rtl_home_alt) const
{
    RallyLocation ral_loc = {};
    Location return_loc = {};
    const struct Location &home_loc = _ahrs.get_home();
    
    if (find_nearest_rally_point(current_loc, ral_loc)) {
        // valid rally point found
        return_loc = rally_location_to_location(ral_loc);
    } else {
        // no valid rally point, return home position
        return_loc = home_loc;
        return_loc.alt = rtl_home_alt;
        return_loc.flags.relative_alt = false; // read_alt_to_hold returns an absolute altitude
    }

    return return_loc;
}
