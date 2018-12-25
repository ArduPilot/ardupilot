/// @file    AP_Rally.h
/// @brief   Handles rally point storage, retrieval and lookup

/*
 * The AP_Rally library:
 * 
 * Initial implementation: Michael Day, September 2013
 * Moved to AP_Rally lib:  Andrew Chapman April 2014
 * 
 * - responsible for managing a list of rally points
 * - reads and writes the rally points to storage
 * - provides access to the rally points, including logic to find the nearest one
 *
 */
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_AHRS/AP_AHRS.h>
#include <StorageManager/StorageManager.h>

#define AP_RALLY_WP_SIZE        15  // eeprom size of rally points

struct PACKED RallyLocation {
    int32_t lat;        //Latitude * 10^7
    int32_t lng;        //Longitude * 10^7
    int16_t alt;        //transit altitude (and loiter altitude) in meters (absolute);
    int16_t break_alt;  //when autolanding, break out of loiter at this alt (meters)
    uint16_t land_dir;   //when the time comes to auto-land, try to land in this direction (centidegrees)
    uint8_t flags;      //bit 0 = seek favorable winds when choosing a landing poi
                        //bit 1 = do auto land after arriving
                        //all other bits are for future use.
};

/// @class    AP_Rally
/// @brief    Object managing Rally Points
class AP_Rally {
public:
    AP_Rally(AP_AHRS &ahrs);

    /* Do not allow copies */
    AP_Rally(const AP_Rally &other) = delete;
    AP_Rally &operator=(const AP_Rally&) = delete;

    // data handling
    bool get_rally_point_with_index(uint8_t i, RallyLocation &ret) const;
    bool set_rally_point_with_index(uint8_t i, const RallyLocation &rallyLoc);
    uint8_t get_rally_total() const { return _rally_point_total_count; }
    uint8_t get_rally_max(void) const { return _storage.size() / AP_RALLY_WP_SIZE; }

    float get_rally_limit_km() const { return _rally_limit_km; }

    Location rally_location_to_location(const RallyLocation &ret) const;

    // logic handling
    Location calc_best_rally_or_home_location(const Location &current_loc, float rtl_home_alt) const;
    bool find_nearest_rally_point(const Location &myloc, RallyLocation &ret) const;

    // last time rally points changed
    uint32_t last_change_time_ms(void) const { return _last_change_time_ms; }

    // parameter block
    static const struct AP_Param::GroupInfo var_info[];

    // get singleton instance
    static AP_Rally *get_singleton() { return _singleton; }


private:
    static AP_Rally *_singleton;

    virtual bool is_valid(const Location &rally_point) const { return true; }

    static StorageAccess _storage;

    // internal variables
    const AP_AHRS& _ahrs; // used only for home position

    // parameters
    AP_Int8  _rally_point_total_count;
    AP_Float _rally_limit_km;
    AP_Int8  _rally_incl_home;

    uint32_t _last_change_time_ms;
};

namespace AP {
    AP_Rally *rally();
};
