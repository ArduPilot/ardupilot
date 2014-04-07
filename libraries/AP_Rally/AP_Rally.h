// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

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


#ifndef AP_Rally_h
#define AP_Rally_h


#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_AHRS.h>

/// @class    AP_Rally
/// @brief    Object managing Rally Points
class AP_Rally {

public:
    AP_Rally(AP_AHRS &ahrs, uint16_t max_rally_points, uint16_t rally_wp_size, uint16_t rally_start_byte);

    // data handling
    bool get_rally_point_with_index(unsigned i, RallyLocation &ret) const;
    bool set_rally_point_with_index(unsigned i, const RallyLocation &rallyLoc);
    uint8_t get_rally_total() const { return _rally_point_total_count; }

    // logic handling
    Location calc_best_rally_or_home_location(const Location &current_loc, float rtl_home_alt) const;
    bool find_nearest_rally_point(const Location &myloc, RallyLocation &ret) const;

    // parameter block
    static const struct AP_Param::GroupInfo var_info[];

private:
    // internal variables
    const AP_AHRS& _ahrs; // used only for home position
    const uint16_t _max_rally_points;
    const uint16_t _rally_wp_size;
    const uint16_t _rally_start_byte;

    // parameters
    AP_Int8  _rally_point_total_count;
    AP_Float _rally_limit_km;
};


#endif // AP_Rally_h
