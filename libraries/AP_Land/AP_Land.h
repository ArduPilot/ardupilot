// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file    AP_Land.h
/// @brief   Class for storing land parameters and methods.  Created to make mission planning for landing more simple.

/*
 * The AP_Land library:
 * 
 * Initial implementation: Michael Day, April 2014
 */
#ifndef AP_Land_H
#define AP_Land_H

#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_AHRS.h>
#include <AP_GPS.h>
#include <Compass.h>
#include <AP_TECS.h>
#include <AP_Rally.h>
#include <AP_Mission.h>

class AP_Land {
public:
    AP_Land(AP_AHRS &ahrs, AP_GPS &gps, Compass &compass, AP_TECS &tecs, AP_Rally &rally,
            AP_Mission &mission);
    
    uint8_t get_land_wings_level() const { return _land_wing_level; }
    uint8_t get_break_path() const { return _break_path; }
    const Location& get_break_point() const { return _break_point; }

    // Called periodically -- moves through each step of the landing sequence.
    // Rally glide landing method.
    // Returns false on error.
    // Before calling: Enter RTL mode (to begin proceeding to a rally point).
    bool preland_step_rally_land(const RallyLocation &ral_loc);

    //clear all settings, usually in preparation for a landing or after an abort
    void preland_clear();

    //Called prior to starting landing sequnece.  
    void preland_init();

    //true if we've started the pre-landing sequend.  false otherwise.
    bool preland_started() const { return _preland_started; }

    //if a landing is aborted the intended behavior is:
    //1. Continue flying along the landing flight path (keeps plane flying down the run and avoids veering to unintended locations near the ground)
    //2. Seek to return to recovery_alt
    //3. Once at _recovery_alt, change to RTL mode and return to rally point
    //
    //This method will fail to abort a landing and return false if:
    //  1. We're not landing (_preland_started == false)
    //  2. We're beyond The Point of No return: the plane has already flared.
    bool abort_landing(const uint16_t recovery_alt);

    bool aborting_landing() const { return _aborting_landing; }

    //returns altitude plane aims for when aborting a landing:
    uint16_t get_recovery_alt() const { return _recovery_alt; }

    uint32_t get_recovery_alt_cm_msl() const;

    Location get_location_1km_beyond_land() const;

    //return -1 if no landing waypoint could be found in the Mission or 
    //if the landing waypoint is too far away from base_loc
    int16_t find_nearest_landing_wp_index(const Location& base_loc) const;
    
    //Caller needs to know when to head to break altitude b/c I can't control
    //the current waypoint altitude from this library.
    bool head_to_break_alt() const { return _head_to_break_alt; }

    bool heading_as_desired_for_landing() const { return _land_heading_as_desired; }
    bool speed_as_desired_for_landing() const { return _land_speed_as_desired; }
    bool arrived_at_break_alt() const { return _land_break_alt_as_desired; }

    // parameter block
    static const struct AP_Param::GroupInfo var_info[];

protected:
    const AP_AHRS& _ahrs;
    const AP_GPS& _gps;
    const Compass& _compass;
    const AP_TECS& _tecs;
    const AP_Rally& _rally;
    const AP_Mission& _mission;

    int16_t _landing_wp_index;
    Location _landing_wp;
    Location _break_point;

    bool _preland_started;
    bool _head_to_break_alt;
    bool _land_break_alt_as_desired;
    bool _land_heading_as_desired;
    bool _land_speed_as_desired;
    uint8_t _turns_complete;
    bool _aborting_landing;
    uint16_t _recovery_alt;

    //parameters
    AP_Int8 _land_wing_level;
    AP_Int8 _break_path;
    AP_Int8 _max_turns;
};

#endif //AP_Land_H
