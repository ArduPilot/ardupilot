// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file    AP_Land.cpp
/// @brief   Class for storing land parameters and methods.  Created to make mission planning for landing more simple.
#include "AP_Land.h"

#include <AP_HAL.h>
extern const AP_HAL::HAL& hal;

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
 #include <stdio.h>
 # define Debug(fmt, args ...)  do {printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); hal.scheduler->delay(1); } while(0)
#else
 # define Debug(fmt, args ...)
#endif

const AP_Param::GroupInfo AP_Land::var_info[] PROGMEM = {
    // @Param: WING_LEVEL
    // @DisplayName: Land Wings Level
    // @Description: How wings are held at landing after flare.  Some planes want to only navigate with the rudder when close to the ground to avoid the wing tips scraping.  Other planes don't have a rudder and don't have this option.  Still other planes have small enough wings that they may or may not want to keep wings level during the final landing phase. 
    // @Values: 0:KeepNavigating,1:HoldLevel,
    // @User: Advanced
    AP_GROUPINFO("WING_LEVEL", 0, AP_Land, _land_wing_level, 1),
    
    // @Param: BREAK_PATH
    // @DisplayName: Path followed after breaking from Rally Loiter during landing.
    // @Description: Type of break path. If Tangential, follow a line tangential to loiter on final approach.  If FromRallyPoint, follow a line from the center of the loiter to the landing point.
    // @Values: 0:Tangential,1:FromRallyPoint
    // @User: Advanced
    AP_GROUPINFO("BREAK_PATH", 1, AP_Land, _break_path, 0),

    // @Param: MAX_TURNS
    // @DisplayName: Maximum number of turns in loiter.
    // @Description Max number of loiter loops to go through before breaking for loiter AFTER reaching the target altitude.  Ensures plane doesn't loiter forever if airspeed is reading high.  Set to 0 to ignore number of turns when deciding when to break.
    // @User: Advanced
    AP_GROUPINFO("MAX_TURNS", 2, AP_Land, _max_turns, 3),

    AP_GROUPEND
};

//constructor
AP_Land::AP_Land(AP_AHRS &ahrs, AP_GPS &gps, Compass &compass, AP_TECS &tecs,
                 AP_Rally &rally, AP_Mission &mission)
    : _ahrs(ahrs)
    , _gps(gps)
    , _compass(compass)
    , _tecs(tecs)
    , _rally(rally)
    , _mission(mission)
    , _landing_wp_index(-1)
    , _preland_started(false)
    , _head_to_break_alt(false)
    , _land_break_alt_as_desired(false)
    , _land_heading_as_desired(false)
    , _land_speed_as_desired(false)
    , _turns_complete(0) 
    , _aborting_landing(false)
    , _recovery_alt(0) {        

    AP_Param::setup_object_defaults(this, var_info);
}

void AP_Land::preland_clear() {
    _landing_wp_index = -1;
    _preland_started = false;
    _head_to_break_alt = false;
    _land_break_alt_as_desired = false;
    _land_heading_as_desired = false;
    _land_speed_as_desired = false;
    _turns_complete = 0;
    _aborting_landing = false;
    _recovery_alt = 0;
}

void AP_Land::preland_init() {
    preland_clear();

    _preland_started = true;
}

bool AP_Land::abort_landing(const uint16_t recovery_alt) {
    //if we're not landing there's no need to abort landing
    if (_preland_started != true && _tecs.get_flight_stage() != AP_SpdHgtControl::FLIGHT_LAND_APPROACH) {
        return false;
    }

    //if we have already flared it's too late to abort.
    if (_tecs.get_flight_stage() == AP_SpdHgtControl::FLIGHT_LAND_FINAL) {
        return false;
    }
    
    preland_clear(); 
    
    _aborting_landing = true;
    _recovery_alt = recovery_alt;

    return true; 
}

uint32_t AP_Land::get_recovery_alt_cm_msl() const {
    return ((_recovery_alt*100UL) + _ahrs.get_home().alt);
}

Location AP_Land::get_location_1km_beyond_land() const {
    Location ret_loc = _break_point;

    location_update(ret_loc, get_bearing_cd(ret_loc, _landing_wp) * 0.01f, 
                    get_distance(ret_loc, _landing_wp) + 1000.0f);

    return ret_loc;
}

int16_t AP_Land::find_nearest_landing_wp_index(const Location& base_loc) const {
    // Start minimum distance at appx infinity
    float min_distance = 9999.9;
    int tmp_distance;
    int16_t land_wp_index = -1;
    
    AP_Mission::Mission_Command tmp = {0};

    // Go through all WayPoints looking for landing waypoints
    for(uint16_t i = 0; i< _mission.num_commands()+1; i++) {
        _mission.read_cmd_from_storage(i, tmp);
        if(tmp.id == MAV_CMD_NAV_LAND) {
           tmp_distance = get_distance(tmp.content.location, base_loc);
           if(tmp_distance < min_distance) {
              min_distance = tmp_distance;
              land_wp_index = i;
              break;
           }
        }
    }

    //DON'T GO TOO FAR! if the closest landing waypoint is too far away
    //(based on g.rally_limit_km), then don't use it -- don't autoland.
    if (_rally.get_rally_limit_km() > 0.f && 
            min_distance > _rally.get_rally_limit_km() * 1000.0f) {
        return -1;
    }

    return land_wp_index; 
}

bool AP_Land::preland_step_rally_land(const RallyLocation &ral_loc) {
    //If we haven't started the pre-land sequence, bail with no error.
    if (! _preland_started) {
        return true;
    }

    //If there are no rally points in this mission,
    //then this landing method doesn't make sense.
    if (_rally.get_rally_total() < 1) {
        Debug("No Rally Points.");
        return false;
    }

    Location current_loc;
    ((AP_AHRS&) _ahrs).get_position(current_loc);

    //find the best landing waypoint (if not already found)
    if (_landing_wp_index == -1) {
        _landing_wp_index = 
            find_nearest_landing_wp_index(_rally.rally_location_to_location(ral_loc));
        if (_landing_wp_index == -1) {
            Debug("No suitable suitable Landing Point (too far away?)");
            return false;
        }

        AP_Mission::Mission_Command lnd_cmd;
        _mission.read_cmd_from_storage(_landing_wp_index, lnd_cmd);
        _landing_wp = lnd_cmd.content.location;
    }
     
    //time to head for break altitude?
    if (_head_to_break_alt == false) {        
        //close enough to start breaking?
        if(get_distance(current_loc,_rally.rally_location_to_location(ral_loc))<100.0f) {
            //CAN'T DO THIS from a library.  Hence the head_to_break_alt method.
            //TODO: be able to modify current WP from a library. 
            //next_WP_cmd.content.location.alt = break_alt;
           
            _head_to_break_alt = true;

            /* Since fence isn't a library, we can't auto disable it from here.
             * TODO: Make geofence a library.
             */
        }
    } else { //we should be already heading for the break_alt
        if( _land_break_alt_as_desired == true || (current_loc.alt < 
                ((ral_loc.break_alt*100L) + _ahrs.get_home().alt + 500)) ) {
            _land_break_alt_as_desired = true;
            // Calculate bearing in radians
            float bearing = (radians( (float)(get_bearing_cd(current_loc,_landing_wp)/100.0) ));

            // Calculate heading
            float heading = 5000;
            if (((Compass &) _compass).read()) {
                const Matrix3f &m = _ahrs.get_dcm_matrix();
                heading = _compass.calculate_heading(m);

                //map heading to bearing's coordinate space:
                if (heading < 0.0f) {
                    heading += 2.0f*PI;
                }
            }
 
            // Check to see if the the plane is heading toward the land waypoint
            //3 degrees = 0.0523598776 radians
            if (fabs(bearing - heading) <= 0.0523598776f) {
                if(_land_heading_as_desired == false) {
                    _turns_complete++;
                }

                _land_heading_as_desired = true;
                                
                //now ensure we're going slower OR we've turned enough times:
                _land_speed_as_desired = false;
                if ( (_turns_complete != 0 && _turns_complete >= _max_turns) ||
                        _gps.ground_speed() - _tecs.get_target_airspeed() <= 2.0f) {
                    _land_speed_as_desired = true;

                    //tangential break path?
                    if (get_break_path() == 0) {
                        _break_point = current_loc;
                    } else { //get_break_path () == 1
                       _break_point = _rally.rally_location_to_location(ral_loc); 
                    }
                                 
                    /*       
                    //Can't set mode from a library!  These calls will
                    //have to be done from elsewhere! 
                    //Hence the heading_as_desired_for_landing and the
                    //speed_as_desired_for_landing methods
                    //
                    //Someday it would be nice to be able to have standardized
                    //modes that are accessible from libs.
                    //
                    //The order of these commands appears to be important
                    _mission.set_current_cmd(rally_land_wp_idx);
                    set_mode(AUTO);
                    mission.resume();
                    */                    
                }
            } else {
                _land_heading_as_desired = false;
            }
        } 
    }

    return true;
}
