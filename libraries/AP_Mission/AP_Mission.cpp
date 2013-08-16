// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file    AP_Mission.cpp
/// @brief   Handles the MAVLINK command mission stack.  Reads and writes mission to storage.

#include "AP_Mission.h"

const AP_Param::GroupInfo AP_Mission::var_info[] PROGMEM = {

    // @Param: CMD_MAX
    // @DisplayName: Number of loaded mission items
    // @Description: The number of mission mission items that has been loaded by the ground station. Do not change this manually.
    // @Range: 1 255
    // @User: Advanced
    AP_GROUPINFO("CMD_MAX",  0, AP_Mission, _cmd_max, 0),

    /* Need to figure out how to get this to default to define in config.h
        the default should be different for planes versus copters. */

    // @Param: RTL_ALT
    // @DisplayName: RTL altitude
    // @Description: Return to launch target altitude. This is the altitude the plane will aim for and loiter at when returning home. If this is negative (usually -1) then the plane will use the current altitude at the time of entering RTL.
    // @Range: 1 255
    // @User: Advanced
    AP_GROUPINFO("RTL_ALT",  1, AP_Mission, _RTL_altitude_cm, 10000),

    AP_GROUPEND
};

extern const AP_HAL::HAL& hal;

#define WP_SIZE 15

/*-------------------Commonly Used Methods --------------------------*/

void AP_Mission::init_commands()
{
    uint8_t tmp_index=_find_nav_index(1);
    change_waypoint_index(tmp_index);
    _prev_index_overriden= false;
    _mission_status = true;
}

bool AP_Mission::increment_waypoint_index()
{
    //Check if the current and after waypoint is home, if so the mission is complete.
    if (_index[1] == 0 && _index[2] == 0) {
        _mission_status = false;
        return false;
    }

    _index[0]=_index[1];
    _prev_index_overriden= false;

    if (_sync_waypoint_index(_index[2])) {
        _mission_status = true;
        return true;
    } else {
        _mission_status = false;
        return false;
    }
}

bool AP_Mission::change_waypoint_index(uint8_t new_index)
{
    //Current index is requested, no change.
    if(new_index == _index[1]) {
        return false;
    }

    //Home is requested.
    if(new_index == 0) {
        goto_home();
        return true;
    }

    Location tmp=get_cmd_with_index(new_index);
    if(_check_nav_valid(tmp)) {
        if(_sync_waypoint_index(new_index)) {
            _nav_waypoints[0]=_current_loc;
            _prev_index_overriden = true;
            _mission_status = true;
            return true;
        }
    }
    return false;
}

bool AP_Mission::get_new_cmd(struct Location &new_CMD)
{
    struct Location temp;
    temp = get_cmd_with_index(_cmd_index);

    if(temp.id <= MAV_CMD_NAV_LAST || _prev_index_overriden) {
        return false;                       //no more commands for this leg
    } else {

        /*  This code is required to properly handle when there is a
         *   conditional command prior to a DO_JUMP*/
        if(temp.id == MAV_CMD_DO_JUMP && (temp.lat > 0 || temp.lat == -1)) {
            uint8_t old_cmd_index = _cmd_index;

            if(change_waypoint_index(temp.p1)) {
                if( temp.lat > 0) {
                    temp.lat--;
                    temp.lat=constrain_int16(temp.lat, 0, 100);
                    set_cmd_with_index(temp, old_cmd_index);
                }
            } else { //Waypoint is already current, or do_jump was invalid.
                _cmd_index++;
                return false;
            }
        }

        new_CMD=temp;
        _cmd_index++;
        return true;
    }
}

/*--------------------------Specific Purpose Methods-------------------*/

void AP_Mission::goto_home()
{
    _index[1]=0;
    _index[2]=0;
    struct Location safe_home; 
    _safe_home(safe_home);
    goto_location(safe_home);
}

bool AP_Mission::goto_location(const struct Location &wp)
{
    if(_check_nav_valid(wp)) {
        _nav_waypoints[0] = _current_loc;
        _prev_index_overriden = true;
        _nav_waypoints[1] = wp;
        _nav_waypoints[2] = wp;
        return true;
    } else {
        return false;
    }
}

void AP_Mission::resume()
{
    _sync_nav_waypoints();
    _nav_waypoints[0]=_current_loc;
    _prev_index_overriden = true;
}

void AP_Mission::override_prev_wp(const struct Location &wp)
{
    _nav_waypoints[0] = wp;
    _prev_index_overriden = true;
}

/*---------------------Utility Methods------------------------------*/

void AP_Mission::set_command_total(uint8_t max_index)
{
    _cmd_max=max_index;
    _cmd_max.set_and_save(max_index);
}

void AP_Mission::set_home(const struct Location &home)
{
    _home=home;
    set_cmd_with_index(_home,0);
}

bool AP_Mission::_sync_waypoint_index(const uint8_t &new_index)
{
    Location tmp=get_cmd_with_index(new_index);
    if (new_index <= _cmd_max) {

        if (_check_nav_valid(tmp)) {

            _index[0]=_index[1];

            if (new_index == _cmd_max) { //The last waypoint in msn was requested.
                _index[1]=_cmd_max;
                _index[2]=0;
            } else if (new_index == 0) { //Home was requested.
                _index[1]=0;
                _index[2]=0;
            } else {
                _index[1]= new_index;
                _index[2]=_find_nav_index(_index[1]+1);
            }

            _cmd_index=_index[0]+1; //Reset command index to read commands associated with current mission leg.

            _sync_nav_waypoints();
            return true;
        }
    }
    return false;
}

void AP_Mission::_sync_nav_waypoints(){
    //TODO: this could be optimimzed by making use of the fact some waypoints are already loaded.
    for(int i=0; i<3; i++) {
        _nav_waypoints[i]=get_cmd_with_index(_index[i]);

        //Special handling for home, to ensure waypoint handed to vehicle is not 0 ft AGL.
        if(_index[i] == 0 && _nav_waypoints[i].id != MAV_CMD_NAV_LAND) {
            _safe_home(_nav_waypoints[i]);
        }
    }
}

void AP_Mission::_safe_home(struct Location &safe_home){ 
    
    safe_home=_home;
    safe_home.id=MAV_CMD_NAV_LOITER_UNLIM;

    if(_RTL_altitude_cm < 0) {
        safe_home.alt=_current_loc.alt;
    } else {
        safe_home.alt = _home.alt + _RTL_altitude_cm;
    }
}

/*
 *  Search for the next navigation index in the stack.  Designed to only return a valid index.
 */
uint8_t AP_Mission::_find_nav_index(uint8_t search_index)
{
    Location tmp;
    bool condition_cmd=false;

    while(search_index <= _cmd_max && search_index >= 0) {

        tmp = get_cmd_with_index(search_index);

        //check to see if there is a condition command.
        //don't want to do_jump before the condition command is executed.
        if (tmp.id > MAV_CMD_NAV_LAST && tmp.id < MAV_CMD_CONDITION_LAST) {
            condition_cmd=true;
        }

        //if there is a do_jump without a condition command preceding it, jump now.
        if (tmp.id == MAV_CMD_DO_JUMP && !condition_cmd) {
            if(tmp.p1 <= command_total() && (tmp.lat > 0 || tmp.lat == -1) ) {
                Location tmp_jump_to;
                tmp_jump_to=get_cmd_with_index(tmp.p1);

                if (_check_nav_valid(tmp_jump_to)) {
                    if (tmp.lat > 0) {
                        tmp.lat--;
                        set_cmd_with_index(tmp, search_index);
                    }
                    return tmp.p1;
                }
            }
        }

        //otherwise, if we come across a nav command, just pass that index along.
        if (_check_nav_valid(tmp)) {
            return search_index;
        }
        search_index++;
    }
    return 0;
}


bool AP_Mission::_check_nav_valid(const struct Location &temp)
{
    /* Other checks could be added such as:
     *   -Distance and altitude diff, if the new waypoint is hundreds of miles away from others, reject.
     *   -others?
     */
    if (temp.id > MAV_CMD_NAV_LAST) {
        return false;
    }

    if ((temp.lat < -900000000 || temp.lat > 900000000) ||
        (temp.lng < -1800000000 || temp.lng > 1800000000)) {
        return false;
    }

    return true;
}

struct Location AP_Mission::get_cmd_with_index_raw(int16_t i)
{
    struct Location temp;
    uint16_t mem;

    // Find out proper location in memory by using the start_byte position + the index
    // --------------------------------------------------------------------------------
    if (i > _cmd_max) {
        // we do not have a valid command to load
        // return a WP with a "Blank" id
        memset(&temp, 0, sizeof(temp));
        temp.id = AP_MISSION_CMD_BLANK;

    }else{
        // we can load a command, we don't process it yet
        // read WP position
        mem = (_start_byte) + (i * WP_SIZE);

        temp.id = hal.storage->read_byte(mem);

        mem++;
        temp.options = hal.storage->read_byte(mem);

        mem++;
        temp.p1 = hal.storage->read_byte(mem);

        mem++;
        temp.alt = hal.storage->read_dword(mem);  // alt is stored in CM! Alt is stored relative!

        mem += 4;
        temp.lat = hal.storage->read_dword(mem); // lat is stored in decimal * 1E7

        mem += 4;
        temp.lng = hal.storage->read_dword(mem); // lon is stored in decimal * 1E7
    }

    return temp;
}

/*
 *  Fetch a mission item from storage. Adjust altitude to be absolute
 */
struct Location AP_Mission::get_cmd_with_index(int16_t i)
{
    struct Location temp;

    temp = get_cmd_with_index_raw(i);

    // Add on home altitude if we are a nav command (or other command with altitude) and stored alt is relative
    if ((temp.id < MAV_CMD_NAV_LAST || temp.id == MAV_CMD_CONDITION_CHANGE_ALT) &&
        (temp.options & AP_MISSION_MASK_OPTIONS_RELATIVE_ALT) &&
        (temp.lat != 0 || temp.lng != 0 || temp.alt != 0)) {
        temp.alt += _home.alt;
    }

    // if lat and lon is zero, then use current lat/lon
    // this allows a mission to contain a "loiter on the spot"
    // command
    if (temp.lat == 0 && temp.lng == 0) {

        temp.lat = _current_loc.lat;
        temp.lng = _current_loc.lng;
        // additionally treat zero altitude as current altitude
        if (temp.alt == 0) {
            temp.alt = _current_loc.alt;
        }
    }

    return temp;
}

void AP_Mission::set_cmd_with_index(struct Location &temp, uint16_t i)
{
    i = constrain_int16(i, 0, command_total());
    uint16_t mem = _start_byte + (i * WP_SIZE);

    // force home wp to absolute height
    if (i == 0) {
        temp.options &= ~(AP_MISSION_MASK_OPTIONS_RELATIVE_ALT);
    }
    // zero unused bits
    temp.options &= (AP_MISSION_MASK_OPTIONS_RELATIVE_ALT | AP_MISSION_MASK_OPTIONS_LOITER_DIRECTION);

    hal.storage->write_byte(mem, temp.id);

    mem++;
    hal.storage->write_byte(mem, temp.options);

    mem++;
    hal.storage->write_byte(mem, temp.p1);

    mem++;
    hal.storage->write_dword(mem, temp.alt);

    mem += 4;
    hal.storage->write_dword(mem, temp.lat);

    mem += 4;
    hal.storage->write_dword(mem, temp.lng);
}
