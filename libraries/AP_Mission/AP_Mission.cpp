// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file    AP_Mission.cpp
/// @brief   Handles the MAVLINK command mission stack.  Reads and writes mission to storage.

#include "AP_Mission.h"

const AP_Param::GroupInfo AP_Mission::var_info[] PROGMEM = {

    // @Param: TOTAL
    // @DisplayName: Total number of commands in the mission
    // @Description: The number of mission mission items that has been loaded by the ground station. Do not change this manually.
    // @Range: 0 255
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("TOTAL",  0, AP_Mission, _cmd_total, 0),

    AP_GROUPEND
};

extern const AP_HAL::HAL& hal;

/// public methods

/// public methods
/// update - ensures the command queues are loaded with the next command and calls main programs command_init and command_verify functions to progress the mission
///     should be called at 10hz or higher
void AP_Mission::update()
{
    // exit immediately if not running or no mission commands
    if (_flags.state != MISSION_RUNNING || _cmd_total == 0) {
        return;
    }

    // check if nav, do, cond commands are loaded
        // if not load next nav-cmd
            // this should also prompt loading of new do or conditional commands
        // call command_init for each command loaded

    // if we're running nav_command, verify it
        // if command completed also push it to the prev_cmd
        // if command completed, mark nav command as unloaded (so it'll be loaded next time through)

    // if we're running do_command or conditional command, verify it
        // if it completes, unload the do-command queue (so a new one will be loaded next time through)
}

/// start - resets current commands to point to the beginning of the mission
///     To-Do: should we validate the mission first and return true/false?
void AP_Mission::start()
{
    _flags.state = MISSION_RUNNING;
    _flags.nav_cmd_loaded = false;
    _flags.do_cmd_loaded = false;
}

/// stop - stops mission execution.  subsequent calls to update() will have no effect until the mission is started or resumed
void AP_Mission::stop()
{
    _flags.state = MISSION_STOPPED;
}

/// resume - continues the mission execution from where we last left off
///     previous running commands will be re-initialised
void AP_Mission::resume()
{
    _flags.state = MISSION_RUNNING;
}

/// get_next_nav_cmd - returns the next navigation command
///     offset parameter controls how many commands forward we should look.  Defaults to 1 meaning the very next nav command
//const AP_Mission::Mission_Command& AP_Mission::get_next_nav_cmd(uint8_t offset) const
//{
//    get_next_cmd_index
//}

/// advance_current_nav_cmd - moves current nav command forward
///     do command will also be loaded
///     accounts for do-jump commands
bool AP_Mission::advance_current_nav_cmd(uint8_t start_index)
{
}

/// clear - clears out mission
///     returns true if mission was running so it could not be cleared
bool AP_Mission::clear()
{
    // do not allow clearing the mission while it is running
    if (_flags.state == MISSION_RUNNING) {
        return false;
    }

    // remove all commands
    _cmd_total.set_and_save(0);

    // clear index to commands
    _nav_cmd.index = AP_MISSION_CMD_INDEX_NONE;
    _do_cmd.index = AP_MISSION_CMD_INDEX_NONE;
}

/// valid - validate the mission has no errors
///     currently only checks that the number of do-commands does not exceed the AP_MISSION_MAX_NUM_DO_JUMP_COMMANDS
bool AP_Mission::valid()
{
    return true;
}

/// add_cmd - adds a command to the end of the command list and writes to storage
///     returns true if successfully added, false on failure
///     cmd.index is updated with it's new position in the mission
bool AP_Mission::add_cmd(Mission_Command& cmd)
{
    // attempt to write the command to storage
    bool ret = write_cmd_to_storage(_cmd_total, cmd);

    if (ret) {
        // update command's index
        cmd.index = _cmd_total;
        // increment total number of commands
        _cmd_total.set_and_save(_cmd_total + 1);
    }

    return ret;
}

/// load_cmd_from_storage - load command from storage
///     true is return if successful
bool AP_Mission::read_cmd_from_storage(uint8_t index, Mission_Command& cmd) const
{
    uint16_t pos_in_storage;    // position in storage from where we will read the next byte

    // exit immediately if index is beyond last command
    if (index > _cmd_total) {
        // return a command with a blank id
        cmd.id = AP_MISSION_CMD_ID_NONE;
        return false;
    }

    // Find out proper location in memory by using the start_byte position + the index
    // we can load a command, we don't process it yet
    // read WP position
    pos_in_storage = (AP_MISSION_EEPROM_START_BYTE) + (index * AP_MISSION_EEPROM_COMMAND_SIZE);

    // set command's index to it's position in eeprom
    cmd.index = index;

    // read command from eeprom
    hal.storage->read_block(cmd.content.bytes, pos_in_storage, AP_MISSION_EEPROM_COMMAND_SIZE);

    // set command from location's command
    // To-Do: remove id (and p1?) from Location structure
    cmd.id = cmd.content.location.id;

    // return success
    return true;
}

/// write_cmd_to_storage - write a command to storage
///     index is used to calculate the storage location
///     true is returned if successful
bool AP_Mission::write_cmd_to_storage(uint8_t index, Mission_Command& cmd)
{
    // range check cmd's index
    if (index >= AP_MISSION_MAX_COMMANDS) {
        // debug
        hal.console->printf_P(PSTR("fail %d > %d\n"),(int)index,(int)AP_MISSION_MAX_COMMANDS);
        return false;
    }

    // calculate where in storage the command should be placed
    uint16_t pos_in_storage = AP_MISSION_EEPROM_START_BYTE + (index * AP_MISSION_EEPROM_COMMAND_SIZE);

    // force home wp to absolute height
    //if (i == 0) {
    //    temp.options &= ~(AP_MISSION_MASK_OPTIONS_RELATIVE_ALT);
    //}

    // set location id to cmd.id
    // To-Do: remove id (and p1?) from Location structure
    cmd.content.location.id = cmd.id;

    // write block to eeprom
    hal.storage->write_block(pos_in_storage, cmd.content.bytes, AP_MISSION_EEPROM_COMMAND_SIZE);

    // return success
    return true;
}

/// get_next_cmd_index - gets next command after curr_cmd_index which is of type cmd_type
///     returns MISSION_CMD_NONE if no command is found
///     accounts for do_jump commands
///     if requesting MISSION_CMD_DO or MISSION_CMD_COND it will stop and return MISSION_CMD_NONE if it hits a MISSION_CMD_NAV first
uint8_t AP_Mission::get_next_cmd_index(uint8_t curr_cmd_index, mission_cmd_type cmd_type)
{
    // To-Do: actually implement this function!
    return curr_cmd_index+1;
}


/////////////// OLD STUFF ///////////////////////
/*
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

        //  This code is required to properly handle when there is a
        //   conditional command prior to a DO_JUMP
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

//---------------------Utility Methods------------------------------

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
    
    safe_home     = _home;
    safe_home.id  = MAV_CMD_NAV_LOITER_UNLIM;
    safe_home.alt = get_home_hold_alt();
}

//
//  Search for the next navigation index in the stack.  Designed to only return a valid index.
//
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
    // Other checks could be added such as:
    //   -Distance and altitude diff, if the new waypoint is hundreds of miles away from others, reject.
    //   -others?
    //
    if (temp.id > MAV_CMD_NAV_LAST) {
        return false;
    }

    if ((temp.lat < -900000000 || temp.lat > 900000000) ||
        (temp.lng < -1800000000 || temp.lng > 1800000000)) {
        return false;
    }

    return true;
}
*/
