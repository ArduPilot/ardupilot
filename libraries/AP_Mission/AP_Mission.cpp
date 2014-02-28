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

///
/// public mission methods
///

/// start - resets current commands to point to the beginning of the mission
///     To-Do: should we validate the mission first and return true/false?
void AP_Mission::start()
{
    _flags.state = MISSION_RUNNING;
    _flags.nav_cmd_loaded = false;
    _flags.do_cmd_loaded = false;
    _nav_cmd.index = AP_MISSION_CMD_INDEX_NONE;
    _do_cmd.index = AP_MISSION_CMD_INDEX_NONE;
    _prev_nav_cmd_index = AP_MISSION_CMD_INDEX_NONE;
    init_jump_tracking();
    // advance to the first command
    if (!advance_current_nav_cmd()) {
        // on failure set mission complete
        complete();
    }
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
    // if mission had completed then start it from the first command
    if (_flags.state == MISSION_COMPLETE) {
        start();
        return;
    }

    // if mission had stopped then restart it
    if (_flags.state == MISSION_STOPPED) {
        _flags.state = MISSION_RUNNING;

        // if no valid nav command index restart from beginning
        if (_nav_cmd.index == AP_MISSION_CMD_INDEX_NONE) {
            start();
            return;
        }

        // restart active navigation command
        // Note: if there is no active command then the mission must have been stopped just after the previous nav command completed
        //      update will take care of finding and starting the nav command
        if (_flags.nav_cmd_loaded) {
            _cmd_start_fn(_nav_cmd);
        }

        // restart active do command
        if (_flags.do_cmd_loaded && _do_cmd.index != AP_MISSION_CMD_INDEX_NONE) {
            _cmd_start_fn(_do_cmd);
        }
    }
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
    _flags.nav_cmd_loaded = false;
    _flags.do_cmd_loaded = false;

    // return success
    return true;
}

/// update - ensures the command queues are loaded with the next command and calls main programs command_init and command_verify functions to progress the mission
///     should be called at 10hz or higher
void AP_Mission::update()
{
    // exit immediately if not running or no mission commands
    if (_flags.state != MISSION_RUNNING || _cmd_total == 0) {
        return;
    }

    // check if we have an active nav command
    if (!_flags.nav_cmd_loaded || _nav_cmd.index == AP_MISSION_CMD_INDEX_NONE) {
        // advance in mission if no active nav command
        if (!advance_current_nav_cmd()) {
            // failure to advance nav command means mission has completed
            complete();
            return;
        }
    }else{
        // run the active nav command
        if (_cmd_verify_fn(_nav_cmd)) {
            // market _nav_cmd as complete (it will be started on the next iteration)
            _flags.nav_cmd_loaded = false;
        }
    }

    // check if we have an active do command
    if (!_flags.do_cmd_loaded || _do_cmd.index == AP_MISSION_CMD_INDEX_NONE) {
        advance_current_do_cmd();
    }else{
        // run the active do command
        if (_cmd_verify_fn(_do_cmd)) {
            // market _nav_cmd as complete (it will be started on the next iteration)
            _flags.do_cmd_loaded = false;
        }
    }
}

///
/// public command methods
///

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

/// is_nav_cmd - returns true if the command's id is a "navigation" command, false if "do" or "conditional" command
bool AP_Mission::is_nav_cmd(const Mission_Command& cmd)
{
    return (cmd.id <= MAV_CMD_NAV_LAST);
}

/// get_next_nav_cmd - gets next "navigation" command found at or after start_index
///     returns true if found, false if not found (i.e. reached end of mission command list)
///     accounts for do_jump commands but never increments the jump's num_times_run (advance_current_nav_cmd is responsible for this)
bool AP_Mission::get_next_nav_cmd(uint8_t start_index, Mission_Command& cmd)
{
    uint8_t cmd_index = start_index;
    Mission_Command temp_cmd;

    // search until the end of the mission command list
    while(cmd_index < _cmd_total) {
        // get next command
        if (!get_next_cmd(cmd_index, temp_cmd, false)) {
            // no more commands so return failure
            return false;
        }else{
            // if found a "navigation" command then return it
            if (is_nav_cmd(temp_cmd)) {
                cmd = temp_cmd;
                return true;
            }else{
                // move on in list
                cmd_index++;
            }
        }
    }

    // if we got this far we did not find a navigation command
    return false;
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

    // special handling for command #0 which is home
    if (index == 0) {
        cmd.index = 0;
        cmd.id = MAV_CMD_NAV_WAYPOINT;
        cmd.content.location = _ahrs.get_home();
    }else{
        // Find out proper location in memory by using the start_byte position + the index
        // we can load a command, we don't process it yet
        // read WP position
        pos_in_storage = (AP_MISSION_EEPROM_START_BYTE) + (index * AP_MISSION_EEPROM_COMMAND_SIZE);

        cmd.content.location.id = hal.storage->read_byte(pos_in_storage);
        pos_in_storage++;

        cmd.content.location.options = hal.storage->read_byte(pos_in_storage);
        pos_in_storage++;

        cmd.content.location.p1 = hal.storage->read_byte(pos_in_storage);
        pos_in_storage++;

        cmd.content.location.alt = hal.storage->read_dword(pos_in_storage);
        pos_in_storage += 4;

        cmd.content.location.lat = hal.storage->read_dword(pos_in_storage);
        pos_in_storage += 4;

        cmd.content.location.lng = hal.storage->read_dword(pos_in_storage);

        // set command's index to it's position in eeprom
        cmd.index = index;

        // set command from location's command
        // To-Do: remove id (and p1?) from Location structure
        cmd.id = cmd.content.location.id;
    }

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
        return false;
    }

    // calculate where in storage the command should be placed
    uint16_t pos_in_storage = AP_MISSION_EEPROM_START_BYTE + (index * AP_MISSION_EEPROM_COMMAND_SIZE);

    cmd.content.location.id = cmd.id;
    hal.storage->write_byte(pos_in_storage, cmd.content.location.id);

    pos_in_storage++;
    hal.storage->write_byte(pos_in_storage, cmd.content.location.options);

    pos_in_storage++;
    hal.storage->write_byte(pos_in_storage, cmd.content.location.p1);

    pos_in_storage++;
    hal.storage->write_dword(pos_in_storage, cmd.content.location.alt);

    pos_in_storage += 4;
    hal.storage->write_dword(pos_in_storage, cmd.content.location.lat);

    pos_in_storage += 4;
    hal.storage->write_dword(pos_in_storage, cmd.content.location.lng);

    // return success
    return true;
}

///
/// private methods
///

/// complete - mission is marked complete and clean-up performed including calling the mission_complete_fn
void AP_Mission::complete()
{
    // flag mission as complete
    _flags.state = MISSION_COMPLETE;

    // callback to main program's mission complete function
    _mission_complete_fn();
}

/// advance_current_nav_cmd - moves current nav command forward
///     do command will also be loaded
///     accounts for do-jump commands
//      returns true if command is advanced, false if failed (i.e. mission completed)
bool AP_Mission::advance_current_nav_cmd()
{
    Mission_Command cmd;
    uint8_t cmd_index;

    // exit immediately if we're not running
    if (_flags.state != MISSION_RUNNING) {
        return false;
    }

    // exit immediately if current nav command has not completed
    if (_flags.nav_cmd_loaded) {
        return false;
    }

    // stop the current running do command
    _do_cmd.index = AP_MISSION_CMD_INDEX_NONE;
    _flags.do_cmd_loaded = false;

    // get starting point for search
    cmd_index = _nav_cmd.index;
    if (cmd_index == AP_MISSION_CMD_INDEX_NONE) {
        cmd_index = AP_MISSION_FIRST_REAL_COMMAND;
    }else{
        // start from one position past the current nav command
        cmd_index++;
    }

    // search until we find next nav command or reach end of command list
    while (!_flags.nav_cmd_loaded) {
        // get next command
        if (!get_next_cmd(cmd_index, cmd, true)) {
            return false;
        }

        // check if navigation or "do" command
        if (is_nav_cmd(cmd)) {
            // save previous nav command index
            _prev_nav_cmd_index = _nav_cmd.index;
            // set current navigation command and start it
            _nav_cmd = cmd;
            _flags.nav_cmd_loaded = true;
            _cmd_start_fn(_nav_cmd);
        }else{
            // set current do command and start it (if not already set)
            if (!_flags.do_cmd_loaded) {
                _do_cmd = cmd;
                _flags.do_cmd_loaded = true;
                _cmd_start_fn(_do_cmd);
            }
        }
        // move onto next command
        cmd_index = cmd.index+1;
    }

    // if we got this far we must have successfully advanced the nav command
    return true;
}

/// advance_current_do_cmd - moves current do command forward
///     accounts for do-jump commands
void AP_Mission::advance_current_do_cmd()
{
    Mission_Command cmd;
    uint8_t cmd_index;

    // exit immediately if we're not running
    if (_flags.state != MISSION_RUNNING) {
        return;
    }

    // get starting point for search
    cmd_index = _do_cmd.index;
    if (cmd_index == AP_MISSION_CMD_INDEX_NONE) {
        cmd_index = AP_MISSION_FIRST_REAL_COMMAND;
    }else{
        // start from one position past the current do command
        cmd_index++;
    }

    // check if we've reached end of mission
    if (cmd_index >= _cmd_total) {
        // To-Do: set a flag to stop us from attempting to look for do-commands over and over?
        return;
    }

    // find next do command
    if (get_next_do_cmd(cmd_index, cmd)) {
        // set current do command and start it
        _do_cmd = cmd;
        _cmd_start_fn(_do_cmd);
    }else{
        // To-Do: set a flag to stop us from attempting to look for do-commands over and over?
        //  if added this flag should be reset once we advance the nav command
    }
}

/// get_next_cmd - gets next command found at or after start_index
///     returns true if found, false if not found (i.e. mission complete)
///     accounts for do_jump commands
///     increment_jump_num_times_if_found should be set to true if advancing the active navigation command
bool AP_Mission::get_next_cmd(uint8_t start_index, Mission_Command& cmd, bool increment_jump_num_times_if_found)
{
    uint8_t cmd_index = start_index;
    Mission_Command temp_cmd;
    uint8_t jump_index = AP_MISSION_CMD_INDEX_NONE;

    // search until the end of the mission command list
    while(cmd_index < _cmd_total) {
        // load the next command
        read_cmd_from_storage(cmd_index, temp_cmd);

        // check for do-jump command
        if (temp_cmd.id == MAV_CMD_DO_JUMP) {
            // check for endless loops
            if (!increment_jump_num_times_if_found && jump_index == cmd_index) {
                // we have somehow reached this jump command twice and there is no chance it will complete
                // To-Do: log an error?
                return false;
            }else{
                if (jump_index == AP_MISSION_CMD_INDEX_NONE) {
                    // record this command so we can check for endless loops
                    jump_index = cmd_index;
                }
            }
            // get number of times jump command has already been run
            uint8_t jump_times_run = get_jump_times_run(temp_cmd);
            if (jump_times_run < temp_cmd.content.jump.num_times) {
                // update the record of the number of times run
                if (increment_jump_num_times_if_found) {
                    increment_jump_times_run(temp_cmd);
                }
                // continue searching from jump target
                cmd_index = temp_cmd.content.jump.target;
            }else{
                // jump has been run specified number of times so move search to next command in mission
                cmd_index++;
            }
        }else{
            // this is a non-jump command so return it
            cmd = temp_cmd;
            return true;
        }
    }

    // if we got this far we did not find a navigation command
    return false;
}

/// get_next_do_cmd - gets next "do" or "conditional" command after start_index
///     returns true if found, false if not found
///     stops and returns false if it hits another navigation command before it finds the first do or conditional command
///     accounts for do_jump commands but never increments the jump's num_times_run (advance_current_nav_cmd is responsible for this)
bool AP_Mission::get_next_do_cmd(uint8_t start_index, Mission_Command& cmd)
{
    Mission_Command temp_cmd;

    // check we have not passed the end of the mission list
    if (start_index >= _cmd_total) {
        return false;
    }

    // get next command
    if (!get_next_cmd(start_index, temp_cmd, false)) {
        // no more commands so return failure
        return false;
    }else if (is_nav_cmd(temp_cmd)) {
        // if it's a "navigation" command then return false because we do not progress past nav commands
        return false;
    }else{
        // this must be a "do" or "conditional" and is not a do-jump command so return it
        cmd = temp_cmd;
        return true;
    }
}

///
/// jump handling methods
///

// init_jump_tracking - initialise jump_tracking variables
void AP_Mission::init_jump_tracking()
{
    for(uint8_t i=0; i<AP_MISSION_MAX_NUM_DO_JUMP_COMMANDS; i++) {
        _jump_tracking[i].index = AP_MISSION_CMD_INDEX_NONE;
        _jump_tracking[i].num_times_run = 0;
    }
}

/// get_jump_times_run - returns number of times the jump command has been run
uint8_t AP_Mission::get_jump_times_run(const Mission_Command& cmd)
{
    // exit immediatley if cmd is not a do-jump command
    if (cmd.id != MAV_CMD_DO_JUMP) {
        // To-Do: log an error?
        return AP_MISSION_JUMP_TIMES_MAX;
    }

    // search through jump_tracking array for this cmd
    for (uint8_t i=0; i<AP_MISSION_MAX_NUM_DO_JUMP_COMMANDS; i++) {
        if (_jump_tracking[i].index == cmd.index) {
            return _jump_tracking[i].num_times_run;
        }else if(_jump_tracking[i].index == AP_MISSION_CMD_INDEX_NONE) {
            // we've searched through all known jump commands and haven't found it so allocate new space in _jump_tracking array
            _jump_tracking[i].index = cmd.index;
            _jump_tracking[i].num_times_run = 0;
            return 0;
        }
    }

    // if we've gotten this far then the _jump_tracking array must be full
    // To-Do: log an error?
    return AP_MISSION_JUMP_TIMES_MAX;
}

/// increment_jump_times_run - increments the recorded number of times the jump command has been run
void AP_Mission::increment_jump_times_run(Mission_Command& cmd)
{
    // exit immediately if cmd is not a do-jump command
    if (cmd.id != MAV_CMD_DO_JUMP) {
        // To-Do: log an error?
        return;
    }

    // search through jump_tracking array for this cmd
    for (uint8_t i=0; i<AP_MISSION_MAX_NUM_DO_JUMP_COMMANDS; i++) {
        if (_jump_tracking[i].index == cmd.index) {
            _jump_tracking[i].num_times_run++;
            return;
        }else if(_jump_tracking[i].index == AP_MISSION_CMD_INDEX_NONE) {
            // we've searched through all known jump commands and haven't found it so allocate new space in _jump_tracking array
            _jump_tracking[i].index = cmd.index;
            _jump_tracking[i].num_times_run = 1;
            return;
        }
    }

    // if we've gotten this far then the _jump_tracking array must be full
    // To-Do: log an error
    return;
}
