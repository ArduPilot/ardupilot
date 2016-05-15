// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 *  Copyright (c) BirdsEyeView Aerobotics, LLC, 2016.
 *
 *  This program is free software: you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 3 as published
 *  by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 *  Public License version 3 for more details.
 *
 *  You should have received a copy of the GNU General Public License version
 *  3 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//BEV this entire file and AP_Mission.h were extensively modified to make them simpler
//and fix the numerous bugs inherent
#include "AP_Mission.h"
#include <AP_Terrain.h>

const AP_Param::GroupInfo AP_Mission::var_info[] PROGMEM = {

    // @Param: TOTAL
    // @DisplayName: Total mission commands
    // @Description: The number of mission mission items that has been loaded by the ground station. Do not change this manually.
    // @Range: 0 32766
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("TOTAL",  0, AP_Mission, _cmd_total, 0),

    // @Param: RESTART
    // @DisplayName: Mission Restart when entering Auto mode
    // @Description: Controls mission starting point when entering Auto mode (either restart from beginning of mission or resume from last command run)
    // @Values: 0:Resume Mission, 1:Restart Mission
    AP_GROUPINFO("RESTART",  1, AP_Mission, _restart, AP_MISSION_RESTART_DEFAULT),

    AP_GROUPEND
};

extern const AP_HAL::HAL& hal;

// storage object
StorageAccess AP_Mission::_storage(StorageManager::StorageMission);

///
/// public mission methods
///

/// init - initialises this library including checks the version in eeprom matches this library
void AP_Mission::init()
{
    // check_eeprom_version - checks version of missions stored in eeprom matches this library
    // command list will be cleared if they do not match
    check_eeprom_version();

    // prevent an easy programming error, this will be optimised out
    if (sizeof(union Content) != 12) {
        hal.scheduler->panic(PSTR("AP_Mission Content must be 12 bytes"));
    }

    _last_change_time_ms = hal.scheduler->millis();
}

/// start - resets current commands to point to the beginning of the mission
///     To-Do: should we validate the mission first and return true/false?
void AP_Mission::start()
{
    _flags.state = MISSION_RUNNING;

    reset(); // reset mission to the first command, resets jump tracking
    
    // advance to the first command
    if (!advance_current_cmd()) {
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
    if ( (_flags.state == MISSION_COMPLETE) || (_flags.state == MISSION_RUNNING_FORCE_RESTART) ) {
        start();
        return;
    }

    // if mission had stopped then restart it
    if (_flags.state == MISSION_STOPPED) {
        _flags.state = MISSION_RUNNING;

        // if no valid nav command index restart from beginning
        if (_cmd.index == AP_MISSION_CMD_INDEX_NONE) {
            start();
            return;
        }
    }

    // restart active navigation command. We run these on resume()
    // regardless of whether the mission was stopped, as we may be
    // re-entering AUTO mode and the nav_cmd callback needs to be run
    // to setup the current target waypoint

    // Note: if there is no active command then the mission must have been stopped just after the previous nav command completed
    //      update will take care of finding and starting the nav command
    if (_flags.cmd_loaded) {
        _cmd_start_fn(_cmd);
    } else {
        //BEV shouldn't make it to this case but it's here as a safety
        start();
    }
}

/// start_or_resume - if MIS_AUTORESTART=0 this will call resume(), otherwise it will call start()
void AP_Mission::start_or_resume()
{
    if (_restart || (_flags.state == MISSION_RUNNING_FORCE_RESTART) ) {
        start();
    } else {
        resume();
    }
}

/// reset - reset mission to the first command
void AP_Mission::reset()
{
    _flags.cmd_loaded  = false;
    _cmd.index         = AP_MISSION_CMD_INDEX_NONE;
    init_jump_tracking();
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
    _cmd.index = AP_MISSION_CMD_INDEX_NONE;
    _flags.cmd_loaded = false;

    // return success
    return true;
}

void AP_Mission::start_loiter_at_home(uint32_t alt)
{
    _cmd.index = 0; //index for home
    _cmd.content.location = _ahrs.get_home();
    _cmd.id = MAV_CMD_NAV_LOITER_UNLIM;
    _cmd.content.location.alt = alt; //if set to zero, _cmd_start_fn will use current altitude (see commands_logic.pde)
    _flags.cmd_loaded = true;

    _cmd_start_fn(_cmd);

    _flags.state = MISSION_RUNNING_FORCE_RESTART;
}


/// trucate - truncate any mission items beyond index
void AP_Mission::truncate(uint16_t index)
{
    if ((unsigned)_cmd_total > index) {        
        _cmd_total.set_and_save(index);
    }
}

/// update - ensures the command queues are loaded with the next command and calls main programs command_init and command_verify functions to progress the mission
///     should be called at 10hz or higher
void AP_Mission::update()
{
    //BEV this is a big problem. mission.update is only called when relying on auto mode to sequence waypoints.
    //if called and mission is not active the vehicle will end up going to the last commanded position,
    // which is an undesirable behavior. Just do a loiter unlimited at home
    // exit immediately if not running or no mission commands
    if (_flags.state == MISSION_STOPPED || _cmd_total == 0) {
        start_loiter_at_home(0); //zero altitude means use current alt
    }

    // check if we have an active nav command
    if (!_flags.cmd_loaded || _cmd.index == AP_MISSION_CMD_INDEX_NONE) {
        // advance in mission if no active nav command
        if (!advance_current_cmd()) {
            // failure to advance nav command means mission has completed
            complete();
            return;
        }
    }else{
        // run the active nav command
        if (_cmd_verify_fn(_cmd)) {
            // market _nav_cmd as complete (it will be started on the next iteration)
            _flags.cmd_loaded = false;
            // immediately advance to the next mission command
            if (!advance_current_cmd()) {
                // failure to advance nav command means mission has completed
                complete();
                return;
            }
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

/// replace_cmd - replaces the command at position 'index' in the command list with the provided cmd
///     replacing the current active command will have no effect until the command is restarted
///     returns true if successfully replaced, false on failure
bool AP_Mission::replace_cmd(uint16_t index, Mission_Command& cmd)
{
    // sanity check index
    if (index >= (unsigned)_cmd_total) {
        return false;
    }

    // attempt to write the command to storage
    return write_cmd_to_storage(index, cmd);
}

/// is_nav_cmd - returns true if the command's id is a "navigation" command, false if "do" or "conditional" command
bool AP_Mission::is_nav_cmd(const Mission_Command& cmd)
{
    return (cmd.id <= MAV_CMD_NAV_LAST);
}

/// get_next_nav_cmd - gets next "navigation" command found at or after start_index
///     returns true if found, false if not found (i.e. reached end of mission command list)
///     accounts for do_jump commands but never increments the jump's num_times_run (advance_current_cmd is responsible for this)
bool AP_Mission::get_next_nav_cmd(uint16_t start_index, Mission_Command& cmd)
{
    uint16_t cmd_index = start_index;

    // search until the end of the mission command list
    while(cmd_index < (unsigned)_cmd_total) {
        // get next command
        if (!get_next_cmd(cmd_index, cmd, false)) {
            // no more commands so return failure
            return false;
        }else{
            // if found a "navigation" command then return it
            if (is_nav_cmd(cmd)) {
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

/// get the ground course of the next navigation leg in centidegrees
/// from 0 36000. Return default_angle if next navigation
/// leg cannot be determined
int32_t AP_Mission::get_next_ground_course_cd(int32_t default_angle)
{
    Mission_Command cmd;
    if (!get_next_nav_cmd(_cmd.index+1, cmd)) {
        return default_angle;
    }
    return get_bearing_cd(_cmd.content.location, cmd.content.location);
}

// set_current_cmd - jumps to command specified by index
bool AP_Mission::set_current_cmd(uint16_t index)
{
    Mission_Command cmd;

    // sanity check index and that we have a mission
    if (index >= (unsigned)_cmd_total || _cmd_total == 1) {
        return false;
    }

    // stop current nav cmd
    _flags.cmd_loaded = false;

    // if index is zero then the user wants to completely restart the mission
    if (index == 0 || _flags.state == MISSION_COMPLETE) {
        // reset the jump tracking to zero
        init_jump_tracking();
        if (index == 0) {
            index = 1;
        }
    }

    // if the mission is stopped or completed move the nav_cmd index to the specified point and set the state to stopped
    // so that if the user resumes the mission it will begin at the specified index
    if (_flags.state != MISSION_RUNNING) {
        // search until we find next nav command or reach end of command list
        while (!_flags.cmd_loaded) {
            // get next command
            if (!get_next_cmd(index, cmd, true)) {
                _cmd.index = AP_MISSION_CMD_INDEX_NONE;
                return false;
            }

            //BEV it should be okay to set the idex to point to "do" commands.
            // check if navigation or "do" command
            //if (is_nav_cmd(cmd)) {
                // set current navigation command
                _cmd = cmd;
                _flags.cmd_loaded = true;
            //}
            // move onto next command
            index = cmd.index+1;
        }

        // if we got this far then the mission can safely be "resumed" from the specified index so we set the state to "stopped"
        _flags.state = MISSION_STOPPED;
        //BEV todo init jump tracking to fix issue with failed jumps first time running mission if AUTO commanded from wp
        //dropdown menu in Mission Planner.
        init_jump_tracking();
        return true;
    }

    // the state must be MISSION_RUNNING
    // search until we find next nav command or reach end of command list
    while (!_flags.cmd_loaded) {
        // get next command
        if (!get_next_cmd(index, cmd, true)) {
            // if we run out of nav commands mark mission as complete
            complete();
            // return true because we did what was requested
            // which was apparently to jump to a command at the end of the mission
            return true;
        }

        // check if navigation or "do" command
        if (is_nav_cmd(cmd)) {
            // set current navigation command and start it
            _cmd = cmd;
            _flags.cmd_loaded = true;
            _cmd_start_fn(_cmd);
        }
        // move onto next command
        index = cmd.index+1;
    }

    // if we got this far we must have successfully advanced the nav command
    return true;
}

/// load_cmd_from_storage - load command from storage
///     true is return if successful
bool AP_Mission::read_cmd_from_storage(uint16_t index, Mission_Command& cmd) const
{
    // exit immediately if index is beyond last command but we always let cmd #0 (i.e. home) be read
    if (index > (unsigned)_cmd_total && index != 0) {
        return false;
    }

    // special handling for command #0 which is home
    if (index == 0) {
        cmd.index = 0;
        cmd.id = MAV_CMD_NAV_WAYPOINT;
        cmd.p1 = 0;
        cmd.content.location = _ahrs.get_home();
    }else{
        // Find out proper location in memory by using the start_byte position + the index
        // we can load a command, we don't process it yet
        // read WP position
        uint16_t pos_in_storage = 4 + (index * AP_MISSION_EEPROM_COMMAND_SIZE);

        cmd.id = _storage.read_byte(pos_in_storage);
        cmd.p1 = _storage.read_uint16(pos_in_storage+1);
        _storage.read_block(cmd.content.bytes, pos_in_storage+3, 12);

        // set command's index to it's position in eeprom
        cmd.index = index;
    }

    // return success
    return true;
}

/// write_cmd_to_storage - write a command to storage
///     index is used to calculate the storage location
///     true is returned if successful
bool AP_Mission::write_cmd_to_storage(uint16_t index, Mission_Command& cmd)
{
    // range check cmd's index
    if (index >= num_commands_max()) {
        return false;
    }

    // calculate where in storage the command should be placed
    uint16_t pos_in_storage = 4 + (index * AP_MISSION_EEPROM_COMMAND_SIZE);

    _storage.write_byte(pos_in_storage, cmd.id);
    _storage.write_uint16(pos_in_storage+1, cmd.p1);
    _storage.write_block(pos_in_storage+3, cmd.content.bytes, 12);

    // remember when the mission last changed
    _last_change_time_ms = hal.scheduler->millis();

    // return success
    return true;
}

/// write_home_to_storage - writes the special purpose cmd 0 (home) to storage
///     home is taken directly from ahrs
void AP_Mission::write_home_to_storage()
{
    Mission_Command home_cmd = {};
    home_cmd.id = MAV_CMD_NAV_WAYPOINT;
    home_cmd.content.location = _ahrs.get_home();
    write_cmd_to_storage(0,home_cmd);
}

// mavlink_to_mission_cmd - converts mavlink message to an AP_Mission::Mission_Command object which can be stored to eeprom
//  return true on success, false on failure
bool AP_Mission::mavlink_to_mission_cmd(const mavlink_mission_item_t& packet, AP_Mission::Mission_Command& cmd)
{
    bool copy_location = false;
    bool copy_alt = false;
    uint8_t num_turns, radius_m; // used by MAV_CMD_NAV_LOITER_TURNS & _TO_ALT

    // command's position in mission list and mavlink id
    cmd.index = packet.seq;
    cmd.id = packet.command;
    cmd.content.location.options = 0;

    // command specific conversions from mavlink packet to mission command
    switch (cmd.id) {

    case MAV_CMD_NAV_WAYPOINT:                          // MAV ID: 16
        copy_location = true;
        // delay at waypoint in seconds
        cmd.p1 = packet.param1;
        break;

    case MAV_CMD_NAV_LOITER_UNLIM:                      // MAV ID: 17
        copy_location = true;
        cmd.content.location.flags.loiter_ccw = (packet.param3 < 0);    // -1 = counter clockwise, +1 = clockwise
        break;

    case MAV_CMD_NAV_LOITER_TURNS:                      // MAV ID: 18
        copy_location = true;
        num_turns = packet.param1;                      // number of times to circle is held in param1
        radius_m = fabsf(packet.param3);                // radius in meters is held in high in param3
        cmd.p1 = (((uint16_t)radius_m)<<8) | (uint16_t)num_turns;   // store radius in high byte of p1, num turns in low byte of p1
        cmd.content.location.flags.loiter_ccw = (packet.param3 < 0);
        break;

    case MAV_CMD_NAV_LOITER_TIME:                       // MAV ID: 19
        copy_location = true;
        cmd.p1 = packet.param1;                         // loiter time in seconds
        cmd.content.location.flags.loiter_ccw = (packet.param3 < 0);
        break;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:                  // MAV ID: 20
        copy_location = true;
        break;

    case MAV_CMD_NAV_LAND:                              // MAV ID: 21
        copy_location = true;
        break;

    case MAV_CMD_NAV_TAKEOFF:                           // MAV ID: 22
        copy_location = true;                           // only altitude is used
        cmd.p1 = packet.param1;                         // minimum pitch (plane only)
        break;

    case MAV_CMD_DO_JUMP:                               // MAV ID: 177
        cmd.content.jump.target = packet.param1;        // jump-to command number
        cmd.content.jump.num_times = packet.param2;     // repeat count
        break;

    case MAV_CMD_DO_SET_SERVO:                          // MAV ID: 183
        cmd.content.servo.channel = packet.param1;      // channel
        cmd.content.servo.pwm = packet.param2;          // PWM
        break;

    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:                 // MAV ID: 206
        cmd.content.cam_trigg_dist.meters = packet.param1;  // distance between camera shots in meters
        break;

    //BEV custom commands
    case MAV_CMD_DO_TRANSITION_TOGGLE:
    case MAV_CMD_DO_GEAR_TOGGLE:
        //these commands take no parameters
        break;


    case MAV_CMD_DO_SET_ROI:
        copy_location = true;
        break;

    default:
        // unrecognised command
        return false;
    }

    // copy location from mavlink to command
    if (copy_location || copy_alt) {
        switch (packet.frame) {

        case MAV_FRAME_MISSION:
        case MAV_FRAME_GLOBAL:
            if (copy_location) {
                cmd.content.location.lat = 1.0e7f * packet.x;   // floating point latitude to int32_t
                cmd.content.location.lng = 1.0e7f * packet.y;   // floating point longitude to int32_t
            }
            cmd.content.location.alt = packet.z * 100.0f;       // convert packet's alt (m) to cmd alt (cm)
            cmd.content.location.flags.relative_alt = 0;
            break;

        case MAV_FRAME_GLOBAL_RELATIVE_ALT:
            if (copy_location) {
                cmd.content.location.lat = 1.0e7f * packet.x;   // floating point latitude to int32_t
                cmd.content.location.lng = 1.0e7f * packet.y;   // floating point longitude to int32_t
            }
            cmd.content.location.alt = packet.z * 100.0f;       // convert packet's alt (m) to cmd alt (cm)
            cmd.content.location.flags.relative_alt = 1;
            break;

#ifdef MAV_FRAME_LOCAL_NED
        case MAV_FRAME_LOCAL_NED:                         // local (relative to home position)
            if (copy_location) {
                cmd.content.location.lat = 1.0e7f*ToDeg(packet.x/
                                           (RADIUS_OF_EARTH*cosf(ToRad(home.lat/1.0e7f)))) + _ahrs.get_home().lat;
                cmd.content.location.lng = 1.0e7f*ToDeg(packet.y/RADIUS_OF_EARTH) + _ahrs.get_home().lng;
            }
            cmd.content.location.alt = -packet.z*1.0e2f;
            cmd.content.location.flags.relative_alt = 1;
            break;
#endif

#ifdef MAV_FRAME_LOCAL
        case MAV_FRAME_LOCAL:                         // local (relative to home position)
            if (copy_location) {
                cmd.content.location.lat = 1.0e7f*ToDeg(packet.x/
                                           (RADIUS_OF_EARTH*cosf(ToRad(home.lat/1.0e7f)))) + _ahrs.get_home().lat;
                cmd.content.location.lng = 1.0e7f*ToDeg(packet.y/RADIUS_OF_EARTH) + _ahrs.get_home().lng;
            }
            cmd.content.location.alt = packet.z*1.0e2f;
            cmd.content.location.flags.relative_alt = 1;
            break;
#endif

        default:
            return false;
        }
    }

    // if we got this far then it must have been succesful
    return true;
}

// mission_cmd_to_mavlink - converts an AP_Mission::Mission_Command object to a mavlink message which can be sent to the GCS
//  return true on success, false on failure
bool AP_Mission::mission_cmd_to_mavlink(const AP_Mission::Mission_Command& cmd, mavlink_mission_item_t& packet)
{
    bool copy_location = false;
    bool copy_alt = false;

    // command's position in mission list and mavlink id
    packet.seq = cmd.index;
    packet.command = cmd.id;

    // set defaults
    packet.current = 0;     // 1 if we are passing back the mission command that is currently being executed
    packet.param1 = 0;
    packet.param2 = 0;
    packet.param3 = 0;
    packet.param4 = 0;
    packet.autocontinue = 1;

    // command specific conversions from mission command to mavlink packet
    switch (cmd.id) {

    case MAV_CMD_NAV_WAYPOINT:                          // MAV ID: 16
        copy_location = true;
        // delay at waypoint in seconds
        packet.param1 = cmd.p1;
        break;

    case MAV_CMD_NAV_LOITER_UNLIM:                      // MAV ID: 17
        copy_location = true;
        if (cmd.content.location.flags.loiter_ccw) {
            packet.param3 = -1;
        }else{
            packet.param3 = 1;
        }
        break;

    case MAV_CMD_NAV_LOITER_TURNS:                      // MAV ID: 18
        copy_location = true;
        packet.param1 = LOWBYTE(cmd.p1);                // number of times to circle is held in low byte of p1
        packet.param3 = HIGHBYTE(cmd.p1);               // radius is held in high byte of p1
        if (cmd.content.location.flags.loiter_ccw) {
            packet.param3 = -packet.param3;
        }
        break;

    case MAV_CMD_NAV_LOITER_TIME:                       // MAV ID: 19
        copy_location = true;
        packet.param1 = cmd.p1;                         // loiter time in seconds
        if (cmd.content.location.flags.loiter_ccw) {
            packet.param3 = -1;
        }else{
            packet.param3 = 1;
        }
        break;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:                  // MAV ID: 20
        copy_location = true;
        break;

    case MAV_CMD_NAV_LAND:                              // MAV ID: 21
        copy_location = true;
        break;

    case MAV_CMD_NAV_TAKEOFF:                           // MAV ID: 22
        copy_location = true;                           // only altitude is used
        packet.param1 = cmd.p1;                         // minimum pitch (plane only)
        break;

    case MAV_CMD_DO_JUMP:                               // MAV ID: 177
        packet.param1 = cmd.content.jump.target;        // jump-to command number
        packet.param2 = cmd.content.jump.num_times;     // repeat count
        break;

    case MAV_CMD_DO_SET_SERVO:                          // MAV ID: 183
        packet.param1 = cmd.content.servo.channel;      // channel
        packet.param2 = cmd.content.servo.pwm;          // PWM
        break;

    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:                 // MAV ID: 206
        packet.param1 = cmd.content.cam_trigg_dist.meters;  // distance between camera shots in meters
        break;

    case MAV_CMD_DO_TRANSITION_TOGGLE:
    case MAV_CMD_DO_GEAR_TOGGLE:
        //these commands take no parameters
        break;


    case MAV_CMD_DO_SET_ROI:
        copy_location = true;
        break;

    default:
        // unrecognised command
        return false;
    }

    // copy location from mavlink to command
    if (copy_location) {
        packet.x = cmd.content.location.lat / 1.0e7f;   // int32_t latitude to float
        packet.y = cmd.content.location.lng / 1.0e7f;   // int32_t longitude to float
    }
    if (copy_location || copy_alt) {
        packet.z = cmd.content.location.alt / 100.0f;   // cmd alt in cm to m
        if (cmd.content.location.flags.relative_alt) {
            packet.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
        }else{
            packet.frame = MAV_FRAME_GLOBAL;
        }
        // don't ever return terrain mission items if no terrain support
        if (cmd.content.location.flags.terrain_alt) {
            return false;
        }
    }

    // if we got this far then it must have been successful
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

/// advance_current_cmd - moves current nav command forward
///     do command will also be loaded
///     accounts for do-jump commands
//      returns true if command is advanced, false if failed (i.e. mission completed)
bool AP_Mission::advance_current_cmd()
{
    Mission_Command cmd;
    uint16_t cmd_index;

    // exit immediately if we're not running
    //BEV no. We have to keep trying, even if the state isn't running
    //if (_flags.state != MISSION_RUNNING) {
    //    return false;
    //}

    if(_flags.state == MISSION_RUNNING_FORCE_RESTART) {
        this->reset();
    }

    // exit immediately if current nav command has not completed
    if (_flags.cmd_loaded) {
        return false;
    }

    // get starting point for search
    cmd_index = _cmd.index;
    if (cmd_index == AP_MISSION_CMD_INDEX_NONE) {
        // start from beginning of the mission command list
        cmd_index = AP_MISSION_FIRST_REAL_COMMAND;
    }else{
        // start from one position past the current nav command
        cmd_index++;
    }

    //avoid endless loops
    uint8_t max_loops = 255;

    // search until we find next nav command or reach end of command list
    while (!_flags.cmd_loaded) {
        // get next command
        if (!get_next_cmd(cmd_index, cmd, true)) {
            return false;
        }

        // check if navigation or "do" command
        if (is_nav_cmd(cmd)) {
            // set current navigation command and start it
            _cmd = cmd;
            _flags.cmd_loaded = true;
            _cmd_start_fn(_cmd);
            return true;
        } else {
            //must be a do command. Load and execute
            _cmd_start_fn(cmd);
            //keep from loading too many (endless loop)
            if(max_loops-- == 0) {
                return false;
            }
        }
        // move onto next command
        cmd_index = cmd.index+1;
    }

    // if we got this far we must have successfully advanced the nav command
    return true;
}

/// get_next_cmd - gets next command found at or after start_index
///     returns true if found, false if not found (i.e. mission complete)
///     accounts for do_jump commands
///     increment_jump_num_times_if_found should be set to true if advancing the active navigation command
bool AP_Mission::get_next_cmd(uint16_t start_index, Mission_Command& cmd, bool increment_jump_num_times_if_found)
{
    uint16_t cmd_index = start_index;
    Mission_Command temp_cmd;
    uint16_t jump_index = AP_MISSION_CMD_INDEX_NONE;

    // search until the end of the mission command list
    uint8_t max_loops = 64;
    while(cmd_index < (unsigned)_cmd_total) {
        // load the next command
        if (!read_cmd_from_storage(cmd_index, temp_cmd)) {
            // this should never happen because of check above but just in case
            return false;
        }

        // check for do-jump command
        if (temp_cmd.id == MAV_CMD_DO_JUMP) {

            if (max_loops-- == 0) {
                return false;
            }

            // check for invalid target
            if ((temp_cmd.content.jump.target >= (unsigned)_cmd_total) || (temp_cmd.content.jump.target == 0)) {
                // To-Do: log an error?
                return false;
            }

            // check for endless loops
            if (!increment_jump_num_times_if_found && jump_index == cmd_index) {
                // we have somehow reached this jump command twice and there is no chance it will complete
                // To-Do: log an error?
                return false;
            }

            // record this command so we can check for endless loops
            if (jump_index == AP_MISSION_CMD_INDEX_NONE) {
                jump_index = cmd_index;
            }

            // check if jump command is 'repeat forever'
            if (temp_cmd.content.jump.num_times == AP_MISSION_JUMP_REPEAT_FOREVER) {
                // continue searching from jump target
                cmd_index = temp_cmd.content.jump.target;
            }else{
                // get number of times jump command has already been run
                int16_t jump_times_run = get_jump_times_run(temp_cmd);
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
            }
        }else{
            // this is a non-jump command so return it
            cmd = temp_cmd;
            return true;
        }
    }

    // if we got this far we did not find a non-jump command
    return false;
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
int16_t AP_Mission::get_jump_times_run(const Mission_Command& cmd)
{
    // exit immediatley if cmd is not a do-jump command or target is invalid
    if ((cmd.id != MAV_CMD_DO_JUMP) || (cmd.content.jump.target >= (unsigned)_cmd_total) || (cmd.content.jump.target == 0)) {
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

// check_eeprom_version - checks version of missions stored in eeprom matches this library
// command list will be cleared if they do not match
void AP_Mission::check_eeprom_version()
{
    uint32_t eeprom_version = _storage.read_uint32(0);

    // if eeprom version does not match, clear the command list and update the eeprom version
    if (eeprom_version != AP_MISSION_EEPROM_VERSION) {
        if (clear()) {
            _storage.write_uint32(0, AP_MISSION_EEPROM_VERSION);
        }
    }
}

/*
  return total number of commands that can fit in storage space
 */
uint16_t AP_Mission::num_commands_max(void) const
{
    // -4 to remove space for eeprom version number
    return (_storage.size() - 4) / AP_MISSION_EEPROM_COMMAND_SIZE;
}
