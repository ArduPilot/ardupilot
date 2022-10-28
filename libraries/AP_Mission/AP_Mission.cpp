/// @file    AP_Mission.cpp
/// @brief   Handles the MAVLINK command mission stack.  Reads and writes mission to storage.

#include "AP_Mission.h"
#include <AP_Terrain/AP_Terrain.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Camera/AP_Camera.h>
#include <AP_Gripper/AP_Gripper_config.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

const AP_Param::GroupInfo AP_Mission::var_info[] = {

    // @Param: TOTAL
    // @DisplayName: Total mission commands
    // @Description: The number of mission mission items that has been loaded by the ground station. Do not change this manually.
    // @Range: 0 32766
    // @Increment: 1
    // @User: Advanced
    // @ReadOnly: True
    AP_GROUPINFO_FLAGS("TOTAL",  0, AP_Mission, _cmd_total, 0, AP_PARAM_FLAG_INTERNAL_USE_ONLY),

    // @Param: RESTART
    // @DisplayName: Mission Restart when entering Auto mode
    // @Description: Controls mission starting point when entering Auto mode (either restart from beginning of mission or resume from last command run)
    // @Values: 0:Resume Mission, 1:Restart Mission
    // @User: Advanced
    AP_GROUPINFO("RESTART",  1, AP_Mission, _restart, AP_MISSION_RESTART_DEFAULT),

    // @Param: OPTIONS
    // @DisplayName: Mission options bitmask
    // @Description: Bitmask of what options to use in missions.
    // @Bitmask: 0:Clear Mission on reboot, 1:Use distance to land calc on battery failsafe,2:ContinueAfterLand
    // @Bitmask{Copter}: 0:Clear Mission on reboot, 2:ContinueAfterLand
    // @Bitmask{Rover, Sub}: 0:Clear Mission on reboot
    // @User: Advanced
    AP_GROUPINFO("OPTIONS",  2, AP_Mission, _options, AP_MISSION_OPTIONS_DEFAULT),

    AP_GROUPEND
};

extern const AP_HAL::HAL& hal;

// storage object
StorageAccess AP_Mission::_storage(StorageManager::StorageMission);

HAL_Semaphore AP_Mission::_rsem;

///
/// public mission methods
///

/// init - initialises this library including checks the version in eeprom matches this library
void AP_Mission::init()
{
    // check_eeprom_version - checks version of missions stored in eeprom matches this library
    // command list will be cleared if they do not match
    check_eeprom_version();

    // initialize the jump tracking array
    init_jump_tracking();

    // If Mission Clear bit is set then it should clear the mission, otherwise retain the mission.
    if (AP_MISSION_MASK_MISSION_CLEAR & _options) {
        gcs().send_text(MAV_SEVERITY_INFO, "Clearing Mission");
        clear();
    }

    _last_change_time_ms = AP_HAL::millis();
}

/// start - resets current commands to point to the beginning of the mission
///     To-Do: should we validate the mission first and return true/false?
void AP_Mission::start()
{
    _flags.state = MISSION_RUNNING;

    reset(); // reset mission to the first command, resets jump tracking

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
///     previous running commands will be re-initialized
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
    }

    // ensure cache coherence
    if (!read_cmd_from_storage(_nav_cmd.index, _nav_cmd)) {
        // if we failed to read the command from storage, then the command may have
        // been from a previously loaded mission it is illogical to ever resume
        // flying to a command that has been excluded from the current mission
        start();
        return;
    }

    // rewind the mission wp if the repeat distance has been set via MAV_CMD_DO_SET_RESUME_REPEAT_DIST
    if (_repeat_dist > 0 && _wp_index_history[LAST_WP_PASSED] != AP_MISSION_CMD_INDEX_NONE) {
        // if not already in a resume state calculate the position to rewind to
        Mission_Command tmp_cmd;
        if (!_flags.resuming_mission && calc_rewind_pos(tmp_cmd)) {
            _resume_cmd = tmp_cmd;
        }

        // resume mission to rewound position
        if (_resume_cmd.index != AP_MISSION_CMD_INDEX_NONE && start_command(_resume_cmd)) {
            _nav_cmd = _resume_cmd;
            _flags.nav_cmd_loaded = true;
            // set flag to prevent history being re-written
            _flags.resuming_mission = true;
            return;
        }
    }

    // restart active navigation command. We run these on resume()
    // regardless of whether the mission was stopped, as we may be
    // re-entering AUTO mode and the nav_cmd callback needs to be run
    // to setup the current target waypoint

    if (_flags.do_cmd_loaded && _do_cmd.index != AP_MISSION_CMD_INDEX_NONE) {
        // restart the active do command, which will also load the nav command for us
        set_current_cmd(_do_cmd.index);
    } else if (_flags.nav_cmd_loaded) {
        // restart the active nav command
        set_current_cmd(_nav_cmd.index);
    }

    // Note: if there is no active command then the mission must have been stopped just after the previous nav command completed
    //      update will take care of finding and starting the nav command
}

/// check if the next nav command is a takeoff, skipping delays
bool AP_Mission::is_takeoff_next(uint16_t cmd_index)
{
    Mission_Command cmd = {};
    // check a maximum of 16 items, remembering that missions can have
    // loops in them
    for (uint8_t i=0; i<16; i++, cmd_index++) {
        if (!get_next_nav_cmd(cmd_index, cmd)) {
            return false;
        }
        switch (cmd.id) {
        // any of these are considered a takeoff command:
        case MAV_CMD_NAV_VTOL_TAKEOFF:
        case MAV_CMD_NAV_TAKEOFF:
        case MAV_CMD_NAV_TAKEOFF_LOCAL:
            return true;
        // any of these are considered "skippable" when determining if
        // we "start with a takeoff command"
        case MAV_CMD_DO_AUX_FUNCTION:
        case MAV_CMD_NAV_DELAY:
            continue;
        default:
            return false;
        }
    }
    return false;
}

/// check mission starts with a takeoff command
bool AP_Mission::starts_with_takeoff_cmd()
{
    uint16_t cmd_index = _restart ? AP_MISSION_CMD_INDEX_NONE : _nav_cmd.index;
    if (cmd_index == AP_MISSION_CMD_INDEX_NONE) {
        cmd_index = AP_MISSION_FIRST_REAL_COMMAND;
    }
    return is_takeoff_next(cmd_index);
}

/*
    return true if MIS_OPTIONS is set to allow continue of mission
    logic after a land and the next waypoint is a takeoff. If this
    is false then after a landing is complete the vehicle should 
    disarm and mission logic should stop
*/
bool AP_Mission::continue_after_land_check_for_takeoff()
{
    if (!continue_after_land()) {
        return false;
    }
    if (_nav_cmd.index == AP_MISSION_CMD_INDEX_NONE) {
        return false;
    }
    return is_takeoff_next(_nav_cmd.index+1);
}

/// start_or_resume - if MIS_AUTORESTART=0 this will call resume(), otherwise it will call start()
void AP_Mission::start_or_resume()
{
    if (_restart == 1 && !_force_resume) {
        start();
    } else {
        resume();
        _force_resume = false;
    }
}

/// reset - reset mission to the first command
void AP_Mission::reset()
{
    _flags.nav_cmd_loaded  = false;
    _flags.do_cmd_loaded   = false;
    _flags.do_cmd_all_done = false;
    _flags.in_landing_sequence = false;
    _nav_cmd.index         = AP_MISSION_CMD_INDEX_NONE;
    _do_cmd.index          = AP_MISSION_CMD_INDEX_NONE;
    _prev_nav_cmd_index    = AP_MISSION_CMD_INDEX_NONE;
    _prev_nav_cmd_wp_index = AP_MISSION_CMD_INDEX_NONE;
    _prev_nav_cmd_id       = AP_MISSION_CMD_ID_NONE;
    init_jump_tracking();
    reset_wp_history();
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
    truncate(0);

    // clear index to commands
    _nav_cmd.index = AP_MISSION_CMD_INDEX_NONE;
    _do_cmd.index = AP_MISSION_CMD_INDEX_NONE;
    _flags.nav_cmd_loaded = false;
    _flags.do_cmd_loaded = false;

    // return success
    return true;
}


/// trucate - truncate any mission items beyond index
void AP_Mission::truncate(uint16_t index)
{
    if ((unsigned)_cmd_total > index) {
        _cmd_total.set_and_save(index);
        _last_change_time_ms = AP_HAL::millis();
    }
}

/// update - ensures the command queues are loaded with the next command and calls main programs command_init and command_verify functions to progress the mission
///     should be called at 10hz or higher
void AP_Mission::update()
{
    // exit immediately if not running or no mission commands
    if (_flags.state != MISSION_RUNNING || _cmd_total == 0) {
        return;
    }

    update_exit_position();

    // save persistent waypoint_num for watchdog restore
    hal.util->persistent_data.waypoint_num = _nav_cmd.index;

    // check if we have an active nav command
    if (!_flags.nav_cmd_loaded || _nav_cmd.index == AP_MISSION_CMD_INDEX_NONE) {
        // advance in mission if no active nav command
        if (!advance_current_nav_cmd()) {
            // failure to advance nav command means mission has completed
            complete();
            return;
        }
    } else {
        // run the active nav command
        if (verify_command(_nav_cmd)) {
            // market _nav_cmd as complete (it will be started on the next iteration)
            _flags.nav_cmd_loaded = false;
            // immediately advance to the next mission command
            if (!advance_current_nav_cmd()) {
                // failure to advance nav command means mission has completed
                complete();
                return;
            }
        }
    }

    // check if we have an active do command
    if (!_flags.do_cmd_loaded) {
        advance_current_do_cmd();
    } else {
        // check the active do command
        if (verify_command(_do_cmd)) {
            // mark _do_cmd as complete
            _flags.do_cmd_loaded = false;
        }
    }
}

bool AP_Mission::verify_command(const Mission_Command& cmd)
{
    switch (cmd.id) {
    // do-commands always return true for verify:
#if AP_GRIPPER_ENABLED
    case MAV_CMD_DO_GRIPPER:
#endif
    case MAV_CMD_DO_SET_SERVO:
    case MAV_CMD_DO_SET_RELAY:
    case MAV_CMD_DO_REPEAT_SERVO:
    case MAV_CMD_DO_REPEAT_RELAY:
    case MAV_CMD_DO_DIGICAM_CONFIGURE:
    case MAV_CMD_DO_DIGICAM_CONTROL:
    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
    case MAV_CMD_DO_PARACHUTE:
    case MAV_CMD_DO_SEND_SCRIPT_MESSAGE:
    case MAV_CMD_DO_SPRAYER:
    case MAV_CMD_DO_AUX_FUNCTION:
    case MAV_CMD_DO_SET_RESUME_REPEAT_DIST:
    case MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW:
        return true;
    default:
        return _cmd_verify_fn(cmd);
    }
}

bool AP_Mission::start_command(const Mission_Command& cmd)
{
    // check for landing related commands and set in_landing_sequence flag
    if (is_landing_type_cmd(cmd.id) || cmd.id == MAV_CMD_DO_LAND_START) {
        set_in_landing_sequence_flag(true);
    } else if (is_takeoff_type_cmd(cmd.id)) {
        set_in_landing_sequence_flag(false);
    }

    gcs().send_text(MAV_SEVERITY_INFO, "Mission: %u %s", cmd.index, cmd.type());
    switch (cmd.id) {
    case MAV_CMD_DO_AUX_FUNCTION:
        return start_command_do_aux_function(cmd);
#if AP_GRIPPER_ENABLED
    case MAV_CMD_DO_GRIPPER:
        return start_command_do_gripper(cmd);
#endif
    case MAV_CMD_DO_SET_SERVO:
    case MAV_CMD_DO_SET_RELAY:
    case MAV_CMD_DO_REPEAT_SERVO:
    case MAV_CMD_DO_REPEAT_RELAY:
        return start_command_do_servorelayevents(cmd);
    case MAV_CMD_DO_CONTROL_VIDEO:
    case MAV_CMD_DO_DIGICAM_CONFIGURE:
    case MAV_CMD_DO_DIGICAM_CONTROL:
#if AP_CAMERA_ENABLED
    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
        return start_command_camera(cmd);
#endif
    case MAV_CMD_DO_PARACHUTE:
        return start_command_parachute(cmd);
    case MAV_CMD_DO_SEND_SCRIPT_MESSAGE:
        return start_command_do_scripting(cmd);
    case MAV_CMD_DO_SPRAYER:
        return start_command_do_sprayer(cmd);
    case MAV_CMD_DO_SET_RESUME_REPEAT_DIST:
        return command_do_set_repeat_dist(cmd);
    case MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW:
        return start_command_do_gimbal_manager_pitchyaw(cmd);
    default:
        return _cmd_start_fn(cmd);
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
bool AP_Mission::replace_cmd(uint16_t index, const Mission_Command& cmd)
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
    // NAV commands all have ids below MAV_CMD_NAV_LAST, plus some exceptions
    return (cmd.id <= MAV_CMD_NAV_LAST ||
            cmd.id == MAV_CMD_NAV_SET_YAW_SPEED ||
            cmd.id == MAV_CMD_NAV_SCRIPT_TIME ||
            cmd.id == MAV_CMD_NAV_ATTITUDE_TIME);
}

/// get_next_nav_cmd - gets next "navigation" command found at or after start_index
///     returns true if found, false if not found (i.e. reached end of mission command list)
///     accounts for do_jump commands but never increments the jump's num_times_run (advance_current_nav_cmd is responsible for this)
bool AP_Mission::get_next_nav_cmd(uint16_t start_index, Mission_Command& cmd)
{
    // search until the end of the mission command list
    for (uint16_t cmd_index = start_index; cmd_index < (unsigned)_cmd_total; cmd_index++) {
        // get next command
        if (!get_next_cmd(cmd_index, cmd, false)) {
            // no more commands so return failure
            return false;
        }
        // if found a "navigation" command then return it
        if (is_nav_cmd(cmd)) {
            return true;
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
    if (!get_next_nav_cmd(_nav_cmd.index+1, cmd)) {
        return default_angle;
    }
    // special handling for nav commands with no target location
    if (cmd.id == MAV_CMD_NAV_GUIDED_ENABLE ||
        cmd.id == MAV_CMD_NAV_DELAY) {
        return default_angle;
    }
    if (cmd.id == MAV_CMD_NAV_SET_YAW_SPEED) {
        return (_nav_cmd.content.set_yaw_speed.angle_deg * 100);
    }
    return _nav_cmd.content.location.get_bearing_to(cmd.content.location);
}

// set_current_cmd - jumps to command specified by index
bool AP_Mission::set_current_cmd(uint16_t index, bool rewind)
{
    // read command to check for DO_LAND_START
    Mission_Command cmd;
    if (!read_cmd_from_storage(index, cmd) || (cmd.id != MAV_CMD_DO_LAND_START)) {
        _flags.in_landing_sequence = false;
    }

    // mission command has been set and not as rewind command, don't track history.
    if (!rewind) {
        reset_wp_history();
    }

    // sanity check index and that we have a mission
    if (index >= (unsigned)_cmd_total || _cmd_total == 1) {
        return false;
    }

    // stop the current running do command
    _do_cmd.index = AP_MISSION_CMD_INDEX_NONE;
    _flags.do_cmd_loaded = false;
    _flags.do_cmd_all_done = false;

    // stop current nav cmd
    _flags.nav_cmd_loaded = false;

    // if index is zero then the user wants to completely restart the mission
    if (index == 0 || _flags.state == MISSION_COMPLETE) {
        _prev_nav_cmd_id    = AP_MISSION_CMD_ID_NONE;
        _prev_nav_cmd_index = AP_MISSION_CMD_INDEX_NONE;
        _prev_nav_cmd_wp_index = AP_MISSION_CMD_INDEX_NONE;
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
        while (!_flags.nav_cmd_loaded) {
            // get next command
            if (!get_next_cmd(index, cmd, true)) {
                _nav_cmd.index = AP_MISSION_CMD_INDEX_NONE;
                return false;
            }

            // check if navigation or "do" command
            if (is_nav_cmd(cmd)) {
                // set current navigation command
                _nav_cmd = cmd;
                _flags.nav_cmd_loaded = true;
            } else {
                // set current do command
                if (!_flags.do_cmd_loaded) {
                    _do_cmd = cmd;
                    _flags.do_cmd_loaded = true;
                }
            }
            // move onto next command
            index = cmd.index+1;
        }

        // if we have not found a do command then set flag to show there are no do-commands to be run before nav command completes
        if (!_flags.do_cmd_loaded) {
            _flags.do_cmd_all_done = true;
        }

        // if we got this far then the mission can safely be "resumed" from the specified index so we set the state to "stopped"
        _flags.state = MISSION_STOPPED;
        return true;
    }

    // the state must be MISSION_RUNNING, allow advance_current_nav_cmd() to manage starting the item
    if (!advance_current_nav_cmd(index)) {
        // on failure set mission complete
        complete();
        return false;
    }

    // if we got this far we must have successfully advanced the nav command
    return true;
}

// restart current navigation command.  Used to handle external changes to mission
// returns true on success, false if mission is not running or current nav command is invalid
bool AP_Mission::restart_current_nav_cmd()
{
    // return immediately if mission is not running
    if (_flags.state != MISSION_RUNNING) {
        return false;
    }

    // return immediately if nav command index is invalid
    const uint16_t nav_cmd_index = get_current_nav_index();
    if ((nav_cmd_index == 0) || (nav_cmd_index >= num_commands())) {
        return false;
    }

    return set_current_cmd(_nav_cmd.index);
}

// returns false on any issue at all.
bool AP_Mission::set_item(uint16_t index, mavlink_mission_item_int_t& src_packet)
{
    // this is the on-storage format
    AP_Mission::Mission_Command cmd;

    // can't handle request for anything bigger than the mission size+1...
    if (index > num_commands()) {
        return false;
    }

    // convert from mavlink-ish format to storage format, if we can.
    if (mavlink_int_to_mission_cmd(src_packet, cmd) != MAV_MISSION_ACCEPTED) {
        return false;
    }

    // A request to set the 'next' item after the end is how we add an extra
    //  item to the list, thus allowing us to write entire missions if needed.
    if (index == num_commands()) {
        return add_cmd(cmd);
    }

    // replacing an existing mission item...
    return AP_Mission::replace_cmd(index, cmd);
}

bool AP_Mission::get_item(uint16_t index, mavlink_mission_item_int_t& ret_packet) const
{
    // setting ret_packet.command = -1  and/or returning false
    //  means it contains invalid data after it leaves here.

    // this is the on-storage format
    AP_Mission::Mission_Command cmd;

    // can't handle request for anything bigger than the mission size...
    if (index >= num_commands()) {
        ret_packet.command = -1;
        return false;
    }

    // minimal placeholder values during read-from-storage
    ret_packet.target_system = 1;     // unused sysid
    ret_packet.target_component =  1; // unused compid

    // 0=home, higher number/s = mission item number.
    ret_packet.seq = index;

    // retrieve mission from eeprom
    if (!read_cmd_from_storage(ret_packet.seq, cmd)) {
        ret_packet.command = -1;
        return false;
    }
    // convert into mavlink-ish format for lua and friends.
    if (!mission_cmd_to_mavlink_int(cmd, ret_packet)) {
        ret_packet.command = -1;
        return false;
    }

    // set packet's current field to 1 if this is the command being executed
    if (cmd.id == (uint16_t)get_current_nav_cmd().index) {
        ret_packet.current = 1;
    } else {
        ret_packet.current = 0;
    }

    // set auto continue to 1, becasue that's what's done elsewhere.
    ret_packet.autocontinue = 1;     // 1 (true), 0 (false)
    ret_packet.command = cmd.id;

    return true;
}


struct PACKED Packed_Location_Option_Flags {
    uint8_t relative_alt : 1;           // 1 if altitude is relative to home
    uint8_t unused1      : 1;           // unused flag (defined so that loiter_ccw uses the correct bit)
    uint8_t loiter_ccw   : 1;           // 0 if clockwise, 1 if counter clockwise
    uint8_t terrain_alt  : 1;           // this altitude is above terrain
    uint8_t origin_alt   : 1;           // this altitude is above ekf origin
    uint8_t loiter_xtrack : 1;          // 0 to crosstrack from center of waypoint, 1 to crosstrack from tangent exit location
    uint8_t type_specific_bit_0 : 1;    // each mission item type can use this for storing 1 bit of extra data
};

struct PACKED PackedLocation {
    union {
        Packed_Location_Option_Flags flags;                    ///< options bitmask (1<<0 = relative altitude)
        uint8_t options;                                /// allows writing all flags to eeprom as one byte
    };
    // by making alt 24 bit we can make p1 in a command 16 bit,
    // allowing an accurate angle in centi-degrees. This keeps the
    // storage cost per mission item at 15 bytes, and allows mission
    // altitudes of up to +/- 83km
    int32_t alt:24;                                     ///< param 2 - Altitude in centimeters (meters * 100) see LOCATION_ALT_MAX_M
    int32_t lat;                                        ///< param 3 - Latitude * 10**7
    int32_t lng;                                        ///< param 4 - Longitude * 10**7
};

union PackedContent {
    // location
    PackedLocation location;      // Waypoint location

    // raw bytes, for reading/writing to eeprom. Note that only 10
    // bytes are available if a 16 bit command ID is used
    uint8_t bytes[12];

};

assert_storage_size<PackedContent, 12> assert_storage_size_PackedContent;

/// load_cmd_from_storage - load command from storage
///     true is return if successful
bool AP_Mission::read_cmd_from_storage(uint16_t index, Mission_Command& cmd) const
{
    WITH_SEMAPHORE(_rsem);

    // special handling for command #0 which is home
    if (index == 0) {
        cmd = {};
        cmd.id = MAV_CMD_NAV_WAYPOINT;
        cmd.content.location = AP::ahrs().get_home();
        return true;
    }

    if (index >= (unsigned)_cmd_total) {
        return false;
    }

    // ensure all bytes of cmd are zeroed
    cmd = {};

    // Find out proper location in memory by using the start_byte position + the index
    // we can load a command, we don't process it yet
    // read WP position
    const uint16_t pos_in_storage = 4 + (index * AP_MISSION_EEPROM_COMMAND_SIZE);

    PackedContent packed_content {};

    const uint8_t b1 = _storage.read_byte(pos_in_storage);
    if (b1 == 0 || b1 == 1) {
        cmd.id = _storage.read_uint16(pos_in_storage+1);
        cmd.p1 = _storage.read_uint16(pos_in_storage+3);
        _storage.read_block(packed_content.bytes, pos_in_storage+5, 10);
        format_conversion(b1, cmd, packed_content);
    } else {
        cmd.id = b1;
        cmd.p1 = _storage.read_uint16(pos_in_storage+1);
        _storage.read_block(packed_content.bytes, pos_in_storage+3, 12);
    }

    if (stored_in_location(cmd.id)) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        // NOTE!  no 16-bit command may be stored_in_location as only
        // 10 bytes are available for storage and lat/lon/alt required
        // 4*sizeof(float) == 12 bytes of storage.
        if (b1 == 0) {
            AP_HAL::panic("May not store location for 16-bit commands");
        }
#endif
        // Location is not PACKED; field-wise copy it:
        cmd.content.location.relative_alt = packed_content.location.flags.relative_alt;
        cmd.content.location.loiter_ccw = packed_content.location.flags.loiter_ccw;
        cmd.content.location.terrain_alt = packed_content.location.flags.terrain_alt;
        cmd.content.location.origin_alt = packed_content.location.flags.origin_alt;
        cmd.content.location.loiter_xtrack = packed_content.location.flags.loiter_xtrack;
        cmd.content.location.alt = packed_content.location.alt;
        cmd.content.location.lat = packed_content.location.lat;
        cmd.content.location.lng = packed_content.location.lng;

        if (packed_content.location.flags.type_specific_bit_0) {
            cmd.type_specific_bits = 1U << 0;
        }
    } else {
        // all other options in Content are assumed to be packed:
        static_assert(sizeof(cmd.content) >= 12,
                      "content is big enough to take bytes");
        // (void *) cast to specify gcc that we know that we are copy byte into a non trivial type and leaving 4 bytes untouched
        memcpy((void *)&cmd.content, packed_content.bytes, 12);
    }

    // set command's index to it's position in eeprom
    cmd.index = index;

    // return success
    return true;
}

bool AP_Mission::stored_in_location(uint16_t id)
{
    switch (id) {
    case MAV_CMD_NAV_WAYPOINT:
    case MAV_CMD_NAV_LOITER_UNLIM:
    case MAV_CMD_NAV_LOITER_TURNS:
    case MAV_CMD_NAV_LOITER_TIME:
    case MAV_CMD_NAV_LAND:
    case MAV_CMD_NAV_TAKEOFF:
    case MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
    case MAV_CMD_NAV_LOITER_TO_ALT:
    case MAV_CMD_NAV_SPLINE_WAYPOINT:
    case MAV_CMD_NAV_GUIDED_ENABLE:
    case MAV_CMD_DO_SET_HOME:
    case MAV_CMD_DO_LAND_START:
    case MAV_CMD_DO_GO_AROUND:
    case MAV_CMD_DO_SET_ROI:
    case MAV_CMD_NAV_VTOL_TAKEOFF:
    case MAV_CMD_NAV_VTOL_LAND:
    case MAV_CMD_NAV_PAYLOAD_PLACE:
        return true;
    default:
        return false;
    }
}

/// write_cmd_to_storage - write a command to storage
///     index is used to calculate the storage location
///     true is returned if successful
bool AP_Mission::write_cmd_to_storage(uint16_t index, const Mission_Command& cmd)
{
    WITH_SEMAPHORE(_rsem);

    // range check cmd's index
    if (index >= num_commands_max()) {
        return false;
    }

    PackedContent packed {};
    if (stored_in_location(cmd.id)) {
        // Location is not PACKED; field-wise copy it:
        packed.location.flags.relative_alt = cmd.content.location.relative_alt;
        packed.location.flags.loiter_ccw = cmd.content.location.loiter_ccw;
        packed.location.flags.terrain_alt = cmd.content.location.terrain_alt;
        packed.location.flags.origin_alt = cmd.content.location.origin_alt;
        packed.location.flags.loiter_xtrack = cmd.content.location.loiter_xtrack;
        packed.location.flags.type_specific_bit_0 = cmd.type_specific_bits & (1U<<0);
        packed.location.alt = cmd.content.location.alt;
        packed.location.lat = cmd.content.location.lat;
        packed.location.lng = cmd.content.location.lng;
    } else {
        // all other options in Content are assumed to be packed:
        static_assert(sizeof(packed.bytes) >= 12,
                      "packed.bytes is big enough to take content");
        memcpy(packed.bytes, &cmd.content, 12);
    }

    // calculate where in storage the command should be placed
    uint16_t pos_in_storage = 4 + (index * AP_MISSION_EEPROM_COMMAND_SIZE);

    if (cmd.id < 256) {
        // for commands below 256 we store up to 12 bytes
        _storage.write_byte(pos_in_storage, cmd.id);
        _storage.write_uint16(pos_in_storage+1, cmd.p1);
        _storage.write_block(pos_in_storage+3, packed.bytes, 12);
    } else {
        // if the command ID is above 256 we store a tag byte followed
        // by the 16 bit command ID. The tag byte is 1 for commands
        // where we have changed the storage format (see
        // format_conversion), 0 otherwise
        uint8_t tag_byte = 0;
        // currently the only converted structure is NAV_SCRIPT_TIME
        if (cmd.id == MAV_CMD_NAV_SCRIPT_TIME) {
            tag_byte = 1;
        }
        _storage.write_byte(pos_in_storage, tag_byte);
        _storage.write_uint16(pos_in_storage+1, cmd.id);
        _storage.write_uint16(pos_in_storage+3, cmd.p1);
        _storage.write_block(pos_in_storage+5, packed.bytes, 10);
    }

    // remember when the mission last changed
    _last_change_time_ms = AP_HAL::millis();

    // return success
    return true;
}

/// write_home_to_storage - writes the special purpose cmd 0 (home) to storage
///     home is taken directly from ahrs
void AP_Mission::write_home_to_storage()
{
    Mission_Command home_cmd = {};
    home_cmd.id = MAV_CMD_NAV_WAYPOINT;
    home_cmd.content.location = AP::ahrs().get_home();
    write_cmd_to_storage(0,home_cmd);
}

MAV_MISSION_RESULT AP_Mission::sanity_check_params(const mavlink_mission_item_int_t& packet)
{
    uint8_t nan_mask;
    switch (packet.command) {
    case MAV_CMD_NAV_WAYPOINT:
        nan_mask = ~(1 << 3); // param 4 can be nan
        break;
    case MAV_CMD_NAV_LAND:
        nan_mask = ~(1 << 3); // param 4 can be nan
        break;
    case MAV_CMD_NAV_TAKEOFF:
        nan_mask = ~(1 << 3); // param 4 can be nan
        break;
    case MAV_CMD_NAV_VTOL_TAKEOFF:
        nan_mask = ~(1 << 3); // param 4 can be nan
        break;
    case MAV_CMD_NAV_VTOL_LAND:
        nan_mask = ~((1 << 2) | (1 << 3)); // param 3 and 4 can be nan
        break;
    default:
        nan_mask = 0xff;
        break;
    }

    if (((nan_mask & (1 << 0)) && isnan(packet.param1)) ||
        isinf(packet.param1)) {
        return MAV_MISSION_INVALID_PARAM1;
    }
    if (((nan_mask & (1 << 1)) && isnan(packet.param2)) ||
        isinf(packet.param2)) {
        return MAV_MISSION_INVALID_PARAM2;
    }
    if (((nan_mask & (1 << 2)) && isnan(packet.param3)) ||
        isinf(packet.param3)) {
        return MAV_MISSION_INVALID_PARAM3;
    }
    if (((nan_mask & (1 << 3)) && isnan(packet.param4)) ||
        isinf(packet.param4)) {
        return MAV_MISSION_INVALID_PARAM4;
    }
    return MAV_MISSION_ACCEPTED;
}

// mavlink_int_to_mission_cmd - converts mavlink message to an AP_Mission::Mission_Command object which can be stored to eeprom
//  return MAV_MISSION_ACCEPTED on success, MAV_MISSION_RESULT error on failure
MAV_MISSION_RESULT AP_Mission::mavlink_int_to_mission_cmd(const mavlink_mission_item_int_t& packet, AP_Mission::Mission_Command& cmd)
{
    // command's position in mission list and mavlink id
    cmd.index = packet.seq;
    cmd.id = packet.command;
    cmd.content.location = {};

    MAV_MISSION_RESULT param_check = sanity_check_params(packet);
    if (param_check != MAV_MISSION_ACCEPTED) {
        return param_check;
    }

    // command specific conversions from mavlink packet to mission command
    switch (cmd.id) {

    case 0 ... 1:
        // these are reserved for storing 16 bit command IDs
        return MAV_MISSION_INVALID;

    case MAV_CMD_NAV_WAYPOINT: {                        // MAV ID: 16
        /*
          the 15 byte limit means we can't fit both delay and radius
          in the cmd structure. When we expand the mission structure
          we can do this properly
         */
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
        // acceptance radius in meters and pass by distance in meters
        uint16_t acp = packet.param2;           // param 2 is acceptance radius in meters is held in low p1
        uint16_t passby = packet.param3;        // param 3 is pass by distance in meters is held in high p1

        // limit to 255 so it does not wrap during the shift or mask operation
        passby = MIN(0xFF,passby);
        acp = MIN(0xFF,acp);

        cmd.p1 = (passby << 8) | (acp & 0x00FF);
#else
        // delay at waypoint in seconds (this is for copters???)
        cmd.p1 = packet.param1;
#endif
    }
    break;

    case MAV_CMD_NAV_LOITER_UNLIM:                      // MAV ID: 17
        cmd.p1 = fabsf(packet.param3);                  // store radius as 16bit since no other params are competing for space
        cmd.content.location.loiter_ccw = (packet.param3 < 0);    // -1 = counter clockwise, +1 = clockwise
        break;

    case MAV_CMD_NAV_LOITER_TURNS: {                    // MAV ID: 18
        // number of turns is stored in the lowest bits.  radii below
        // 255m are stored in the top 8 bits as an 8-bit integer.
        // Radii above 255m are stored divided by 10 and a bit set in
        // storage so that on retrieval they are multiplied by 10
        cmd.p1 = MIN(255, packet.param1); // store number of times to circle in low p1
        uint8_t radius_m;
        const float abs_radius = fabsf(packet.param3);
        if (abs_radius <= 255) {
            radius_m = abs_radius;
        } else {
            radius_m = MIN(255, abs_radius * 0.1);
            cmd.type_specific_bits = 1U << 0;
        }
        cmd.p1 |= (radius_m<<8);   // store radius in high byte of p1
        cmd.content.location.loiter_ccw = (packet.param3 < 0);
        cmd.content.location.loiter_xtrack = (packet.param4 > 0); // 0 to xtrack from center of waypoint, 1 to xtrack from tangent exit location
    }
    break;

    case MAV_CMD_NAV_LOITER_TIME:                       // MAV ID: 19
        cmd.p1 = packet.param1;                         // loiter time in seconds uses all 16 bits, 8bit seconds is too small. No room for radius.
        cmd.content.location.loiter_ccw = (packet.param3 < 0);
        cmd.content.location.loiter_xtrack = (packet.param4 > 0); // 0 to xtrack from center of waypoint, 1 to xtrack from tangent exit location
        break;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:                  // MAV ID: 20
        break;

    case MAV_CMD_NAV_LAND:                              // MAV ID: 21
        cmd.p1 = packet.param1;                         // abort target altitude(m)  (plane only)
        if (!isnan(packet.param4)) {
            cmd.content.location.loiter_ccw = is_negative(packet.param4); // yaw direction, (plane deepstall only)
        }
        break;

    case MAV_CMD_NAV_TAKEOFF:                           // MAV ID: 22
        cmd.p1 = packet.param1;                         // minimum pitch (plane only)
        break;

    case MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:           // MAV ID: 30
        cmd.p1 = packet.param1;                         // Climb/Descend
        // 0 = Neutral, cmd complete at +/- 5 of indicated alt.
        // 1 = Climb, cmd complete at or above indicated alt.
        // 2 = Descend, cmd complete at or below indicated alt.
        break;

    case MAV_CMD_NAV_LOITER_TO_ALT:                     // MAV ID: 31
        cmd.p1 = fabsf(packet.param2);                  // param2 is radius in meters
        cmd.content.location.loiter_ccw = (packet.param2 < 0);
        cmd.content.location.loiter_xtrack = (packet.param4 > 0); // 0 to xtrack from center of waypoint, 1 to xtrack from tangent exit location
        break;

    case MAV_CMD_NAV_SPLINE_WAYPOINT:                   // MAV ID: 82
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
        return MAV_MISSION_UNSUPPORTED;
#else
        cmd.p1 = packet.param1;                         // delay at waypoint in seconds
        break;
#endif

    case MAV_CMD_NAV_GUIDED_ENABLE:                     // MAV ID: 92
        cmd.p1 = packet.param1;                         // on/off. >0.5 means "on", hand-over control to external controller
        break;

    case MAV_CMD_NAV_DELAY:                            // MAV ID: 93
        cmd.content.nav_delay.seconds = packet.param1; // delay in seconds
        cmd.content.nav_delay.hour_utc = packet.param2;// absolute time's hour (utc)
        cmd.content.nav_delay.min_utc = packet.param3;// absolute time's min (utc)
        cmd.content.nav_delay.sec_utc = packet.param4; // absolute time's second (utc)
        break;

    case MAV_CMD_CONDITION_DELAY:                       // MAV ID: 112
        cmd.content.delay.seconds = packet.param1;      // delay in seconds
        break;

    case MAV_CMD_CONDITION_DISTANCE:                    // MAV ID: 114
        cmd.content.distance.meters = packet.param1;    // distance in meters from next waypoint
        break;

    case MAV_CMD_CONDITION_YAW:                         // MAV ID: 115
        cmd.content.yaw.angle_deg = packet.param1;      // target angle in degrees
        cmd.content.yaw.turn_rate_dps = packet.param2;  // 0 = use default turn rate otherwise specific turn rate in deg/sec
        cmd.content.yaw.direction = packet.param3;      // -1 = ccw, +1 = cw
        cmd.content.yaw.relative_angle = packet.param4; // lng=0: absolute angle provided, lng=1: relative angle provided
        break;

    case MAV_CMD_DO_JUMP:                               // MAV ID: 177
        cmd.content.jump.target = packet.param1;        // jump-to command number
        cmd.content.jump.num_times = packet.param2;     // repeat count
        break;

    case MAV_CMD_DO_CHANGE_SPEED:                       // MAV ID: 178
        cmd.content.speed.speed_type = packet.param1;   // 0 = airspeed, 1 = ground speed
        cmd.content.speed.target_ms = packet.param2;    // target speed in m/s
        cmd.content.speed.throttle_pct = packet.param3; // throttle as a percentage from 1 ~ 100%
        break;

    case MAV_CMD_DO_SET_HOME:
        cmd.p1 = packet.param1;                         // p1=0 means use current location, p=1 means use provided location
        break;

    case MAV_CMD_DO_SET_RELAY:                          // MAV ID: 181
        cmd.content.relay.num = packet.param1;          // relay number
        cmd.content.relay.state = packet.param2;        // 0:off, 1:on
        break;

    case MAV_CMD_DO_REPEAT_RELAY:                       // MAV ID: 182
        cmd.content.repeat_relay.num = packet.param1;           // relay number
        cmd.content.repeat_relay.repeat_count = packet.param2;  // count
        cmd.content.repeat_relay.cycle_time = packet.param3;    // time converted from seconds to milliseconds
        break;

    case MAV_CMD_DO_SET_SERVO:                          // MAV ID: 183
        cmd.content.servo.channel = packet.param1;      // channel
        cmd.content.servo.pwm = packet.param2;          // PWM
        break;

    case MAV_CMD_DO_REPEAT_SERVO:                       // MAV ID: 184
        cmd.content.repeat_servo.channel = packet.param1;      // channel
        cmd.content.repeat_servo.pwm = packet.param2;          // PWM
        cmd.content.repeat_servo.repeat_count = packet.param3; // count
        cmd.content.repeat_servo.cycle_time = packet.param4;   // time in seconds
        break;

    case MAV_CMD_DO_LAND_START:                         // MAV ID: 189
        break;

    case MAV_CMD_DO_GO_AROUND:                          // MAV ID: 191
        break;

    case MAV_CMD_DO_SET_ROI:                            // MAV ID: 201
        cmd.p1 = packet.param1;                         // 0 = no roi, 1 = next waypoint, 2 = waypoint number, 3 = fixed location, 4 = given target (not supported)
        break;

    case MAV_CMD_DO_DIGICAM_CONFIGURE:                  // MAV ID: 202
        cmd.content.digicam_configure.shooting_mode = packet.param1;
        cmd.content.digicam_configure.shutter_speed = packet.param2;
        cmd.content.digicam_configure.aperture = packet.param3;
        cmd.content.digicam_configure.ISO = packet.param4;
        cmd.content.digicam_configure.exposure_type = packet.x;
        cmd.content.digicam_configure.cmd_id = packet.y;
        cmd.content.digicam_configure.engine_cutoff_time = packet.z;
        break;

    case MAV_CMD_DO_DIGICAM_CONTROL:                    // MAV ID: 203
        cmd.content.digicam_control.session = packet.param1;
        cmd.content.digicam_control.zoom_pos = packet.param2;
        cmd.content.digicam_control.zoom_step = packet.param3;
        cmd.content.digicam_control.focus_lock = packet.param4;
        cmd.content.digicam_control.shooting_cmd = packet.x;
        cmd.content.digicam_control.cmd_id = packet.y;
        break;

    case MAV_CMD_DO_MOUNT_CONTROL:                      // MAV ID: 205
        cmd.content.mount_control.pitch = packet.param1;
        cmd.content.mount_control.roll = packet.param2;
        cmd.content.mount_control.yaw = packet.param3;
        break;

    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:                 // MAV ID: 206
        cmd.content.cam_trigg_dist.meters = packet.param1;  // distance between camera shots in meters
        cmd.content.cam_trigg_dist.trigger = packet.param3; // when enabled, camera triggers once immediately
        break;

    case MAV_CMD_DO_FENCE_ENABLE:                       // MAV ID: 207
        cmd.p1 = packet.param1;                         // action 0=disable, 1=enable
        break;

    case MAV_CMD_DO_AUX_FUNCTION:
        cmd.content.auxfunction.function = packet.param1;
        cmd.content.auxfunction.switchpos = packet.param2;
        break;

    case MAV_CMD_DO_PARACHUTE:                         // MAV ID: 208
        cmd.p1 = packet.param1;                        // action 0=disable, 1=enable, 2=release.  See PARACHUTE_ACTION enum
        break;

    case MAV_CMD_DO_INVERTED_FLIGHT:                    // MAV ID: 210
        cmd.p1 = packet.param1;                         // normal=0 inverted=1
        break;

#if AP_GRIPPER_ENABLED
    case MAV_CMD_DO_GRIPPER:                            // MAV ID: 211
        cmd.content.gripper.num = packet.param1;        // gripper number
        cmd.content.gripper.action = packet.param2;     // action 0=release, 1=grab.  See GRIPPER_ACTION enum
        break;
#endif

    case MAV_CMD_DO_GUIDED_LIMITS:                      // MAV ID: 222
        cmd.p1 = packet.param1;                         // max time in seconds the external controller will be allowed to control the vehicle
        cmd.content.guided_limits.alt_min = packet.param2;  // min alt below which the command will be aborted.  0 for no lower alt limit
        cmd.content.guided_limits.alt_max = packet.param3;  // max alt above which the command will be aborted.  0 for no upper alt limit
        cmd.content.guided_limits.horiz_max = packet.param4;// max horizontal distance the vehicle can move before the command will be aborted.  0 for no horizontal limit
        break;

    case MAV_CMD_DO_AUTOTUNE_ENABLE:                    // MAV ID: 211
        cmd.p1 = packet.param1;                         // disable=0 enable=1
        break;

    case MAV_CMD_NAV_ALTITUDE_WAIT:                     // MAV ID: 83
        cmd.content.altitude_wait.altitude = packet.param1;
        cmd.content.altitude_wait.descent_rate = packet.param2;
        cmd.content.altitude_wait.wiggle_time = packet.param3;
        break;

    case MAV_CMD_NAV_VTOL_TAKEOFF:
        break;

    case MAV_CMD_NAV_VTOL_LAND:
        cmd.p1 = (NAV_VTOL_LAND_OPTIONS)packet.param1;
        break;

    case MAV_CMD_DO_VTOL_TRANSITION:
        cmd.content.do_vtol_transition.target_state = packet.param1;
        break;

    case MAV_CMD_DO_SET_REVERSE:
        cmd.p1 = packet.param1; // 0 = forward, 1 = reverse
        break;

    case MAV_CMD_DO_ENGINE_CONTROL:
        cmd.content.do_engine_control.start_control = (packet.param1>0);
        cmd.content.do_engine_control.cold_start = (packet.param2>0);
        cmd.content.do_engine_control.height_delay_cm = packet.param3*100;
        break;

    case MAV_CMD_NAV_PAYLOAD_PLACE:
        cmd.p1 = packet.param1*100; // copy max-descend parameter (m->cm)
        break;

    case MAV_CMD_NAV_SET_YAW_SPEED:
        cmd.content.set_yaw_speed.angle_deg = packet.param1;        // target angle in degrees
        cmd.content.set_yaw_speed.speed = packet.param2;            // speed in meters/second
        cmd.content.set_yaw_speed.relative_angle = packet.param3;   // 0 = absolute angle, 1 = relative angle
        break;

    case MAV_CMD_DO_WINCH:                              // MAV ID: 42600
        cmd.content.winch.num = packet.param1;          // winch number
        cmd.content.winch.action = packet.param2;       // action (0 = relax, 1 = length control, 2 = rate control).  See WINCH_ACTION enum
        cmd.content.winch.release_length = packet.param3;   // cable distance to unwind in meters, negative numbers to wind in cable
        cmd.content.winch.release_rate = packet.param4; // release rate in meters/second
        break;

    case MAV_CMD_DO_SET_RESUME_REPEAT_DIST:
        cmd.p1 = packet.param1; // Resume repeat distance (m)
        break;

    case MAV_CMD_DO_SPRAYER:
        cmd.p1 = packet.param1;                        // action 0=disable, 1=enable
        break;

    case MAV_CMD_DO_SEND_SCRIPT_MESSAGE:
        cmd.p1 = packet.param1;
        cmd.content.scripting.p1 = packet.param2;
        cmd.content.scripting.p2 = packet.param3;
        cmd.content.scripting.p3 = packet.param4;
        break;

#if AP_SCRIPTING_ENABLED
    case MAV_CMD_NAV_SCRIPT_TIME:
        cmd.content.nav_script_time.command = packet.param1;
        cmd.content.nav_script_time.timeout_s = packet.param2;
        cmd.content.nav_script_time.arg1.set(packet.param3);
        cmd.content.nav_script_time.arg2.set(packet.param4);
        cmd.content.nav_script_time.arg3 = int16_t(packet.x);
        cmd.content.nav_script_time.arg4 = int16_t(packet.y);
        break;
#endif

    case MAV_CMD_NAV_ATTITUDE_TIME:
        cmd.content.nav_attitude_time.time_sec = constrain_float(packet.param1, 0, UINT16_MAX);
        cmd.content.nav_attitude_time.roll_deg = (fabsf(packet.param2) <= 180) ? packet.param2 : 0;
        cmd.content.nav_attitude_time.pitch_deg = (fabsf(packet.param3) <= 90) ? packet.param3 : 0;
        cmd.content.nav_attitude_time.yaw_deg = ((packet.param4 >= -180) && (packet.param4 <= 360)) ? packet.param4 : 0;
        cmd.content.nav_attitude_time.climb_rate = packet.x;
        break;

    case MAV_CMD_DO_PAUSE_CONTINUE:
        cmd.p1 = packet.param1;
        break;

    case MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW:
        cmd.content.gimbal_manager_pitchyaw.pitch_angle_deg = packet.param1;
        cmd.content.gimbal_manager_pitchyaw.yaw_angle_deg = packet.param2;
        cmd.content.gimbal_manager_pitchyaw.pitch_rate_degs = packet.param3;
        cmd.content.gimbal_manager_pitchyaw.yaw_rate_degs = packet.param4;
        cmd.content.gimbal_manager_pitchyaw.flags = packet.x;
        cmd.content.gimbal_manager_pitchyaw.gimbal_id = packet.z;
        break;

    default:
        // unrecognised command
        return MAV_MISSION_UNSUPPORTED;
    }

    // copy location from mavlink to command
    if (stored_in_location(cmd.id)) {

        // sanity check location
        if (!check_lat(packet.x)) {
            return MAV_MISSION_INVALID_PARAM5_X;
        }
        if (!check_lng(packet.y)) {
            return MAV_MISSION_INVALID_PARAM6_Y;
        }
        if (isnan(packet.z) || fabsf(packet.z) >= LOCATION_ALT_MAX_M) {
            return MAV_MISSION_INVALID_PARAM7;
        }

        cmd.content.location.lat = packet.x;
        cmd.content.location.lng = packet.y;

        cmd.content.location.alt = packet.z * 100.0f;       // convert packet's alt (m) to cmd alt (cm)

        switch (packet.frame) {

        case MAV_FRAME_MISSION:
        case MAV_FRAME_GLOBAL:
        case MAV_FRAME_GLOBAL_INT:
            cmd.content.location.relative_alt = 0;
            break;

        case MAV_FRAME_GLOBAL_RELATIVE_ALT:
        case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
            cmd.content.location.relative_alt = 1;
            break;

#if AP_TERRAIN_AVAILABLE
        case MAV_FRAME_GLOBAL_TERRAIN_ALT:
        case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
            // we mark it as a relative altitude, as it doesn't have
            // home alt added
            cmd.content.location.relative_alt = 1;
            // mark altitude as above terrain, not above home
            cmd.content.location.terrain_alt = 1;
            break;
#endif

        default:
            return MAV_MISSION_UNSUPPORTED_FRAME;
        }
    }

    // if we got this far then it must have been successful
    return MAV_MISSION_ACCEPTED;
}

MAV_MISSION_RESULT AP_Mission::convert_MISSION_ITEM_to_MISSION_ITEM_INT(const mavlink_mission_item_t &packet,
        mavlink_mission_item_int_t &mav_cmd)
{
    // TODO: rename mav_cmd to mission_item_int
    // TODO: rename packet to mission_item
    mav_cmd.param1 = packet.param1;
    mav_cmd.param2 = packet.param2;
    mav_cmd.param3 = packet.param3;
    mav_cmd.param4 = packet.param4;
    mav_cmd.z = packet.z;
    mav_cmd.seq = packet.seq;
    mav_cmd.command = packet.command;
    mav_cmd.target_system = packet.target_system;
    mav_cmd.target_component = packet.target_component;
    mav_cmd.frame = packet.frame;
    mav_cmd.current = packet.current;
    mav_cmd.autocontinue = packet.autocontinue;
    mav_cmd.mission_type = packet.mission_type;

    /*
      the strategy for handling both MISSION_ITEM and MISSION_ITEM_INT
      is to pass the lat/lng in MISSION_ITEM_INT straight through, and
      for MISSION_ITEM multiply by 1e7 here. We need an exception for
      any commands which use the x and y fields not as
      latitude/longitude.
     */
    switch (packet.command) {
    case MAV_CMD_DO_DIGICAM_CONTROL:
    case MAV_CMD_DO_DIGICAM_CONFIGURE:
    case MAV_CMD_NAV_ATTITUDE_TIME:
    case MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW:
        mav_cmd.x = packet.x;
        mav_cmd.y = packet.y;
        break;

    default:
        // all other commands use x and y as lat/lon. We need to
        // multiply by 1e7 to convert to int32_t
        if (!check_lat(packet.x)) {
            return MAV_MISSION_INVALID_PARAM5_X;
        }
        if (!check_lng(packet.y)) {
            return MAV_MISSION_INVALID_PARAM6_Y;
        }
        mav_cmd.x = packet.x * 1.0e7f;
        mav_cmd.y = packet.y * 1.0e7f;
        break;
    }

    return MAV_MISSION_ACCEPTED;
}

MAV_MISSION_RESULT AP_Mission::convert_MISSION_ITEM_INT_to_MISSION_ITEM(const mavlink_mission_item_int_t &item_int,
        mavlink_mission_item_t &item)
{
    item.param1 = item_int.param1;
    item.param2 = item_int.param2;
    item.param3 = item_int.param3;
    item.param4 = item_int.param4;
    item.z = item_int.z;
    item.seq = item_int.seq;
    item.command = item_int.command;
    item.target_system = item_int.target_system;
    item.target_component = item_int.target_component;
    item.frame = item_int.frame;
    item.current = item_int.current;
    item.autocontinue = item_int.autocontinue;
    item.mission_type = item_int.mission_type;

    switch (item_int.command) {
    case MAV_CMD_DO_DIGICAM_CONTROL:
    case MAV_CMD_DO_DIGICAM_CONFIGURE:
    case MAV_CMD_NAV_ATTITUDE_TIME:
    case MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW:
        item.x = item_int.x;
        item.y = item_int.y;
        break;

    default:
        // all other commands use x and y as lat/lon. We need to
        // multiply by 1e-7 to convert to float
        item.x = item_int.x * 1.0e-7f;
        item.y = item_int.y * 1.0e-7f;
        if (!check_lat(item.x)) {
            return MAV_MISSION_INVALID_PARAM5_X;
        }
        if (!check_lng(item.y)) {
            return MAV_MISSION_INVALID_PARAM6_Y;
        }
        break;
    }

    return MAV_MISSION_ACCEPTED;
}

// mavlink_cmd_long_to_mission_cmd - converts a mavlink cmd long to an AP_Mission::Mission_Command object which can be stored to eeprom
// return MAV_MISSION_ACCEPTED on success, MAV_MISSION_RESULT error on failure
MAV_MISSION_RESULT AP_Mission::mavlink_cmd_long_to_mission_cmd(const mavlink_command_long_t& packet, AP_Mission::Mission_Command& cmd)
{
    mavlink_mission_item_int_t miss_item = {0};

    miss_item.param1 = packet.param1;
    miss_item.param2 = packet.param2;
    miss_item.param3 = packet.param3;
    miss_item.param4 = packet.param4;

    miss_item.command = packet.command;
    miss_item.target_system = packet.target_system;
    miss_item.target_component = packet.target_component;

    return mavlink_int_to_mission_cmd(miss_item, cmd);
}

// mission_cmd_to_mavlink_int - converts an AP_Mission::Mission_Command object to a mavlink message which can be sent to the GCS
//  return true on success, false on failure
//  NOTE: callers to this method current fill parts of "packet" in before calling this method, so do NOT attempt to zero the entire packet in here
bool AP_Mission::mission_cmd_to_mavlink_int(const AP_Mission::Mission_Command& cmd, mavlink_mission_item_int_t& packet)
{
    // command's position in mission list and mavlink id
    packet.seq = cmd.index;
    packet.command = cmd.id;

    // set defaults
    packet.current = 0;     // 1 if we are passing back the mission command that is currently being executed
    packet.param1 = 0;
    packet.param2 = 0;
    packet.param3 = 0;
    packet.param4 = 0;
    packet.frame = 0;
    packet.autocontinue = 1;

    // command specific conversions from mission command to mavlink packet
    switch (cmd.id) {
    case 0:
        // this is reserved for 16 bit command IDs
        return false;

    case MAV_CMD_NAV_WAYPOINT:                          // MAV ID: 16
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
        // acceptance radius in meters

        packet.param2 = LOWBYTE(cmd.p1);        // param 2 is acceptance radius in meters is held in low p1
        packet.param3 = HIGHBYTE(cmd.p1);       // param 3 is pass by distance in meters is held in high p1
#else
        // delay at waypoint in seconds
        packet.param1 = cmd.p1;
#endif
        break;

    case MAV_CMD_NAV_LOITER_UNLIM:                      // MAV ID: 17
        packet.param3 = (float)cmd.p1;
        if (cmd.content.location.loiter_ccw) {
            packet.param3 *= -1;
        }
        break;

    case MAV_CMD_NAV_LOITER_TURNS:                      // MAV ID: 18
        packet.param1 = LOWBYTE(cmd.p1);                // number of times to circle is held in low byte of p1
        packet.param3 = HIGHBYTE(cmd.p1);               // radius is held in high byte of p1
        if (cmd.content.location.loiter_ccw) {
            packet.param3 = -packet.param3;
        }
        if (cmd.type_specific_bits & (1U<<0)) {
            packet.param3 *= 10;
        }
        packet.param4 = cmd.content.location.loiter_xtrack; // 0 to xtrack from center of waypoint, 1 to xtrack from tangent exit location
        break;

    case MAV_CMD_NAV_LOITER_TIME:                       // MAV ID: 19
        packet.param1 = cmd.p1;                         // loiter time in seconds
        if (cmd.content.location.loiter_ccw) {
            packet.param3 = -1;
        } else {
            packet.param3 = 1;
        }
        packet.param4 = cmd.content.location.loiter_xtrack; // 0 to xtrack from center of waypoint, 1 to xtrack from tangent exit location
        break;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:                  // MAV ID: 20
        break;

    case MAV_CMD_NAV_LAND:                              // MAV ID: 21
        packet.param1 = cmd.p1;                        // abort target altitude(m)  (plane only)
        packet.param4 = cmd.content.location.loiter_ccw ? -1 : 1; // yaw direction, (plane deepstall only)
        break;

    case MAV_CMD_NAV_TAKEOFF:                           // MAV ID: 22
        packet.param1 = cmd.p1;                         // minimum pitch (plane only)
        break;

    case MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:           // MAV ID: 30
        packet.param1 = cmd.p1;                         // Climb/Descend
        // 0 = Neutral, cmd complete at +/- 5 of indicated alt.
        // 1 = Climb, cmd complete at or above indicated alt.
        // 2 = Descend, cmd complete at or below indicated alt.
        break;

    case MAV_CMD_NAV_LOITER_TO_ALT:                     // MAV ID: 31
        packet.param2 = cmd.p1;                        // loiter radius(m)
        if (cmd.content.location.loiter_ccw) {
            packet.param2 = -packet.param2;
        }
        packet.param4 = cmd.content.location.loiter_xtrack; // 0 to xtrack from center of waypoint, 1 to xtrack from tangent exit location
        break;

    case MAV_CMD_NAV_SPLINE_WAYPOINT:                   // MAV ID: 82
        packet.param1 = cmd.p1;                         // delay at waypoint in seconds
        break;

    case MAV_CMD_NAV_GUIDED_ENABLE:                     // MAV ID: 92
        packet.param1 = cmd.p1;                         // on/off. >0.5 means "on", hand-over control to external controller
        break;

    case MAV_CMD_NAV_DELAY:                            // MAV ID: 93
        packet.param1 = cmd.content.nav_delay.seconds; // delay in seconds
        packet.param2 = cmd.content.nav_delay.hour_utc; // absolute time's day of week (utc)
        packet.param3 = cmd.content.nav_delay.min_utc; // absolute time's hour (utc)
        packet.param4 = cmd.content.nav_delay.sec_utc; // absolute time's min (utc)
        break;

    case MAV_CMD_CONDITION_DELAY:                       // MAV ID: 112
        packet.param1 = cmd.content.delay.seconds;      // delay in seconds
        break;

    case MAV_CMD_CONDITION_DISTANCE:                    // MAV ID: 114
        packet.param1 = cmd.content.distance.meters;    // distance in meters from next waypoint
        break;

    case MAV_CMD_CONDITION_YAW:                         // MAV ID: 115
        packet.param1 = cmd.content.yaw.angle_deg;      // target angle in degrees
        packet.param2 = cmd.content.yaw.turn_rate_dps;  // 0 = use default turn rate otherwise specific turn rate in deg/sec
        packet.param3 = cmd.content.yaw.direction;      // -1 = ccw, +1 = cw
        packet.param4 = cmd.content.yaw.relative_angle; // 0 = absolute angle provided, 1 = relative angle provided
        break;

    case MAV_CMD_DO_JUMP:                               // MAV ID: 177
        packet.param1 = cmd.content.jump.target;        // jump-to command number
        packet.param2 = cmd.content.jump.num_times;     // repeat count
        break;

    case MAV_CMD_DO_CHANGE_SPEED:                       // MAV ID: 178
        packet.param1 = cmd.content.speed.speed_type;   // 0 = airspeed, 1 = ground speed
        packet.param2 = cmd.content.speed.target_ms;    // speed in m/s
        packet.param3 = cmd.content.speed.throttle_pct; // throttle as a percentage from 1 ~ 100%
        break;

    case MAV_CMD_DO_SET_HOME:                           // MAV ID: 179
        packet.param1 = cmd.p1;                         // p1=0 means use current location, p=1 means use provided location
        break;

    case MAV_CMD_DO_SET_RELAY:                          // MAV ID: 181
        packet.param1 = cmd.content.relay.num;          // relay number
        packet.param2 = cmd.content.relay.state;        // 0:off, 1:on
        break;

    case MAV_CMD_DO_REPEAT_RELAY:                       // MAV ID: 182
        packet.param1 = cmd.content.repeat_relay.num;           // relay number
        packet.param2 = cmd.content.repeat_relay.repeat_count;  // count
        packet.param3 = cmd.content.repeat_relay.cycle_time;    // time in seconds
        break;

    case MAV_CMD_DO_SET_SERVO:                          // MAV ID: 183
        packet.param1 = cmd.content.servo.channel;      // channel
        packet.param2 = cmd.content.servo.pwm;          // PWM
        break;

    case MAV_CMD_DO_REPEAT_SERVO:                       // MAV ID: 184
        packet.param1 = cmd.content.repeat_servo.channel;       // channel
        packet.param2 = cmd.content.repeat_servo.pwm;           // PWM
        packet.param3 = cmd.content.repeat_servo.repeat_count;  // count
        packet.param4 = cmd.content.repeat_servo.cycle_time;    // time in milliseconds converted to seconds
        break;

    case MAV_CMD_DO_LAND_START:                         // MAV ID: 189
        break;

    case MAV_CMD_DO_GO_AROUND:                          // MAV ID: 191
        break;

    case MAV_CMD_DO_SET_ROI:                            // MAV ID: 201
        packet.param1 = cmd.p1;                         // 0 = no roi, 1 = next waypoint, 2 = waypoint number, 3 = fixed location, 4 = given target (not supported)
        break;

    case MAV_CMD_DO_DIGICAM_CONFIGURE:                  // MAV ID: 202
        packet.param1 = cmd.content.digicam_configure.shooting_mode;
        packet.param2 = cmd.content.digicam_configure.shutter_speed;
        packet.param3 = cmd.content.digicam_configure.aperture;
        packet.param4 = cmd.content.digicam_configure.ISO;
        packet.x = cmd.content.digicam_configure.exposure_type;
        packet.y = cmd.content.digicam_configure.cmd_id;
        packet.z = cmd.content.digicam_configure.engine_cutoff_time;
        break;

    case MAV_CMD_DO_DIGICAM_CONTROL:                    // MAV ID: 203
        packet.param1 = cmd.content.digicam_control.session;
        packet.param2 = cmd.content.digicam_control.zoom_pos;
        packet.param3 = cmd.content.digicam_control.zoom_step;
        packet.param4 = cmd.content.digicam_control.focus_lock;
        packet.x = cmd.content.digicam_control.shooting_cmd;
        packet.y = cmd.content.digicam_control.cmd_id;
        break;

    case MAV_CMD_DO_MOUNT_CONTROL:                      // MAV ID: 205
        packet.param1 = cmd.content.mount_control.pitch;
        packet.param2 = cmd.content.mount_control.roll;
        packet.param3 = cmd.content.mount_control.yaw;
        break;

    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:                 // MAV ID: 206
        packet.param1 = cmd.content.cam_trigg_dist.meters;  // distance between camera shots in meters
        packet.param3 = cmd.content.cam_trigg_dist.trigger; // when enabled, camera triggers once immediately
        break;

    case MAV_CMD_DO_FENCE_ENABLE:                       // MAV ID: 207
        packet.param1 = cmd.p1;                         // action 0=disable, 1=enable
        break;

    case MAV_CMD_DO_PARACHUTE:                          // MAV ID: 208
        packet.param1 = cmd.p1;                         // action 0=disable, 1=enable, 2=release.  See PARACHUTE_ACTION enum
        break;

    case MAV_CMD_DO_SPRAYER:
        packet.param1 = cmd.p1;                         // action 0=disable, 1=enable
        break;

    case MAV_CMD_DO_AUX_FUNCTION:
        packet.param1 = cmd.content.auxfunction.function;
        packet.param2 = cmd.content.auxfunction.switchpos;
        break;

    case MAV_CMD_DO_INVERTED_FLIGHT:                    // MAV ID: 210
        packet.param1 = cmd.p1;                         // normal=0 inverted=1
        break;

#if AP_GRIPPER_ENABLED
    case MAV_CMD_DO_GRIPPER:                            // MAV ID: 211
        packet.param1 = cmd.content.gripper.num;        // gripper number
        packet.param2 = cmd.content.gripper.action;     // action 0=release, 1=grab.  See GRIPPER_ACTION enum
        break;
#endif

    case MAV_CMD_DO_GUIDED_LIMITS:                      // MAV ID: 222
        packet.param1 = cmd.p1;                         // max time in seconds the external controller will be allowed to control the vehicle
        packet.param2 = cmd.content.guided_limits.alt_min;  // min alt below which the command will be aborted.  0 for no lower alt limit
        packet.param3 = cmd.content.guided_limits.alt_max;  // max alt above which the command will be aborted.  0 for no upper alt limit
        packet.param4 = cmd.content.guided_limits.horiz_max;// max horizontal distance the vehicle can move before the command will be aborted.  0 for no horizontal limit
        break;

    case MAV_CMD_DO_AUTOTUNE_ENABLE:
        packet.param1 = cmd.p1;                         // disable=0 enable=1
        break;

    case MAV_CMD_DO_SET_REVERSE:
        packet.param1 = cmd.p1;   // 0 = forward, 1 = reverse
        break;

    case MAV_CMD_NAV_ALTITUDE_WAIT:                     // MAV ID: 83
        packet.param1 = cmd.content.altitude_wait.altitude;
        packet.param2 = cmd.content.altitude_wait.descent_rate;
        packet.param3 = cmd.content.altitude_wait.wiggle_time;
        break;

    case MAV_CMD_NAV_VTOL_TAKEOFF:
        break;

    case MAV_CMD_NAV_VTOL_LAND:
        packet.param1 = cmd.p1;
        break;

    case MAV_CMD_DO_VTOL_TRANSITION:
        packet.param1 = cmd.content.do_vtol_transition.target_state;
        break;

    case MAV_CMD_DO_ENGINE_CONTROL:
        packet.param1 = cmd.content.do_engine_control.start_control?1:0;
        packet.param2 = cmd.content.do_engine_control.cold_start?1:0;
        packet.param3 = cmd.content.do_engine_control.height_delay_cm*0.01f;
        break;

    case MAV_CMD_NAV_PAYLOAD_PLACE:
        packet.param1 = cmd.p1/100.0f; // copy max-descend parameter (cm->m)
        break;

    case MAV_CMD_NAV_SET_YAW_SPEED:
        packet.param1 = cmd.content.set_yaw_speed.angle_deg;        // target angle in degrees
        packet.param2 = cmd.content.set_yaw_speed.speed;            // speed in meters/second
        packet.param3 = cmd.content.set_yaw_speed.relative_angle;   // 0 = absolute angle, 1 = relative angle
        break;

    case MAV_CMD_DO_WINCH:
        packet.param1 = cmd.content.winch.num;              // winch number
        packet.param2 = cmd.content.winch.action;           // action (0 = relax, 1 = length control, 2 = rate control).  See WINCH_ACTION enum
        packet.param3 = cmd.content.winch.release_length;   // cable distance to unwind in meters, negative numbers to wind in cable
        packet.param4 = cmd.content.winch.release_rate;     // release rate in meters/second
        break;

    case MAV_CMD_DO_SET_RESUME_REPEAT_DIST:
        packet.param1 = cmd.p1; // Resume repeat distance (m)
        break;

    case MAV_CMD_DO_SEND_SCRIPT_MESSAGE:
        packet.param1 = cmd.p1;
        packet.param2 = cmd.content.scripting.p1;
        packet.param3 = cmd.content.scripting.p2;
        packet.param4 = cmd.content.scripting.p3;
        break;

#if AP_SCRIPTING_ENABLED
    case MAV_CMD_NAV_SCRIPT_TIME:
        packet.param1 = cmd.content.nav_script_time.command;
        packet.param2 = cmd.content.nav_script_time.timeout_s;
        packet.param3 = cmd.content.nav_script_time.arg1.get();
        packet.param4 = cmd.content.nav_script_time.arg2.get();
        packet.x = cmd.content.nav_script_time.arg3;
        packet.y = cmd.content.nav_script_time.arg4;
        break;
#endif

    case MAV_CMD_NAV_ATTITUDE_TIME:
        packet.param1 = cmd.content.nav_attitude_time.time_sec;
        packet.param2 = cmd.content.nav_attitude_time.roll_deg;
        packet.param3 = cmd.content.nav_attitude_time.pitch_deg;
        packet.param4 = cmd.content.nav_attitude_time.yaw_deg;
        packet.x = cmd.content.nav_attitude_time.climb_rate;
        break;

    case MAV_CMD_DO_PAUSE_CONTINUE:
        packet.param1 = cmd.p1;
        break;

    case MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW:
        packet.param1 = cmd.content.gimbal_manager_pitchyaw.pitch_angle_deg;
        packet.param2 = cmd.content.gimbal_manager_pitchyaw.yaw_angle_deg;
        packet.param3 = cmd.content.gimbal_manager_pitchyaw.pitch_rate_degs;
        packet.param4 = cmd.content.gimbal_manager_pitchyaw.yaw_rate_degs;
        packet.x = cmd.content.gimbal_manager_pitchyaw.flags;
        packet.z = cmd.content.gimbal_manager_pitchyaw.gimbal_id;
        break;

    default:
        // unrecognised command
        return false;
    }

    // copy location from mavlink to command
    if (stored_in_location(cmd.id)) {
        packet.x = cmd.content.location.lat;
        packet.y = cmd.content.location.lng;

        packet.z = cmd.content.location.alt * 0.01f;   // cmd alt in cm to m
        if (cmd.content.location.relative_alt) {
            packet.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
        } else {
            packet.frame = MAV_FRAME_GLOBAL;
        }
#if AP_TERRAIN_AVAILABLE
        if (cmd.content.location.terrain_alt) {
            // this is a above-terrain altitude
            if (!cmd.content.location.relative_alt) {
                // refuse to return non-relative terrain mission
                // items. Internally we do have these, and they
                // have home.alt added, but we should never be
                // returning them to the GCS, as the GCS doesn't know
                // our home.alt, so it would have no way to properly
                // interpret it
                return false;
            }
            packet.z = cmd.content.location.alt * 0.01f;
            packet.frame = MAV_FRAME_GLOBAL_TERRAIN_ALT;
        }
#else
        // don't ever return terrain mission items if no terrain support
        if (cmd.content.location.terrain_alt) {
            return false;
        }
#endif
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
    _flags.in_landing_sequence = false;

    // callback to main program's mission complete function
    _mission_complete_fn();
}

/// advance_current_nav_cmd - moves current nav command forward
///     do command will also be loaded
///     accounts for do-jump commands
//      returns true if command is advanced, false if failed (i.e. mission completed)
bool AP_Mission::advance_current_nav_cmd(uint16_t starting_index)
{
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
    _flags.do_cmd_all_done = false;

    // get starting point for search
    uint16_t cmd_index = starting_index > 0 ? starting_index - 1 : _nav_cmd.index;
    if (cmd_index == AP_MISSION_CMD_INDEX_NONE) {
        // start from beginning of the mission command list
        cmd_index = AP_MISSION_FIRST_REAL_COMMAND;
    } else {
        // start from one position past the current nav command
        cmd_index++;
    }

    // avoid endless loops
    uint8_t max_loops = 255;

    // search until we find next nav command or reach end of command list
    while (!_flags.nav_cmd_loaded && max_loops-- > 0) {
        // get next command
        Mission_Command cmd;
        if (!get_next_cmd(cmd_index, cmd, true)) {
            return false;
        }

        // check if navigation or "do" command
        if (is_nav_cmd(cmd)) {
            // save previous nav command index
            _prev_nav_cmd_id = _nav_cmd.id;
            _prev_nav_cmd_index = _nav_cmd.index;
            // save separate previous nav command index if it contains lat,long,alt
            if (!(cmd.content.location.lat == 0 && cmd.content.location.lng == 0)) {
                _prev_nav_cmd_wp_index = _nav_cmd.index;
            }
            // set current navigation command and start it
            _nav_cmd = cmd;
            if (start_command(_nav_cmd)) {
                _flags.nav_cmd_loaded = true;
            }
            // save a loaded wp index in history array for when _repeat_dist is set via MAV_CMD_DO_SET_RESUME_REPEAT_DIST
            // and prevent history being re-written until vehicle returns to interrupted position
            if (_repeat_dist > 0 && !_flags.resuming_mission && _nav_cmd.index != AP_MISSION_CMD_INDEX_NONE && !(_nav_cmd.content.location.lat == 0 && _nav_cmd.content.location.lng == 0)) {
                // update mission history. last index position is always the most recent wp loaded.
                for (uint8_t i=0; i<AP_MISSION_MAX_WP_HISTORY-1; i++) {
                    _wp_index_history[i] = _wp_index_history[i+1];
                }
                _wp_index_history[AP_MISSION_MAX_WP_HISTORY-1] = _nav_cmd.index;
            }
            // check if the vehicle is resuming and has returned to where it was interrupted
            if (_flags.resuming_mission && _nav_cmd.index == _wp_index_history[AP_MISSION_MAX_WP_HISTORY-1]) {
                // vehicle has resumed previous position
                gcs().send_text(MAV_SEVERITY_INFO, "Mission: Returned to interrupted WP");
                _flags.resuming_mission = false;
            }

        } else {
            // set current do command and start it (if not already set)
            if (!_flags.do_cmd_loaded) {
                _do_cmd = cmd;
                _flags.do_cmd_loaded = true;
                start_command(_do_cmd);
            }
        }
        // move onto next command
        cmd_index = cmd.index+1;
    }

    if (max_loops == 0) {
        // infinite loop.  This can happen if there's a loop involving
        // only nav commands (no DO commands) which won't start()
        return false;
    }

    // if we have not found a do command then set flag to show there are no do-commands to be run before nav command completes
    if (!_flags.do_cmd_loaded) {
        _flags.do_cmd_all_done = true;
    }

    // if we got this far we must have successfully advanced the nav command
    return true;
}

/// advance_current_do_cmd - moves current do command forward
///     accounts for do-jump commands
void AP_Mission::advance_current_do_cmd()
{
    // exit immediately if we're not running or we've completed all possible "do" commands
    if (_flags.state != MISSION_RUNNING || _flags.do_cmd_all_done) {
        return;
    }

    // get starting point for search
    uint16_t cmd_index = _do_cmd.index;
    if (cmd_index == AP_MISSION_CMD_INDEX_NONE) {
        cmd_index = AP_MISSION_FIRST_REAL_COMMAND;
    } else {
        // start from one position past the current do command
        cmd_index = _do_cmd.index + 1;
    }

    // find next do command
    Mission_Command cmd;
    if (!get_next_do_cmd(cmd_index, cmd)) {
        // set flag to stop unnecessarily searching for do commands
        _flags.do_cmd_all_done = true;
        return;
    }

    // set current do command and start it
    _do_cmd = cmd;
    _flags.do_cmd_loaded = true;
    start_command(_do_cmd);
}

/// get_next_cmd - gets next command found at or after start_index
///     returns true if found, false if not found (i.e. mission complete)
///     accounts for do_jump commands
///     increment_jump_num_times_if_found should be set to true if advancing the active navigation command
bool AP_Mission::get_next_cmd(uint16_t start_index, Mission_Command& cmd, bool increment_jump_num_times_if_found, bool send_gcs_msg)
{
    uint16_t cmd_index = start_index;
    Mission_Command temp_cmd;
    uint16_t jump_index = AP_MISSION_CMD_INDEX_NONE;

    // search until the end of the mission command list
    uint8_t max_loops = 64;
    while (cmd_index < (unsigned)_cmd_total) {
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
            } else {
                // get number of times jump command has already been run
                int16_t jump_times_run = get_jump_times_run(temp_cmd);
                if (jump_times_run < temp_cmd.content.jump.num_times) {
                    // update the record of the number of times run
                    if (increment_jump_num_times_if_found && !_flags.resuming_mission) {
                        increment_jump_times_run(temp_cmd, send_gcs_msg);
                    }
                    // continue searching from jump target
                    cmd_index = temp_cmd.content.jump.target;
                } else {
                    // jump has been run specified number of times so move search to next command in mission
                    cmd_index++;
                }
            }
        } else {
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
bool AP_Mission::get_next_do_cmd(uint16_t start_index, Mission_Command& cmd)
{
    Mission_Command temp_cmd;

    // check we have not passed the end of the mission list
    if (start_index >= (unsigned)_cmd_total) {
        return false;
    }

    // get next command
    if (!get_next_cmd(start_index, temp_cmd, false)) {
        // no more commands so return failure
        return false;
    } else if (is_nav_cmd(temp_cmd)) {
        // if it's a "navigation" command then return false because we do not progress past nav commands
        return false;
    } else {
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
    for (uint8_t i=0; i<AP_MISSION_MAX_NUM_DO_JUMP_COMMANDS; i++) {
        _jump_tracking[i].index = AP_MISSION_CMD_INDEX_NONE;
        _jump_tracking[i].num_times_run = 0;
    }
}

/// get_jump_times_run - returns number of times the jump command has been run
int16_t AP_Mission::get_jump_times_run(const Mission_Command& cmd)
{
    // exit immediately if cmd is not a do-jump command or target is invalid
    if ((cmd.id != MAV_CMD_DO_JUMP) || (cmd.content.jump.target >= (unsigned)_cmd_total) || (cmd.content.jump.target == 0)) {
        // To-Do: log an error?
        return AP_MISSION_JUMP_TIMES_MAX;
    }

    // search through jump_tracking array for this cmd
    for (uint8_t i=0; i<AP_MISSION_MAX_NUM_DO_JUMP_COMMANDS; i++) {
        if (_jump_tracking[i].index == cmd.index) {
            return _jump_tracking[i].num_times_run;
        } else if (_jump_tracking[i].index == AP_MISSION_CMD_INDEX_NONE) {
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
void AP_Mission::increment_jump_times_run(Mission_Command& cmd, bool send_gcs_msg)
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
            if (send_gcs_msg) {
                gcs().send_text(MAV_SEVERITY_INFO, "Mission: %u Jump %i/%i", _jump_tracking[i].index, _jump_tracking[i].num_times_run, cmd.content.jump.num_times);
            }
            return;
        } else if (_jump_tracking[i].index == AP_MISSION_CMD_INDEX_NONE) {
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

// find the nearest landing sequence starting point (DO_LAND_START) and
// return its index.  Returns 0 if no appropriate DO_LAND_START point can
// be found.
uint16_t AP_Mission::get_landing_sequence_start()
{
    struct Location current_loc;

    if (!AP::ahrs().get_location(current_loc)) {
        return 0;
    }

    uint16_t landing_start_index = 0;
    float min_distance = -1;

    // Go through mission looking for nearest landing start command
    for (uint16_t i = 1; i < num_commands(); i++) {
        Mission_Command tmp;
        if (!read_cmd_from_storage(i, tmp)) {
            continue;
        }
        if (tmp.id == MAV_CMD_DO_LAND_START) {
            if (!tmp.content.location.initialised() && !get_next_nav_cmd(i, tmp)) {
                // command does not have a valid location and cannot get next valid
                continue;
            }
            float tmp_distance = tmp.content.location.get_distance(current_loc);
            if (min_distance < 0 || tmp_distance < min_distance) {
                min_distance = tmp_distance;
                landing_start_index = i;
            }
        }
    }

    return landing_start_index;
}

/*
   find the nearest landing sequence starting point (DO_LAND_START) and
   switch to that mission item.  Returns false if no DO_LAND_START
   available.
 */
bool AP_Mission::jump_to_landing_sequence(void)
{
    uint16_t land_idx = get_landing_sequence_start();
    if (land_idx != 0 && set_current_cmd(land_idx)) {

        //if the mission has ended it has to be restarted
        if (state() == AP_Mission::MISSION_STOPPED) {
            resume();
        }

        gcs().send_text(MAV_SEVERITY_INFO, "Landing sequence start");
        _flags.in_landing_sequence = true;
        return true;
    }

    gcs().send_text(MAV_SEVERITY_WARNING, "Unable to start landing sequence");
    return false;
}

// jumps the mission to the closest landing abort that is planned, returns false if unable to find a valid abort
bool AP_Mission::jump_to_abort_landing_sequence(void)
{
    struct Location current_loc;

    uint16_t abort_index = 0;
    if (AP::ahrs().get_location(current_loc)) {
        float min_distance = FLT_MAX;

        for (uint16_t i = 1; i < num_commands(); i++) {
            Mission_Command tmp;
            if (!read_cmd_from_storage(i, tmp)) {
                continue;
            }
            if (tmp.id == MAV_CMD_DO_GO_AROUND) {
                float tmp_distance = tmp.content.location.get_distance(current_loc);
                if (tmp_distance < min_distance) {
                    min_distance = tmp_distance;
                    abort_index = i;
                }
            }
        }
    }

    if (abort_index != 0 && set_current_cmd(abort_index)) {

        //if the mission has ended it has to be restarted
        if (state() == AP_Mission::MISSION_STOPPED) {
            resume();
        }

        _flags.in_landing_sequence = false;

        gcs().send_text(MAV_SEVERITY_INFO, "Landing abort sequence start");
        return true;
    }

    gcs().send_text(MAV_SEVERITY_WARNING, "Unable to start find a landing abort sequence");
    return false;
}

// check which is the shortest route to landing an RTL via a DO_LAND_START or continuing on the current mission plan
bool AP_Mission::is_best_land_sequence(void)
{
    // check if there is even a running mission to interupt
    if (_flags.state != MISSION_RUNNING) {
        return false;
    }

    // check if aircraft has already jumped to a landing sequence
    if (_flags.in_landing_sequence) {
        return true;
    }

    // check if MIS_OPTIONS bit set to allow distance calculation to be done
    if (!(_options & AP_MISSION_MASK_DIST_TO_LAND_CALC)) {
        return false;
    }

    // The decision to allow a failsafe to interupt a potential landing approach
    // is a distance travelled minimization problem.  Look forward in
    // mission to evaluate the shortest remaining distance to land.

    // go through the mission for the nearest DO_LAND_START first as this is the most probable route
    // to a landing with the minimum number of WP.
    uint16_t do_land_start_index = get_landing_sequence_start();
    if (do_land_start_index == 0) {
        // then no DO_LAND_START commands are in mission and normal failsafe behaviour should be maintained
        return false;
    }

    // get our current location
    Location current_loc;
    if (!AP::ahrs().get_location(current_loc)) {
        // we don't know where we are!!
        return false;
    }

    // get distance to landing if travelled to nearest DO_LAND_START via RTL
    float dist_via_do_land;
    if (!distance_to_landing(do_land_start_index, dist_via_do_land, current_loc)) {
        // cant get a valid distance to landing
        return false;
    }

    // get distance to landing if continue along current mission path
    float dist_continue_to_land;
    if (!distance_to_landing(_nav_cmd.index, dist_continue_to_land, current_loc)) {
        // cant get a valid distance to landing
        return false;
    }

    // compare distances
    if (dist_via_do_land >= dist_continue_to_land) {
        // then the mission should carry on uninterrupted as that is the shorter distance
        gcs().send_text(MAV_SEVERITY_NOTICE, "Rejecting RTL: closer land if mis continued");
        return true;
    } else {
        // allow failsafes to interrupt the current mission
        return false;
    }
}

// Approximate the distance travelled to get to a landing.  DO_JUMP commands are observed in look forward.
bool AP_Mission::distance_to_landing(uint16_t index, float &tot_distance, Location prev_loc)
{
    Mission_Command temp_cmd;
    tot_distance = 0.0f;
    bool ret;

    // back up jump tracking to reset after distance calculation
    jump_tracking_struct _jump_tracking_backup[AP_MISSION_MAX_NUM_DO_JUMP_COMMANDS];
    for (uint8_t i=0; i<AP_MISSION_MAX_NUM_DO_JUMP_COMMANDS; i++) {
        _jump_tracking_backup[i] = _jump_tracking[i];
    }

    // run through remainder of mission to approximate a distance to landing
    for (uint8_t i=0; i<255; i++) {
        // search until the end of the mission command list
        for (uint16_t cmd_index = index; cmd_index < (unsigned)_cmd_total; cmd_index++) {
            // get next command
            if (!get_next_cmd(cmd_index, temp_cmd, true, false)) {
                // we got to the end of the mission
                ret = false;
                goto reset_do_jump_tracking;
            }
            if (temp_cmd.id == MAV_CMD_NAV_WAYPOINT || temp_cmd.id == MAV_CMD_NAV_SPLINE_WAYPOINT || is_landing_type_cmd(temp_cmd.id)) {
                break;
            } else if (is_nav_cmd(temp_cmd) || temp_cmd.id == MAV_CMD_CONDITION_DELAY) {
                // if we receive a nav command that we dont handle then give up as cant measure the distance e.g. MAV_CMD_NAV_LOITER_UNLIM
                ret = false;
                goto reset_do_jump_tracking;
            }
        }
        index = temp_cmd.index+1;

        if (!(temp_cmd.content.location.lat == 0 && temp_cmd.content.location.lng == 0)) {
            // add distance to running total
            float disttemp = prev_loc.get_distance(temp_cmd.content.location);
            tot_distance = tot_distance + disttemp;

            // store wp location as previous
            prev_loc = temp_cmd.content.location;
        }

        if (is_landing_type_cmd(temp_cmd.id)) {
            // reached a landing!
            ret = true;
            goto reset_do_jump_tracking;
        }
    }
    // reached end of loop without getting to a landing
    ret = false;

reset_do_jump_tracking:
    for (uint8_t i=0; i<AP_MISSION_MAX_NUM_DO_JUMP_COMMANDS; i++) {
        _jump_tracking[i] = _jump_tracking_backup[i];
    }

    return ret;
}

// check if command is a landing type command.
bool AP_Mission::is_landing_type_cmd(uint16_t id) const
{
    switch (id) {
    case MAV_CMD_NAV_LAND:
    case MAV_CMD_NAV_VTOL_LAND:
    case MAV_CMD_DO_PARACHUTE:
        return true;
    default:
        return false;
    }
}

// check if command is a takeoff type command.
bool AP_Mission::is_takeoff_type_cmd(uint16_t id) const
{
    switch (id) {
    case MAV_CMD_NAV_TAKEOFF:
    case MAV_CMD_NAV_VTOL_TAKEOFF:
        return true;
    default:
        return false;
    }
}

const char *AP_Mission::Mission_Command::type() const
{
    switch (id) {
    case MAV_CMD_NAV_WAYPOINT:
        return "WP";
    case MAV_CMD_NAV_SPLINE_WAYPOINT:
        return "SplineWP";
    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        return "RTL";
    case MAV_CMD_NAV_LOITER_UNLIM:
        return "LoitUnlim";
    case MAV_CMD_NAV_LOITER_TIME:
        return "LoitTime";
    case MAV_CMD_NAV_GUIDED_ENABLE:
        return "GuidedEnable";
    case MAV_CMD_NAV_LOITER_TURNS:
        return "LoitTurns";
    case MAV_CMD_NAV_LOITER_TO_ALT:
        return "LoitAltitude";
    case MAV_CMD_NAV_SET_YAW_SPEED:
        return "SetYawSpd";
    case MAV_CMD_CONDITION_DELAY:
        return "CondDelay";
    case MAV_CMD_CONDITION_DISTANCE:
        return "CondDist";
    case MAV_CMD_DO_CHANGE_SPEED:
        return "ChangeSpeed";
    case MAV_CMD_DO_SET_HOME:
        return "SetHome";
    case MAV_CMD_DO_SET_SERVO:
        return "SetServo";
    case MAV_CMD_DO_SET_RELAY:
        return "SetRelay";
    case MAV_CMD_DO_REPEAT_SERVO:
        return "RepeatServo";
    case MAV_CMD_DO_REPEAT_RELAY:
        return "RepeatRelay";
    case MAV_CMD_DO_CONTROL_VIDEO:
        return "CtrlVideo";
    case MAV_CMD_DO_DIGICAM_CONFIGURE:
        return "DigiCamCfg";
    case MAV_CMD_DO_DIGICAM_CONTROL:
        return "DigiCamCtrl";
    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
        return "SetCamTrigDst";
    case MAV_CMD_DO_SET_ROI:
        return "SetROI";
    case MAV_CMD_DO_SET_REVERSE:
        return "SetReverse";
    case MAV_CMD_DO_SET_RESUME_REPEAT_DIST:
        return "SetRepeatDist";
    case MAV_CMD_DO_GUIDED_LIMITS:
        return "GuidedLimits";
    case MAV_CMD_NAV_TAKEOFF:
        return "Takeoff";
    case MAV_CMD_NAV_LAND:
        return "Land";
    case MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
        return "ContinueAndChangeAlt";
    case MAV_CMD_NAV_ALTITUDE_WAIT:
        return "AltitudeWait";
    case MAV_CMD_NAV_VTOL_TAKEOFF:
        return "VTOLTakeoff";
    case MAV_CMD_NAV_VTOL_LAND:
        return "VTOLLand";
    case MAV_CMD_DO_INVERTED_FLIGHT:
        return "InvertedFlight";
    case MAV_CMD_DO_FENCE_ENABLE:
        return "FenceEnable";
    case MAV_CMD_DO_AUTOTUNE_ENABLE:
        return "AutoTuneEnable";
    case MAV_CMD_DO_VTOL_TRANSITION:
        return "VTOLTransition";
    case MAV_CMD_DO_ENGINE_CONTROL:
        return "EngineControl";
    case MAV_CMD_CONDITION_YAW:
        return "CondYaw";
    case MAV_CMD_DO_LAND_START:
        return "LandStart";
    case MAV_CMD_NAV_DELAY:
        return "Delay";
#if AP_GRIPPER_ENABLED
    case MAV_CMD_DO_GRIPPER:
        return "Gripper";
#endif
    case MAV_CMD_NAV_PAYLOAD_PLACE:
        return "PayloadPlace";
    case MAV_CMD_DO_PARACHUTE:
        return "Parachute";
    case MAV_CMD_DO_SPRAYER:
        return "Sprayer";
    case MAV_CMD_DO_AUX_FUNCTION:
        return "AuxFunction";
    case MAV_CMD_DO_MOUNT_CONTROL:
        return "MountControl";
    case MAV_CMD_DO_WINCH:
        return "Winch";
    case MAV_CMD_DO_SEND_SCRIPT_MESSAGE:
        return "Scripting";
    case MAV_CMD_DO_JUMP:
        return "Jump";
    case MAV_CMD_DO_GO_AROUND:
        return "Go Around";
#if AP_SCRIPTING_ENABLED
    case MAV_CMD_NAV_SCRIPT_TIME:
        return "NavScriptTime";
#endif
    case MAV_CMD_NAV_ATTITUDE_TIME:
        return "NavAttitudeTime";
    case MAV_CMD_DO_PAUSE_CONTINUE:
        return "PauseContinue";
    case MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW:
        return "GimbalPitchYaw";
    default:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Mission command with ID %u has no string", id);
#endif
        return "?";
    }
}

bool AP_Mission::contains_item(MAV_CMD command) const
{
    for (int i = 1; i < num_commands(); i++) {
        Mission_Command tmp;
        if (!read_cmd_from_storage(i, tmp)) {
            continue;
        }
        if (tmp.id == command) {
            return true;
        }
    }
    return false;
}

/*
  return true if the mission has a terrain relative item.  ~2200us for 530 items on H7
 */
bool AP_Mission::contains_terrain_alt_items(void)
{
    if (_last_contains_relative_calculated_ms != _last_change_time_ms) {
        _contains_terrain_alt_items = calculate_contains_terrain_alt_items();
        _last_contains_relative_calculated_ms = _last_change_time_ms;
    }
    return _contains_terrain_alt_items;
}

bool AP_Mission::calculate_contains_terrain_alt_items(void) const
{
    for (int i = 1; i < num_commands(); i++) {
        Mission_Command tmp;
        if (!read_cmd_from_storage(i, tmp)) {
            continue;
        }
        if (stored_in_location(tmp.id) && tmp.content.location.terrain_alt) {
            return true;
        }
    }
    return false;
}

// reset the mission history to prevent recalling previous mission histories after a mission restart.
void AP_Mission::reset_wp_history(void)
{
    for (uint8_t i = 0; i<AP_MISSION_MAX_WP_HISTORY; i++) {
        _wp_index_history[i] = AP_MISSION_CMD_INDEX_NONE;
    }
    _resume_cmd.index = AP_MISSION_CMD_INDEX_NONE;
    _flags.resuming_mission = false;
    _repeat_dist = 0;
}

// store the latest reported position incase of mission exit and rewind resume
void AP_Mission::update_exit_position(void)
{
    if (!AP::ahrs().get_location(_exit_position)) {
        _exit_position.lat = 0;
        _exit_position.lng = 0;
    }
}

// calculate the location of the _resume_cmd wp and set as current
bool AP_Mission::calc_rewind_pos(Mission_Command& rewind_cmd)
{
    // check for a recent history
    if (_wp_index_history[LAST_WP_PASSED] == AP_MISSION_CMD_INDEX_NONE) {
        // no saved history so can't rewind
        return false;
    }

    // check that we have a valid exit position
    if (_exit_position.lat == 0 && _exit_position.lng == 0) {
        return false;
    }

    Mission_Command temp_cmd;
    float rewind_distance = _repeat_dist; //(m)
    uint16_t resume_index;
    Location prev_loc = _exit_position;

    for (int8_t i = (LAST_WP_PASSED); i>=0; i--) {

        // to get this far there has to be at least one 'passed wp' stored in history.  This is to check incase
        // of history array no being completely filled with valid waypoints upon resume.
        if (_wp_index_history[i] == AP_MISSION_CMD_INDEX_NONE) {
            // no more stored history
            resume_index = i+1;
            break;
        }

        if (!read_cmd_from_storage(_wp_index_history[i], temp_cmd)) {
            // if read from storage failed then don't use rewind
            return false;
        }

        // calculate distance
        float disttemp = prev_loc.get_distance(temp_cmd.content.location); //(m)
        rewind_distance -= disttemp;
        resume_index = i;

        if (rewind_distance <= 0.0f) {
            // history rewound enough distance to meet _repeat_dist requirement
            rewind_cmd = temp_cmd;
            break;
        }

        // store wp location as previous
        prev_loc = temp_cmd.content.location;
    }

    if (rewind_distance > 0.0f) {
        // then the history array was rewound all of the way without finding a wp distance > _repeat_dist distance.
        // the last read temp_cmd will be the furthest cmd back in the history array so resume to that.
        rewind_cmd = temp_cmd;
        return true;
    }

    // if we have got this far the desired rewind distance lies between two waypoints stored in history array.
    // calculate the location for the mission to resume

    // the last wp read from storage is the wp that is before the resume wp in the mission order
    Location passed_wp_loc = temp_cmd.content.location;

    // fetch next destination wp
    if (!read_cmd_from_storage(_wp_index_history[resume_index+1], temp_cmd)) {
        // if read from storage failed then don't use rewind
        return false;
    }

    // determine the length of the mission leg that the resume wp lies in
    float leg_length = passed_wp_loc.get_distance(temp_cmd.content.location); //(m)

    // calculate the percentage along the leg that resume wp will be positioned
    float leg_percent = fabsf(rewind_distance)/leg_length;

    // calculate difference vector of mission leg
    Vector3f dist_vec = passed_wp_loc.get_distance_NED(temp_cmd.content.location);

    // calculate the resume wp position
    rewind_cmd.content.location.offset(dist_vec.x * leg_percent, dist_vec.y * leg_percent);
    rewind_cmd.content.location.alt -= dist_vec.z * leg_percent * 100; //(cm)

    // The rewind_cmd.index has the index of the 'last passed wp' from the history array.  This ensures that the mission order
    // continues as planned without further intervention.  The resume wp is not written to memory so will not perminantely change the mission.

    // if we got this far then mission rewind position was successfully calculated
    return true;
}

/*
  handle format conversion of storage format to allow us to update
  format to take advantage of new packing. This is particularly useful
  for conversion to Float16 to get extra parameter space
*/
void AP_Mission::format_conversion(uint8_t tag_byte, const Mission_Command &cmd, PackedContent &packed_content) const
{
    // currently only one conversion needed, more can be added
#if AP_SCRIPTING_ENABLED
    if (tag_byte == 0 && cmd.id == MAV_CMD_NAV_SCRIPT_TIME) {
        // PARAMETER_CONVERSION: conversion code added Oct 2022
        struct nav_script_time_Command_tag0 old_fmt;
        struct nav_script_time_Command new_fmt;
        memcpy((void*)&old_fmt, packed_content.bytes, sizeof(old_fmt));
        new_fmt.command = old_fmt.command;
        new_fmt.timeout_s = old_fmt.timeout_s;
        new_fmt.arg1.set(old_fmt.arg1);
        new_fmt.arg2.set(old_fmt.arg2);
        new_fmt.arg3 = 0;
        new_fmt.arg4 = 0;
        memcpy(packed_content.bytes, (void*)&new_fmt, sizeof(new_fmt));
    }
#endif
}

// singleton instance
AP_Mission *AP_Mission::_singleton;

namespace AP
{

AP_Mission *mission()
{
    return AP_Mission::get_singleton();
}

}
