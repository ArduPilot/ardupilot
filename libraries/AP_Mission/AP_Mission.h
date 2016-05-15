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

//BEV this entire file and AP_Mission.cpp were extensively modified to make them simpler
//and fix the numerous bugs inherent

#ifndef AP_Mission_h
#define AP_Mission_h

#include <AP_HAL.h>
#include <AP_Vehicle.h>
#include <GCS_MAVLink.h>
#include <AP_Math.h>
#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_AHRS.h>
#include <../StorageManager/StorageManager.h>

// definitions
#define AP_MISSION_EEPROM_VERSION           0x65AE  // version number stored in first four bytes of eeprom.  increment this by one when eeprom format is changed
#define AP_MISSION_EEPROM_COMMAND_SIZE      15      // size in bytes of all mission commands

#define AP_MISSION_MAX_NUM_DO_JUMP_COMMANDS 15    // allow up to 15 do-jump commands all high speed CPUs

#define AP_MISSION_JUMP_REPEAT_FOREVER      -1      // when do-jump command's repeat count is -1 this means endless repeat

#define AP_MISSION_CMD_ID_NONE              0       // mavlink cmd id of zero means invalid or missing command
#define AP_MISSION_CMD_INDEX_NONE           65535   // command index of 65535 means invalid or missing command
#define AP_MISSION_JUMP_TIMES_MAX           32767   // maximum number of times a jump can be executed.  Used when jump tracking fails (i.e. when too many jumps in mission)

#define AP_MISSION_FIRST_REAL_COMMAND       1       // command #0 reserved to hold home position

#define AP_MISSION_RESTART_DEFAULT          0       // resume the mission from the last command run by default

/// @class    AP_Mission
/// @brief    Object managing Mission
class AP_Mission {

public:
    // jump command structure
    struct PACKED Jump_Command {
        uint16_t target;        // target command id
        int16_t num_times;      // num times to repeat.  -1 = repeat forever
    };

    // set servo command structure
    struct PACKED Set_Servo_Command {
        uint8_t channel;        // servo channel
        uint16_t pwm;           // pwm value for servo
    };

    // set cam trigger distance command structure
    struct PACKED Cam_Trigg_Distance {
        float meters;           // distance
    };

    union PACKED Content {
        // jump structure
        Jump_Command jump;

        // do-set-servo
        Set_Servo_Command servo;

        // cam trigg distance
        Cam_Trigg_Distance cam_trigg_dist;

        // location
        Location location;      // Waypoint location

        // raw bytes, for reading/writing to eeprom
        uint8_t bytes[12];
    };

    // command structure
    struct PACKED Mission_Command {
        uint16_t index;             // this commands position in the command list
        uint8_t id;                 // mavlink command id
        uint16_t p1;                // general purpose parameter 1
        Content content;
    };

    typedef bool (*mission_cmd_fn_t)(const Mission_Command& cmd);
    typedef void (*mission_complete_fn_t)(void);

    // mission state enumeration
    enum mission_state {
        MISSION_STOPPED=0,
        MISSION_RUNNING=1,
        MISSION_RUNNING_FORCE_RESTART=2,
        MISSION_COMPLETE=3
    };

    /// constructor
    AP_Mission(AP_AHRS &ahrs, mission_cmd_fn_t cmd_start_fn, mission_cmd_fn_t cmd_verify_fn, mission_complete_fn_t mission_complete_fn) :
        _ahrs(ahrs),
        _cmd_start_fn(cmd_start_fn),
        _cmd_verify_fn(cmd_verify_fn),
        _mission_complete_fn(mission_complete_fn),
        //_prev_nav_cmd_index(AP_MISSION_CMD_INDEX_NONE),
        _last_change_time_ms(0)
    {
        // load parameter defaults
        AP_Param::setup_object_defaults(this, var_info);

        // clear commands
        _cmd.index = AP_MISSION_CMD_INDEX_NONE;

        // initialise other internal variables
        _flags.state = MISSION_STOPPED;
        _flags.cmd_loaded = false;
    }

    ///
    /// public mission methods
    ///

    /// init - initialises this library including checks the version in eeprom matches this library
    void init();

    /// status - returns the status of the mission (i.e. Mission_Started, Mission_Complete, Mission_Stopped
    mission_state state() const { return _flags.state; }

    /// num_commands - returns total number of commands in the mission
    uint16_t num_commands() const { return _cmd_total; }

    /// num_commands_max - returns maximum number of commands that can be stored
    uint16_t num_commands_max() const;

    /// start - resets current commands to point to the beginning of the mission
    ///     To-Do: should we validate the mission first and return true/false?
    void start();

    /// stop - stops mission execution.  subsequent calls to update() will have no effect until the mission is started or resumed
    void stop();

    /// resume - continues the mission execution from where we last left off
    ///     previous running commands will be re-initialised
    void resume();

    /// start_or_resume - if MIS_AUTORESTART=0 this will call resume(), otherwise it will call start()
    void start_or_resume();

    /// reset - reset mission to the first command
    void reset();

    /// clear - clears out mission
    ///     returns true if mission was running so it could not be cleared
    bool clear();

    /// truncate - truncate any mission items beyond given index
    void truncate(uint16_t index);

    /// update - ensures the command queues are loaded with the next command and calls main programs command_init and command_verify functions to progress the mission
    ///     should be called at 10hz or higher
    void update();

    ///
    /// public command methods
    ///

    /// add_cmd - adds a command to the end of the command list and writes to storage
    ///     returns true if successfully added, false on failure
    ///     cmd.index is updated with it's new position in the mission
    bool add_cmd(Mission_Command& cmd);

    /// replace_cmd - replaces the command at position 'index' in the command list with the provided cmd
    ///     replacing the current active command will have no effect until the command is restarted
    ///     returns true if successfully replaced, false on failure
    bool replace_cmd(uint16_t index, Mission_Command& cmd);

    /// is_nav_cmd - returns true if the command's id is a "navigation" command, false if "do" or "conditional" command
    static bool is_nav_cmd(const Mission_Command& cmd);

    /// get_current_nav_cmd - returns the current "navigation" command
    const Mission_Command& get_current_nav_cmd() const { return _cmd; }

    /// get_current_nav_index - returns the current command index
    /// Note that this will return 0 if there is no command. This is
    /// used in MAVLink reporting of the mission command
    uint16_t get_current_nav_index() const { 
        return _cmd.index==AP_MISSION_CMD_INDEX_NONE?0:_cmd.index; }

    /// get_next_nav_cmd - gets next "navigation" command found at or after start_index
    ///     returns true if found, false if not found (i.e. reached end of mission command list)
    ///     accounts for do_jump commands
    bool get_next_nav_cmd(uint16_t start_index, Mission_Command& cmd);

    /// get the ground course of the next navigation leg in centidegrees
    /// from 0 36000. Return default_angle if next navigation
    /// leg cannot be determined
    int32_t get_next_ground_course_cd(int32_t default_angle);

    // set_current_cmd - jumps to command specified by index
    bool set_current_cmd(uint16_t index);

    /// load_cmd_from_storage - load command from storage
    ///     true is return if successful
    bool read_cmd_from_storage(uint16_t index, Mission_Command& cmd) const;

    /// write_cmd_to_storage - write a command to storage
    ///     cmd.index is used to calculate the storage location
    ///     true is returned if successful
    bool write_cmd_to_storage(uint16_t index, Mission_Command& cmd);

    /// write_home_to_storage - writes the special purpose cmd 0 (home) to storage
    ///     home is taken directly from ahrs
    void write_home_to_storage();

    // mavlink_to_mission_cmd - converts mavlink message to an AP_Mission::Mission_Command object which can be stored to eeprom
    //  return true on success, false on failure
    static bool mavlink_to_mission_cmd(const mavlink_mission_item_t& packet, AP_Mission::Mission_Command& cmd);

    // mission_cmd_to_mavlink - converts an AP_Mission::Mission_Command object to a mavlink message which can be sent to the GCS
    //  return true on success, false on failure
    static bool mission_cmd_to_mavlink(const AP_Mission::Mission_Command& cmd, mavlink_mission_item_t& packet);

    // return the last time the mission changed in milliseconds
    uint32_t last_change_time_ms(void) const { return _last_change_time_ms; }

    //BEV added this one
    void start_loiter_at_home(uint32_t alt);

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

private:
    static StorageAccess _storage;

    struct Mission_Flags {
        mission_state state;
        uint8_t cmd_loaded  : 1; // true if a "navigation" command has been loaded into _cmd
    } _flags;

    ///
    /// private methods
    ///

    /// complete - mission is marked complete and clean-up performed including calling the mission_complete_fn
    void complete();

    /// advance_current_cmd - moves current nav command forward
    ///     do command will also be loaded
    ///     accounts for do-jump commands
    //      returns true if command is advanced, false if failed (i.e. mission completed)
    bool advance_current_cmd();

    /// get_next_cmd - gets next command found at or after start_index
    ///     returns true if found, false if not found (i.e. mission complete)
    ///     accounts for do_jump commands
    ///     increment_jump_num_times_if_found should be set to true if advancing the active navigation command
    bool get_next_cmd(uint16_t start_index, Mission_Command& cmd, bool increment_jump_num_times_if_found);

    ///
    /// jump handling methods
    ///
    // init_jump_tracking - initialise jump_tracking variables
    void init_jump_tracking();

    /// get_jump_times_run - returns number of times the jump command has been run
    ///     return is signed to be consistent with do-jump cmd's repeat count which can be -1 (to signify to repeat forever)
    int16_t get_jump_times_run(const Mission_Command& cmd);

    /// increment_jump_times_run - increments the recorded number of times the jump command has been run
    void increment_jump_times_run(Mission_Command& cmd);

    /// check_eeprom_version - checks version of missions stored in eeprom matches this library
    /// command list will be cleared if they do not match
    void check_eeprom_version();

    // references to external libraries
    const AP_AHRS&   _ahrs;      // used only for home position

    // parameters
    AP_Int16                _cmd_total;  // total number of commands in the mission
    AP_Int8                 _restart;   // controls mission starting point when entering Auto mode (either restart from beginning of mission or resume from last command run)

    // pointer to main program functions
    mission_cmd_fn_t        _cmd_start_fn;  // pointer to function which will be called when a new command is started
    mission_cmd_fn_t        _cmd_verify_fn; // pointer to function which will be called repeatedly to ensure a command is progressing
    mission_complete_fn_t   _mission_complete_fn;   // pointer to function which will be called when mission completes

    // internal variables
    struct Mission_Command  _cmd;   // current "navigation" command.  It's position in the command list is held in _nav_cmd.index

    // jump related variables
    struct jump_tracking_struct {
        uint16_t index;                 // index of do-jump commands in mission
        int16_t num_times_run;          // number of times this jump command has been run
    } _jump_tracking[AP_MISSION_MAX_NUM_DO_JUMP_COMMANDS];

    // last time that mission changed
    uint32_t _last_change_time_ms;
};

#endif
