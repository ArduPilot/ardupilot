// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file    AP_Mission.h
/// @brief   Handles the MAVLINK command mission stack.  Reads and writes mission to storage.

/*
 *   The AP_Mission library:
 *   - responsible for managing a list of commands made up of "nav", "do" and "conditional" commands
 *   - reads and writes the mission commands to storage.
 *   - provides easy acces to current, previous and upcoming waypoints
 *   - calls main program's command execution and verify functions.
 *   - accounts for the DO_JUMP command
 *
 */
#ifndef AP_Mission_h
#define AP_Mission_h

#include <GCS_MAVLink.h>
#include <AP_Math.h>
#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_AHRS.h>
#include <AP_HAL.h>

// definitions
#define AP_MISSION_EEPROM_MAX_ADDR          4096    // parameters get the first 1536 bytes of EEPROM, remainder is for waypoints
#define AP_MISSION_EEPROM_START_BYTE        0x600   // where in memory home WP is stored, all mission commands appear afterthis
#define AP_MISSION_EEPROM_COMMAND_SIZE      15      // size in bytes of all mission commands
#define AP_MISSION_FENCEPOINTS_MAX          6       // we reserve space for 6 fence points at the end of EEPROM although this is not currently implemented
#define AP_MISSION_FENCEPOINTS_SIZE         sizeof(Vector2l)    // each fence points size in eeprom
#define AP_MISSION_FENCE_START_BYTE         (AP_MISSION_EEPROM_MAX_ADDR-(AP_MISSION_FENCEPOINTS_MAX*AP_MISSION_FENCEPOINTS_SIZE))   // space reserved for fence points
#define AP_MISSION_MAX_COMMANDS             ((AP_MISSION_FENCE_START_BYTE - AP_MISSION_EEPROM_START_BYTE) / AP_MISSION_EEPROM_COMMAND_SIZE) - 1 // -1 to be safe

#define AP_MISSION_MAX_NUM_DO_JUMP_COMMANDS 3       // only allow up to 3 do-jump commands (due to RAM limitations on the APM2)

#define AP_MISSION_CMD_ID_NONE              0       // mavlink cmd id of zero means invalid or missing command
#define AP_MISSION_CMD_INDEX_NONE           255     // command index of 255 means invalid or missing command
#define AP_MISSION_JUMP_TIMES_MAX           255     // maximum number of times a jump can be executed.  Used when jump tracking fails (i.e. when too many jumps in mission)

/// @class    AP_Mission
/// @brief    Object managing Mission
class AP_Mission {

public:

    // jump command structure
    struct Jump_Command {
        uint8_t id;         // mavlink command id.  To-Do: this can be removed once it is also removed from Location structure
        uint8_t target;     // DO_JUMP target command id
        uint8_t num_times;  // DO_JUMP num times to repeat
    };

    union Content {
        // jump structure
        Jump_Command jump;

        // location
        Location location;      // Waypoint location

        uint8_t bytes[AP_MISSION_EEPROM_COMMAND_SIZE];
    };

    // command structure
    struct Mission_Command {
        uint8_t index;              // this commands position in the command list
        uint8_t id;                 // mavlink command id
        Content content;
    };

    // main program function pointers
    typedef bool (*mission_cmd_fn_t)(const Mission_Command& cmd);
    typedef void (*mission_complete_fn_t)(void);

    // mission state enumeration
    enum mission_state {
        MISSION_STOPPED=0,
        MISSION_RUNNING=1,
        MISSION_COMPLETE=2
    };

    /// constructor
    AP_Mission(mission_cmd_fn_t cmd_start_fn, mission_cmd_fn_t cmd_verify_fn, mission_complete_fn_t mission_complete_fn) :
        _cmd_start_fn(cmd_start_fn),
        _cmd_verify_fn(cmd_verify_fn),
        _mission_complete_fn(mission_complete_fn)
    {
        // load parameter defaults
        AP_Param::setup_object_defaults(this, var_info);

        // clear commands
        _nav_cmd.index = AP_MISSION_CMD_INDEX_NONE;
        _do_cmd.index = AP_MISSION_CMD_INDEX_NONE;

        // initialise other internal variables
        _flags.state = MISSION_STOPPED;
        _flags.nav_cmd_loaded = false;
        _flags.do_cmd_loaded = false;
    }

    ///
    /// mission methods
    ///

    /// status - returns the status of the mission (i.e. Mission_Started, Mission_Complete, Mission_Stopped
    mission_state state() const { return _flags.state; }

    /// num_commands - returns total number of commands in the mission
    uint8_t num_commands() const { return _cmd_total; }

    /// update - ensures the command queues are loaded with the next command and calls main programs command_init and command_verify functions to progress the mission
    ///     should be called at 10hz or higher
    void update();

    /// start - resets current commands to point to the beginning of the mission
    ///     To-Do: should we validate the mission first and return true/false?
    void start();

    /// stop - stops mission execution.  subsequent calls to update() will have no effect until the mission is started or resumed
    void stop();

    /// resume - continues the mission execution from where we last left off
    ///     previous running commands will be re-initialised
    void resume();

    /// clear - clears out mission
    ///     returns true if mission was running so it could not be cleared
    bool clear();

    /// valid - validate the mission has no errors
    ///     currently only checks that the number of do-commands does not exceed the AP_MISSION_MAX_NUM_DO_JUMP_COMMANDS
    bool valid();

    ///
    /// command methods
    ///

    /// get_active_nav_cmd - returns the current "navigation" command
    const Mission_Command& get_current_nav_cmd() const { return _nav_cmd; }

    /// get_next_nav_cmd - gets next "navigation" command found at or after start_index
    ///     returns true if found, false if not found (i.e. reached end of mission command list)
    ///     accounts for do_jump commands
    bool get_next_nav_cmd(uint8_t start_index, Mission_Command& cmd);

    /// set_current_nav_cmd - sets the current "navigation" command to the command number
    ///     returns true if successful, false on failure (i.e. if the index does not refer to a navigation command)
    ///     current do and conditional commands will also be modified
    bool set_current_nav_cmd(uint8_t index);

    /// get_active_do_cmd - returns active "do" command
    const Mission_Command& get_current_do_cmd() const { return _do_cmd; }

    /// add_cmd - adds a command to the end of the command list and writes to storage
    ///     returns true if successfully added, false on failure
    ///     cmd.index is updated with it's new position in the mission
    bool add_cmd(Mission_Command& cmd);

    /// load_cmd_from_storage - load command from storage
    ///     true is return if successful
    bool read_cmd_from_storage(uint8_t index, Mission_Command& cmd) const;

    /// write_cmd_to_storage - write a command to storage
    ///     cmd.index is used to calculate the storage location
    ///     true is returned if successful
    bool write_cmd_to_storage(uint8_t index, Mission_Command& cmd);

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

private:

    struct Mission_Flags {
        mission_state state;
        uint8_t nav_cmd_loaded  : 1; // true if a "navigation" command has been loaded into _nav_cmd
        uint8_t do_cmd_loaded   : 1; // true if a "do"/"conditional" command has been loaded into _do_cmd
    } _flags;

    ///
    /// private methods
    ///

    /// complete - mission is marked complete and clean-up performed including calling the mission_complete_fn
    void complete();

    /// advance_current_nav_cmd - moves current nav command forward
    ///     do command will also be loaded
    ///     accounts for do-jump commands
    //      will call complete method if it reaches end of mission command list
    void advance_current_nav_cmd();

    /// advance_current_do_cmd - moves current do command forward
    ///     accounts for do-jump commands
    ///     returns true if successfully advanced (can it ever be unsuccessful?)
    void advance_current_do_cmd();

    /// is_nav_cmd - returns true if the command's id is a "navigation" command, false if "do" or "conditional" command
    bool is_nav_cmd(const Mission_Command& cmd) const;

    /// get_next_cmd - gets next command found at or after start_index
    ///     returns true if found, false if not found (i.e. mission complete)
    ///     accounts for do_jump commands
    ///     increment_jump_num_times_if_found should be set to true if advancing the active navigation command
    bool get_next_cmd(uint8_t start_index, Mission_Command& cmd, bool increment_jump_num_times_if_found);

    /// get_next_do_cmd - gets next "do" or "conditional" command after start_index
    ///     returns true if found, false if not found
    ///     stops and returns false if it hits another navigation command before it finds the first do or conditional command
    ///     accounts for do_jump commands but never increments the jump's num_times_run (get_next_nav_cmd is responsible for this)
    bool get_next_do_cmd(uint8_t start_index, Mission_Command& cmd);

    ///
    /// jump handling methods
    ///
    // init_jump_tracking - initialise jump_tracking variables
    void init_jump_tracking();

    /// get_jump_times_run - returns number of times the jump command has been run
    uint8_t get_jump_times_run(const Mission_Command& cmd);

    /// increment_jump_times_run - increments the recorded number of times the jump command has been run
    void increment_jump_times_run(Mission_Command& cmd);

    // parameters
    AP_Int16                _cmd_total; // total number of commands in the mission

    // pointer to main program functions
    mission_cmd_fn_t        _cmd_start_fn;  // pointer to function which will be called when a new command is started
    mission_cmd_fn_t        _cmd_verify_fn; // pointer to function which will be called repeatedly to ensure a command is progressing
    mission_complete_fn_t   _mission_complete_fn;   // pointer to function which will be called when mission completes

    // internal variables
    struct Mission_Command  _nav_cmd;   // current "navigation" command.  It's position in the command list is held in _nav_cmd.index
    struct Mission_Command  _do_cmd;    // current "do" command.  It's position in the command list is held in _do_cmd.index

    // jump related variables
    struct jump_tracking_struct {
        uint8_t index;                  // index of do-jump commands in mission
        uint8_t num_times_run;          // number of times
    } _jump_tracking[AP_MISSION_MAX_NUM_DO_JUMP_COMMANDS];
};

#endif
