// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file    AP_Mission.h
/// @brief   Handles the MAVLINK command mission stack.  Reads and writes mission to storage.

/*
 *   The AP_Mission library:
 *   - Conditional and Do commands associated with the current leg may be requested
 *   - Reads and writes the mission MAVLINK commands to and from storage.
 *   - Performs error checking on waypoint values.
 *   - Keeps track of the indicies of current nav command and non_nav_waypoints.
 *   - Accounts for the DO_JUMP command in how is sequences waypoints.
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

#define AP_MISSION_CMD_BLANK 0

// Command/Waypoint/Location Options Bitmask
//--------------------
#define AP_MISSION_MASK_OPTIONS_RELATIVE_ALT       (1<<0)     // 1 = Relative altitude
#define AP_MISSION_MASK_OPTIONS_LOITER_DIRECTION   (1<<2)     // 0 = CW, 1 = CCW

/// @class    AP_Mission
/// @brief    Object managing Mission
class AP_Mission {

public:
    AP_Mission(struct Location &current_loc, uint16_t start_byte = 0x500) :
        _start_byte(start_byte),
        _current_loc(current_loc)
    {
        AP_Param::setup_object_defaults(this, var_info);
        _index[0]=0; _index[1]=0; _index[2]=0;
        _mission_status = false;
    }

    /*-------------------Commonly Used Methods --------------------------*/

    /*  Initialize the mission index and waypoint array with data in storage.
     *  Execute this first. */
    void        init_commands();

    /*  Return the waypoints in the mission stack.
     *   prev_wp:    Behind the vehicle.
     *   current_wp: Vehicle is navigating towards this waypoint.
     *   after_wp:   The waypoint that the vehicle would navigate towards after
     *               reaching current_wp  */
    const struct Location &         prev_wp() const {
        return _nav_waypoints[0];
    }
    const struct Location &         current_wp() const {
        return _nav_waypoints[1];
    }
    const struct Location &         after_wp() const {
        return _nav_waypoints[2];
    }

    /*Sequencies the entire waypoint queue to the next waypoint.
     *  Returns false if there is an error.  */
    bool        increment_waypoint_index();

    /* Forces a reset of the command queue to a specified waypoint.
    *  Returns false if a non_nav command is requested or error. */
    bool        change_waypoint_index(uint8_t new_index);

    /*Gets a new command associated with current leg of the mission.
     *  Each time this is called a new command is returned.
     *  Returns false if error or if there are no more commands.  */
    bool        get_new_cmd(struct Location &new_cmd);

    /* Returns the overall health of the mission.  If the mission is complete, or
     * there is an error, this will return false. */
    bool        get_status() const {
        return _mission_status;
    }

    /*--------------------------Specific Purpose Methods-------------------*/

    /* An override that sets the waypoint and command stack approprately for
     *  going home, to waypoint 0. */
    void        goto_home();

    /* An override that sets the waypoint and command stack approprately for
     *  going to an arbritary location */
    bool        goto_location(const struct Location &wp);

    /* Overrides the current waypoint altitude */
    void        override_altitude(const int32_t &altitude) {
        _nav_waypoints[1].alt=altitude;
    }

    /* Resume the mission.  Used when transitioning into AUTO modes from other modes */
    void        resume();

    /* Forces the previous wp to a defined location.  Typically used when transitioning into
     *  another mode from AUTO. */
    void        override_prev_wp(const struct Location &wp);

    /* Forces the current waypoint to a defined location. */
    void        set_current_wp(const struct Location &wp) {
        _nav_waypoints[1] = wp;
    }

    /*---------------------Utility Methods------------------------------*/

    /*returns just the index.  */
    uint8_t        waypoint_index() const {
        return _index[1];
    }
    uint8_t        prev_waypoint_index() const {
        return _index[0];
    }
    uint8_t        after_waypoint_index() const {
        return _index[2];
    }

    /*returns just the current command index.  */
    uint8_t        command_index() const {
        return _cmd_index;
    }

    //gets the total number of commands in the mission.
    uint8_t        command_total() const {
        return _cmd_max;
    }

    /*Sets the total number of commands in the mission.
     *  Used when a new mission is uploaded by a planner. */
    void        set_command_total(uint8_t max_index);

    /*Sets the home location, and writes it to storage */
    void        set_home(const struct Location &home);

    /*Returns home location and altitudes */
    struct Location         get_home() const {
        return _home;
    }
    int32_t        get_home_alt() const {
        return _home.alt;
    }

    /*Returns altitude to RTL, specified in parameter */
    int32_t        get_home_hold_alt() const {
        return (_home.alt + _RTL_altitude_cm);
    }


    //Low(er) level functions to store commands and waypoints into storage.
    struct Location         get_cmd_with_index(int16_t inx);
    struct Location         get_cmd_with_index_raw(int16_t inx);
    void                    set_cmd_with_index(struct Location &temp, uint16_t inx);
    uint16_t                _start_byte;

    // this supports the Mission_* user settable parameters
    static const struct AP_Param::GroupInfo        var_info[];

private:

    /* nav_waypoints is a 3 element array that contain navigation waypoints.
     *  nav_waypoints[0]: Previous Waypoint
     *  nav_waypoints[1]: Current Waypoint, the vehicle is navigating towards this.
     *  nav_waypoints[2]: After Waypoint
     *  The vehicle is typically located between waypoints corresponding to indicies 0 and 1 */
    struct Location        _nav_waypoints[3];

    /*Starts from search_index, and searches forward (=1) or
     *  backward (=0) for the next navigation waypoint.  This function is mindful of DO_JUMP
     *  commands.  Returns the index of the found navigation waypoint.  */
    uint8_t        _find_nav_index(uint8_t search_index);

    //Checks a navigation command for validity.
    bool        _check_nav_valid(const struct Location &temp);

    /* Takes the input index, sets as _index[1] and rebases index[2] to that index.
     *  Returns false if it fails to rebase to the index.  */
    bool        _sync_waypoint_index(const uint8_t &new_index);

    //Synchronizes the nav_waypoints array with the indicies in _index
    void        _sync_nav_waypoints();

    void        _safe_home(struct Location &safe_home);

    //Current index for non navigation commands
    uint8_t        _cmd_index;

    //A flag that marks if the mission is finished or invalid.
    bool        _mission_status;

    //The stack of indicies for prev, current, and after waypoints
    uint8_t        _index[3];
    /*A flag that marks if the previous index was overridden.  Required to prevent non_nav
     *  commands from being executed when overriding the pre-programed mission sequence. */
    bool        _prev_index_overriden;

    // The home location.  Altitude does not include RTL altitude.
    struct Location        _home;

    //Total number of commands in storage.
    AP_Float        _cmd_max;

    //The user specified RTL altitude.
    AP_Float        _RTL_altitude_cm;

    //The address of the current location, provided by vehicle code.
    struct Location &      _current_loc;

};

#endif
