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
#define AP_MISSION_MASK_OPTIONS_RELATIVE_ALT    (1<<0)        // 1 = Relative altitude
#define AP_MISSION_MASK_OPTIONS_LOITER_DIRECTION   (1<<2)     // 0 = CW, 1 = CCW

/// @class    AP_Mission
/// @brief    Object managing one Mission
class AP_Mission {

public:
    AP_Mission(AP_AHRS *ahrs, uint8_t wp_size = 15, uint16_t start_byte = 0x500) : 
        _ahrs(ahrs),
        _wp_size(wp_size),
        _start_byte(start_byte)
    	{
			AP_Param::setup_object_defaults(this, var_info);
            _index[0]=0; _index[1]=0; _index[2]=0;
            _mission_status = false;
		}

    /*---------Basic Functions and variables--------*/

    /* Initialize the mission index and waypoint array with data in storage.
     *  Do this first. */
    void        init_commands();

    struct      Location prev_wp()    {return _nav_waypoints[0];};
    struct      Location current_wp() {return _nav_waypoints[1];};
    struct      Location after_wp()   {return _nav_waypoints[2];};
    
    void        goto_home();
    bool        goto_location(const struct Location &wp);
    
    /* Forces the previous wp to a vehicle defined location */
    void        set_prev_wp(const struct Location &wp) { _nav_waypoints[0] = wp; } ;
    void        set_current_wp(const struct Location &wp) { _nav_waypoints[1] = wp; } ;
    
    /*Sequencies the entire waypoint queue to the next waypoint.
     *  Returns false if there is an error.  */
    bool        increment_waypoint_index();

    /* Forces a reset of the command queue to a specified waypoint.
    *  Returns false if a non_nav command is requested or error. */
    bool        change_waypoint_index(const uint8_t &new_index);

    /*Gets a new command associated with current leg of the mission.
     *  Each time this is called a new command is returned.
     *  Returns false if error or if there are no more commands.  */
    bool        get_new_cmd(struct Location &new_cmd);

    /* Returns the overall health of the mission.  If the mission is complete, or
     * there is an error, this will return false. */
    bool        get_mission_status() {return _mission_status;};

    /*---------------------Utility Functions-------------------*/


    uint8_t *      waypoint_array_index()           {
        return _index;
    };
    
    /*returns just the current index.  */
    uint8_t        waypoint_index()    {
        return _index[1];
    };
    
    /*returns just the current command index.  */
    uint8_t        command_index()    {
        return _cmd_index;
    };
    
    //gets the total number of commands in the mission.
    uint8_t        command_total();

    /*Sets the total number of commands in the mission.
     *  Used when a new mission is uploaded by a planner. */
    void        set_command_total(uint8_t max_index);

    /*Sets the home location, and writes it to storage */
    void        set_home(const struct Location &home);

    /*Returns home location */
    const struct Location           get_home()            {
        return _home;
    };
    
    const int32_t get_home_alt()    {
        return _home.alt;
    };
    
    //Low(er) level functions to store commands and waypoints into storage.
    struct Location         get_cmd_with_index(int16_t inx);
    struct Location         get_cmd_with_index_raw(int16_t inx);
    void                    set_cmd_with_index(struct Location &temp, uint16_t inx);
    uint16_t _start_byte;
    uint8_t _wp_size;
    
    // this supports the Mission_* user settable parameters
    static const struct AP_Param::GroupInfo var_info[];
    
private:
    
    /* nav_waypoints is a 3 element array that contain navigation waypoints.
     *  nav_waypoints[0]: Previous Waypoint
     *  nav_waypoints[1]: Current Waypoint, the vehicle is navigating towards this.
     *  nav_waypoints[2]: After Waypoint
     *  The vehicle is typically located between waypoints corresponding to indicies 0 and 1 */
    struct Location     _nav_waypoints[3];
    
    /*Starts from search_index, and searches forward (=1) or
     *  backward (=0) for the next navigation waypoint.  This function is mindful of DO_JUMP
     *  commands.  Returns the index of the found navigation waypoint.  */
    uint8_t             _find_nav_index(uint8_t search_index);

    //Checks a navigation command for validity.
    bool                _check_nav_valid(const struct Location &temp);

    /* Takes the input index, sets as _index[1] and rebases index[2] to that index.
     *  Returns false if it fails to rebase to the index.  */
    bool                _sync_waypoint_index(const uint8_t &new_index);

    //Synchronizes the nav_waypoints array with the indicies in _index
    void                _sync_nav_waypoints();

    //Current index for non navigation commands
    uint8_t             _cmd_index;

    bool                _mission_status;
    uint8_t             _index[3];
    struct Location     _home;
    AP_Float            _cmd_max;
    
    // reference to the AHRS object
    AP_AHRS *_ahrs;
    struct Location _current_loc;
    

};

#endif
