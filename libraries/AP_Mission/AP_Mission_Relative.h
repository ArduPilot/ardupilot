/*
 * AP_Mission_Relative.h
 *
 *  Created on: 08.05.2020
 *      Author: WillyZehnder
 */
/// @file    AP_Mission_Relative.h
/// @brief   translates and rotates Missions according to the location where Mode-AUTO has been switched on

/*
 *   The AP_Mission_Relative library:
 *   - memorizes the Location where Mode-AUTO has been switched on (Base-Point)
 *   - moves the individual Waypoints according to Base-Point and Parameters
 */
#pragma once

#ifndef ENABLED
    #define ENABLED 1
#endif

#include <AP_Param/AP_Param.h>
#include <AP_Vehicle/ModeReason.h>

// definitions
#ifndef AP_MISSION_RELATIVE_NO_TRANSLATION_RADIUS_DEFAULT
    // no translation of the Mission within a radius of XX m around Homepoint (it's a compromise: for Copter/Rover/Boat 5-10m, for Plane 50-100m should be sufficient)
    #define AP_MISSION_RELATIVE_NO_TRANSLATION_RADIUS_DEFAULT 50
#endif
#define AP_MISSION_RELATIVE_KIND_OF_MOVE_DEFAULT    0   // no translation
#define AP_MISSION_RELATIVE_OPTIONS_DEFAULT         0   // don't use altitude offset

    class AP_Mission;

/// @class    AP_Mission_Relative
/// @brief    Object managing movement of Waypoints
class AP_Mission_Relative
{

    friend class AP_Mission;

public:

    AP_Mission_Relative(void);

    /* Do not allow copies */
    AP_Mission_Relative(const AP_Mission_Relative &other) = delete;
    AP_Mission_Relative &operator=(const AP_Mission_Relative&) = delete;

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

    static AP_Mission_Relative *get_singleton(void) {
        return _singleton;
    }

private:

    static AP_Mission_Relative *_singleton;

    // variables for Parameters
    AP_Int8     _kind_of_move;          // defines the kind of move (translation/rotation) of the mission when entering Auto mode
    AP_Int8     _rel_options;           // bitmask options for special behaviour at translation
    AP_Float    _no_translation_radius; // distance in [m] from HomeLocation wherein a translation of a Relative Mission will be ignored
    AP_Int8     _rotate_ch;             // channel to rotate by rc-channel-input

    AP_Int8     _rel_options_fixed;     // bitmask options for special behaviour at translation - fixed at Mission-Start

    enum class Restart_Behaviour {
        RESTART_NOT_TRANSLATED,
        RESTART_PARALLEL_TRANSLATED,
        RESTART_ROTATED_NORTH,
        RESTART_ROTATED_FIRST_WP,
        RESTART_ROTATED_HEADING,
        RESTART_ROTATED_CHANNEL
    };

    const char* _behaviour_info[6] = {
        "Not Translated",
        "Parallel Translated",
        "Rotated related 1st WP",
        "Rotated related North",
        "Rotated by Heading",
        "Rotated by RC-Channel"
    };

    // for lat/lng translation of a Relative Mission
    struct Translation {
        int32_t alt;            // altitude-displacement of location where AUTO has been switched on, relative to first waypoint [cm]
        int32_t direction;      // direction from HomePoint to the location where AUTO has been switched on  [10^2°] North=0 East=9000
        bool    do_translation; // for marking if location where AUTO has been switched is far enough from home-location
        bool    calculated;     // for marking if first waypoint is still proceeded and displacement is calculated
        };

    // for lat/lng rotation of a Relative Mission
    struct Rotation {
        int32_t lat;            // latitude-displacement of location where AUTO has been switched on relative to current translated waypoint [10^7°]
        int32_t lng;            // longitude-displacement of location where AUTO has been switched on relative to current translated waypoint [10^7°]
        int32_t direction;      // direction from the location where AUTO has been switched on to the current translated waypoint [10^2°] North=0 East=9000
        };

    // pw values for rotating by RC-channel (!don't change, because of simplification of the formula in the program-code!)
    static const uint16_t PW_MINUS_180_DEG = 1000;
    static const uint16_t PW_PLUS_180_DEG  = 2000;

    // internal variables
    Restart_Behaviour _restart_behaviour;    // behaviour at restart of a Mission

    union MoveKind { // for moving the Parameter-value to the enum-class
        Restart_Behaviour   _rb;
        AP_Int8             _kom;
    };

    struct Location         _basepoint_loc; // location where the MODE has been switched to AUTO
    struct Location         _first_wp_loc;  // original location of very first Waypoint
    struct Translation      _translation;   // info concerning the translation of a  Relative Mission

    /// memorizes the Location and Attitudes at Base-Point
    void memorize(ModeReason& mode_reason);

    /// set _translation.do_translation = false (necessary after DO_LAND_START has been detected)
    void set_no_translation();

    /// move - if the Command is a Waypoint the Location will be moved according to Parameters
    void moveloc(Location& loc, const uint16_t id);

    /// translate - if the Command is a Waypoint the Location will be translated according to Parameters
    void translate(Location& loc);

    /// rotate - if the Command is a Waypoint the Location will be rotated according to Parameters
    void rotate(Location& loc);

};

namespace AP {
    AP_Mission_Relative &mission_relative();
};
