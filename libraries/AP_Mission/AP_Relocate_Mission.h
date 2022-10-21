/*
 *   Created: 08.05.2020
 *    Author: Dipl.Ing.(FH)WillyZehnder
 */
/// @file    AP_Relocate_Mission.h
/// @brief   translates and rotates Missions according to the location where Mode-AUTO has been switched on
/*
 *   The AP_Relocate_Mission library:
 *   - memorizes the location where Mode-AUTO has been switched on (Base-Point)
 *   - moves the individual Waypoints according to Base-Point and switch-setting
 */
#pragma once

#include <AP_Vehicle/ModeReason.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

    class AP_Mission;

/// @class    AP_Relocate_Mission
/// @brief    Object managing movement of Waypoints
class AP_Relocate_Mission
{
    friend class AP_Mission;

public:

    AP_Relocate_Mission(void);
    /* Do not allow copies */
    AP_Relocate_Mission(const AP_Relocate_Mission &other) = delete;
    AP_Relocate_Mission &operator=(const AP_Relocate_Mission&) = delete;
    static AP_Relocate_Mission *get_singleton(void) {
        return _singleton;
    }

private:

    static AP_Relocate_Mission *_singleton;

    enum class Restart_Behaviour {
        RESTART_NOT_TRANSLATED,
        RESTART_PARALLEL_TRANSLATED,
        RESTART_ROTATED_HEADING
    };

    // for lat/lng translation of a Relative Mission
    struct Translation {
        int32_t alt;            // altitude-displacement [cm]
        int32_t direction;      // direction from HomePoint to Basepoint [10^2deg] North=0 East=9000
        bool    do_translation; // flag for release or blocking of translation
        bool    calculated;     // flag if first Waypoint is still proceeded and displacement is calculated
        };

    // internal variables
    Restart_Behaviour _restart_behaviour;     // behavior at restart of a Mission
    int32_t           _no_translation_radius; // distance in [m] from HomeLocation wherein a translation of a Relative Mission will be ignored

    struct Location         _basepoint_loc; // location where the MODE has been switched to AUTO
    struct Location         _first_wp_loc;  // original location of very first Waypoint of a Mission
    struct Translation      _translation;   // info concerning the translation of a  Relative Mission

    /// memorizes the Location and Attitudes at Base-Point
    void memorize_basepoint(void);

    /// set _translation.do_translation = false
    void set_no_translation();

    /// if the Command id is a Waypoint, the Location will be moved according to switch-setting
    void move_location(Location& loc, const uint16_t id);

    /// translate a Waypoint location
    void translate_location(Location& loc);

    /// rotate a Waypoint location
    void rotate_location(Location& loc);

};

namespace AP {
    AP_Relocate_Mission &mission_relative();
};
