/// @file    AP_Mission_ChangeDetector_Copter.h
/// @brief   Detects changes in the next few nav commands in the mission

/*
 *   The AP_Mission_ChangeDetector library:
 *   - records the index of the active nav command
 *   - maintains a copy of the next few navigation commands
 *   - checks for changes in either the active command or the next few nav commands
 *
 *   Detecting changes in the next few nav commands is critical for SCurves and splines
 *   which plan the path through the corners
 */
#pragma once

#include "AP_Mission_ChangeDetector.h"

#if HAL_MISSION_ENABLED

/// @class    AP_Mission_ChangeDetector
/// @brief    Mission command change detector
class AP_Mission_ChangeDetector_Copter: private AP_Mission_ChangeDetector
{

public:

    enum ChangeResponseType {
        NONE = 0,           // no response to change is required
        ADD_NEXT_WAYPOINT,  // add next waypoint when convenient
        RESET_REQUIRED      // reload / reset of active commands is required
    };

    // check for changes to mission. returns required response to change (if any)
    // using_next_command should be set to try if waypoint controller is already using the next navigation command
    ChangeResponseType check_for_mission_change(bool using_next_command) WARN_IF_UNUSED;

};

#endif // HAL_MISSION_ENABLED
