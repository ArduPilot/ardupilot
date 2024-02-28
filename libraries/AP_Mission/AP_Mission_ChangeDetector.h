/// @file    AP_Mission_ChangeDetector.h
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

#include "AP_Mission.h"

#if AP_MISSION_ENABLED

/// @class    AP_Mission_ChangeDetector
/// @brief    Mission command change detector
class AP_Mission_ChangeDetector
{

public:

    // check for changes to mission. returns true if mission has been changed since last check
    bool check_for_mission_change() WARN_IF_UNUSED;

private:

    // number of upcoming commands to monitor for changes
    static const uint8_t mis_change_detect_cmd_max = 3;
    struct {
        uint32_t last_change_time_ms;       // local copy of last time mission was changed
        uint16_t curr_cmd_index;            // local copy of AP_Mission's current command index
        uint8_t cmd_count;                  // number of commands in the cmd array
        AP_Mission::Mission_Command cmd[mis_change_detect_cmd_max]; // local copy of the next few mission commands
    } mis_change_detect;
};

#endif // AP_MISSION_ENABLED
