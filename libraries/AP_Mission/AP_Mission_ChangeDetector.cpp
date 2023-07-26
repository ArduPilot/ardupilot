/// @file    AP_Mission_ChangeDetector.cpp
/// @brief   Detects changes in the next few nav commands in the mission

#include "AP_Mission_ChangeDetector.h"

#if AP_MISSION_ENABLED

// detect external changes to mission
bool AP_Mission_ChangeDetector::check_for_mission_change()
{
    AP_Mission *mission = AP::mission();
    if (mission == nullptr) {
        return false;
    }

    // check if mission has been updated
    const uint32_t change_time_ms = mission->last_change_time_ms();
    const bool update_time_changed = (change_time_ms != mis_change_detect.last_change_time_ms);

    // check if active command index has changed
    const uint16_t curr_cmd_idx = mission->get_current_nav_index();
    const bool curr_cmd_idx_changed = (curr_cmd_idx != mis_change_detect.curr_cmd_index);

    // no changes if neither mission update time nor active command index has changed
    if (!update_time_changed && !curr_cmd_idx_changed) {
        return false;
    }

    // the mission has been updated (but maybe not changed) and/or the current command index has changed
    // check the contents of the next three commands to ensure they have not changed
    // and update storage so we can detect future changes

    bool cmds_changed = false;  // true if upcoming command contents have changed

    // retrieve cmds from mission and compare with mis_change_detect
    uint8_t num_cmds = 0;
    uint16_t cmd_idx = curr_cmd_idx;
    AP_Mission::Mission_Command cmd[mis_change_detect_cmd_max];
    while ((num_cmds < ARRAY_SIZE(cmd)) && mission->get_next_nav_cmd(cmd_idx, cmd[num_cmds])) {
        num_cmds++;
        if ((num_cmds > mis_change_detect.cmd_count) || (cmd[num_cmds-1] != mis_change_detect.cmd[num_cmds-1])) {
            cmds_changed = true;
            mis_change_detect.cmd[num_cmds-1] = cmd[num_cmds-1];
        }
        cmd_idx = cmd[num_cmds-1].index+1;
    }

    // mission has changed if number of upcoming commands does not match mis_change_detect
    if (num_cmds != mis_change_detect.cmd_count) {
        cmds_changed = true;
    }

    // update mis_change_detect with last change time, command index and number of commands
    mis_change_detect.last_change_time_ms = change_time_ms;
    mis_change_detect.curr_cmd_index = curr_cmd_idx;
    mis_change_detect.cmd_count = num_cmds;

    // mission has changed if upcoming command contents have changed without the current command index changing
    return cmds_changed && !curr_cmd_idx_changed;
}

#endif  // AP_MISSION_ENABLED
