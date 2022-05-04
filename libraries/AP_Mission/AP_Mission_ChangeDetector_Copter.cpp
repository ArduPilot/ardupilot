/// @file    AP_Mission_ChangeDetector_Copter.cpp
/// @brief   Detects changes in the next few nav commands in the mission

#include "AP_Mission_ChangeDetector_Copter.h"

extern const AP_HAL::HAL& hal;

#define MIS_CHANGE_DETECT_DEBUG 1
#if MIS_CHANGE_DETECT_DEBUG
#include <GCS_MAVLink/GCS.h>
#define debug(fmt, args...) gcs().send_text(MAV_SEVERITY_CRITICAL, fmt, ##args)
#else
#define debug(fmt, args...)
#endif

// check for changes to mission. returns required response to change (if any)
// using_next_command should be set to try if waypoint controller is already using the next navigation command
AP_Mission_ChangeDetector_Copter::ChangeResponseType AP_Mission_ChangeDetector_Copter::check_for_mission_change(bool using_next_command)
{
    // take backup of command list
    MissionCommandList cmd_list_bak = mis_change_detect;

    uint8_t first_changed_cmd_idx = 0;
    if (!AP_Mission_ChangeDetector::check_for_mission_change(first_changed_cmd_idx)) {
        // the mission has not changed
        return AP_Mission_ChangeDetector_Copter::ChangeResponseType::NONE;
    }

    // if the current command has change a reset is always required
    // ToDo: check this handles mission erased
    if (first_changed_cmd_idx == 0) {
        debug("check_for_mission_change: 1st comment changed, Reset");
        return AP_Mission_ChangeDetector_Copter::ChangeResponseType::RESET_REQUIRED;
    }

    // 1st exists and 2nd or 3rd command has been added, changed or deleted

    const bool cmd0_is_wp = (mis_change_detect.cmd[0].id == MAV_CMD_NAV_WAYPOINT);
    const bool cmd0_is_loiter = (mis_change_detect.cmd[0].id == MAV_CMD_NAV_LOITER_UNLIM ||
                                 mis_change_detect.cmd[0].id == MAV_CMD_NAV_LOITER_TIME);
    const bool cmd0_is_spline = (mis_change_detect.cmd[0].id == MAV_CMD_NAV_SPLINE_WAYPOINT);
    const bool cmd0_is_other = (!cmd0_is_wp && !cmd0_is_loiter && !cmd0_is_spline);

    // if 1st segment is not a wp, loiter or spline then return NONE
    if (cmd0_is_other) {
        debug("check_for_mission_change: 1st is neither wp nor spline, None");
        return AP_Mission_ChangeDetector_Copter::ChangeResponseType::NONE;
    }

    const bool cmd0_has_pause = (mis_change_detect.cmd[0].p1 > 0);

    // if 1st segment wp/spline has pause or is loiter then return NONE
    if (((cmd0_is_wp || cmd0_is_spline) && cmd0_has_pause) || cmd0_is_loiter) {
        debug("check_for_mission_change: 1st has pause, None");
        return AP_Mission_ChangeDetector_Copter::ChangeResponseType::NONE;
    }

    const bool cmd1_exists = mis_change_detect.cmd_count >= 2;
    const bool cmd1_has_pause = (mis_change_detect.cmd[1].p1 > 0);
    const bool cmd1_is_wp = (mis_change_detect.cmd[1].id == MAV_CMD_NAV_WAYPOINT ||
                             mis_change_detect.cmd[1].id == MAV_CMD_NAV_LOITER_UNLIM ||
                             mis_change_detect.cmd[1].id == MAV_CMD_NAV_LOITER_TIME);
    const bool cmd1_is_spline = (mis_change_detect.cmd[1].id == MAV_CMD_NAV_SPLINE_WAYPOINT);
    const bool cmd1_is_other = (!cmd1_is_wp && !cmd1_is_spline);
    const bool cmd1_existed = cmd_list_bak.cmd_count >= 2;
    const bool cmd1_was_wp = (cmd_list_bak.cmd[1].id == MAV_CMD_NAV_WAYPOINT ||
                             cmd_list_bak.cmd[1].id == MAV_CMD_NAV_LOITER_UNLIM ||
                             cmd_list_bak.cmd[1].id == MAV_CMD_NAV_LOITER_TIME);
    const bool cmd1_was_spline = (cmd_list_bak.cmd[1].id == MAV_CMD_NAV_SPLINE_WAYPOINT);
    const bool cmd1_was_other = (!cmd1_was_wp && !cmd1_was_spline);

    // if 2nd segment forces a pause then return NONE
    if ((!cmd1_existed || cmd1_was_other) && (!cmd1_exists || cmd1_is_other)) {
        debug("check_for_mission_change: 1st is forced pause, None");
        return AP_Mission_ChangeDetector_Copter::ChangeResponseType::NONE;
    }

    // check for 1st is wp (without a pause), if add of 2nd wp then ADD_NEXT_WAYPOINT
    const bool cmd1_added = (cmd_list_bak.cmd_count == 1) && (mis_change_detect.cmd_count > 1);

    if (cmd0_is_wp && cmd1_added) {
        if (cmd1_is_wp) {
            // 2nd is wp
            debug("check_for_mission_change: 1st is wp, no pause, 2nd added, AddNextWP");
            return AP_Mission_ChangeDetector_Copter::ChangeResponseType::ADD_NEXT_WAYPOINT;
        } else if (cmd1_is_spline) {
            // 2nd is spline
            // Currently we don't support set_destination_speed_max after leg has been started
            // Therefore we can't add a spline without reseting
            debug("check_for_mission_change: 1st is wp, no pause, 2nd added but is spline, Reset");
            return AP_Mission_ChangeDetector_Copter::ChangeResponseType::RESET_REQUIRED;
        } else {
            // 2nd is nether wp or spline
            // should be handled by pause above so we should not reach here
            debug("check_for_mission_change: 1st is wp, no pause, 2nd added but not wp or spline, None");
            return AP_Mission_ChangeDetector_Copter::ChangeResponseType::NONE;
        }
    }

    // trajectory may be dependent on multiple waypoints, handle changed waypoints
    if (cmd0_is_wp) {
        // 1st is wp
        if (!cmd1_exists) {
            // 2nd deleted, Reset
            // This could be extended to check cmd_list_bak for the deleted waypoint and return REMOVE_NEXT_WAYPOINT
            debug("check_for_mission_change: 1st is wp, deleted 2nd wp, Reset");
            return AP_Mission_ChangeDetector_Copter::ChangeResponseType::RESET_REQUIRED;
        } else if (cmd1_is_wp && cmd1_was_wp) {
            // 2nd was and is wp
            if (first_changed_cmd_idx == 1) {
                // if 2nd has changed
                if (using_next_command) {
                    debug("check_for_mission_change: 1st is wp, 2nd wp changed, Reset");
                    return AP_Mission_ChangeDetector_Copter::ChangeResponseType::RESET_REQUIRED;
                } else {
                    debug("check_for_mission_change: 1st is wp, not using changed 2nd wp, None");
                    return AP_Mission_ChangeDetector_Copter::ChangeResponseType::NONE;
                }
            } else {
                // 2nd has not changed
                debug("check_for_mission_change: 1st is wp, 2nd wp same, None");
                return AP_Mission_ChangeDetector_Copter::ChangeResponseType::NONE;
            }
        } else if (cmd1_is_spline && cmd1_has_pause && (first_changed_cmd_idx == 2)) {
                // 2nd spline with pause has not changed
                // we do not check 3rd forces pause and change in forces pause
                debug("check_for_mission_change: 1st is wp, 2nd spline with pause not changed, None");
                return AP_Mission_ChangeDetector_Copter::ChangeResponseType::NONE;
        } else {
            // not all combinations will result in a changed state but to keep this simple we will call a reset
            debug("check_for_mission_change: 1st is wp, catch change, Reset");
            return AP_Mission_ChangeDetector_Copter::ChangeResponseType::RESET_REQUIRED;
        }
    } else {
        // 1st is spline (cmd0_is_loiter and cmd0_is_other have already been handled above)
        if (!cmd1_exists) {
            // 2nd deleted, Reset
            // This could be extended to check cmd_list_bak for the deleted waypoint and return REMOVE_NEXT_WAYPOINT
            debug("check_for_mission_change: 1st is spline, deleted 2nd wp, Reset");
            return AP_Mission_ChangeDetector_Copter::ChangeResponseType::RESET_REQUIRED;
        } else if (cmd1_is_wp) {
            // 2nd is wp
            if (first_changed_cmd_idx == 1) {
                // 2nd has changed
                debug("check_for_mission_change: 1st is spline, 2nd wp changed, Reset");
                return AP_Mission_ChangeDetector_Copter::ChangeResponseType::RESET_REQUIRED;
            } else {
                // 2nd not changed, 3rd must have changed
                debug("check_for_mission_change: 1st is spline, 2nd wp same, None");
                return AP_Mission_ChangeDetector_Copter::ChangeResponseType::NONE;
            }
        } else if (cmd1_is_spline && cmd1_has_pause && (first_changed_cmd_idx == 2)) {
            // 2nd spline with pause has not changed
            // we do not check 3rd forces pause and change in forces pause
            debug("check_for_mission_change: 1st is spline, 2nd spline with pause not changed, None");
            return AP_Mission_ChangeDetector_Copter::ChangeResponseType::NONE;
        } else {
            // not all combinations will result in a changed state but to keep this simple we will call a reset
            debug("check_for_mission_change: 1st is spline, catch change, Reset");
            return AP_Mission_ChangeDetector_Copter::ChangeResponseType::RESET_REQUIRED;
        }
    }

    // we should not reach here unless we have missed something so reset to be safe
    debug("check_for_mission_change: got to end, Reset");
    return AP_Mission_ChangeDetector_Copter::ChangeResponseType::RESET_REQUIRED;
}

