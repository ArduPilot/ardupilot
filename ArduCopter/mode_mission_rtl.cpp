#include "Copter.h"

#if MODE_RTL_ENABLED == ENABLED && MODE_AUTO_ENABLED == ENABLED

/*
 * Mission RTL Mode
 * Perform Do_Land_Start mission to get to landing
 * Enabled with MISSION_RTL param
 */

bool Mission_RTL::init(bool ignore_checks)
{
    // back up Auto mode state
    copter.mode_auto.mission.backup_mission(backup);

    // Go straight to landing sequence in auto submode
    if (((Type)g2.mission_RTL_type.get() == Type::CLOSEST_LANDING_SEQUENCE) && copter.mode_auto.mission.jump_to_landing_sequence()) {
        submode = SubMode::AUTO;
        copter.mode_auto.mission.set_force_resume(true);
        return copter.mode_auto.init(ignore_checks);
    }

    // Go home first in RLT submode, setup auto mode ready for switch at home
    if (((Type)g2.mission_RTL_type.get() == Type::HOME_THEN_LANDING_SEQUENCE)) {
        submode = SubMode::RTL;
        return copter.mode_rtl.init(ignore_checks);
    }

    copter.set_mode(Mode::Number::RTL, ModeReason::MISSION_RTL_FAILED);

    if ((Type)g2.mission_RTL_type.get() == Type::NONE) {
        gcs().send_text(MAV_SEVERITY_WARNING, "MRTL not enabled");
    } else {
        gcs().send_text(MAV_SEVERITY_WARNING, "MRTL mission not configured correclty");
    }

    return false;
}

void Mission_RTL::run()
{
    if (submode == SubMode::AUTO) {
        copter.mode_auto.run();
        return;
    }

    copter.mode_rtl.run();

    // check for reaching home and switch to auto submode for landing sequence
    if (copter.mode_rtl._state >= ModeRTL::SubMode::LOITER_AT_HOME) {
        if (!copter.mode_auto.mission.jump_to_landing_sequence() || !copter.mode_auto.init(false)) {
            copter.set_mode(Mode::Number::RTL, ModeReason::MISSION_RTL_FAILED);
            gcs().send_text(MAV_SEVERITY_WARNING, "MRTL mission not configured correclty");
            return;
        }

        copter.mode_rtl.exit();
        submode = SubMode::AUTO;
    }
}

void Mission_RTL::exit()
{
    if (submode == SubMode::AUTO) {
        copter.mode_auto.exit();
    } else {
        copter.mode_rtl.exit();
    }

    // restore mission state back to where it was when we entered
    copter.mode_auto.mission.restore_mission(backup);
}

bool Mission_RTL::requires_GPS() const
{
    if (submode == SubMode::AUTO) {
        return copter.mode_auto.requires_GPS();
    }
    return copter.mode_rtl.requires_GPS();
}

bool Mission_RTL::has_manual_throttle() const
{
    if (submode == SubMode::AUTO) {
        return copter.mode_auto.has_manual_throttle();
    }
    return copter.mode_rtl.has_manual_throttle();
}

bool Mission_RTL::allows_arming(AP_Arming::Method method) const
{
    return copter.mode_rtl.allows_arming(method);
}

bool Mission_RTL::is_autopilot() const
{
    if (submode == SubMode::AUTO) {
        return copter.mode_auto.is_autopilot();
    }
    return copter.mode_rtl.is_autopilot();
}

bool Mission_RTL::in_guided_mode() const
{
    if (submode == SubMode::AUTO) {
        return copter.mode_auto.in_guided_mode();
    }
    return copter.mode_rtl.in_guided_mode();
}

bool Mission_RTL::requires_terrain_failsafe() const
{
    if (submode == SubMode::AUTO) {
        return copter.mode_auto.requires_terrain_failsafe();
    }
    return copter.mode_rtl.requires_terrain_failsafe();
}

bool Mission_RTL::has_user_takeoff(bool must_navigate) const
{
    if (submode == SubMode::AUTO) {
        return copter.mode_auto.has_user_takeoff(must_navigate);
    }
    return copter.mode_rtl.has_user_takeoff(must_navigate);
}

uint32_t Mission_RTL::wp_distance() const
{
    if (submode == SubMode::AUTO) {
        return copter.mode_auto.wp_distance();
    }
    return copter.mode_rtl.wp_distance();
}

int32_t Mission_RTL::wp_bearing() const
{
    if (submode == SubMode::AUTO) {
        return copter.mode_auto.wp_bearing();
    }
    return copter.mode_rtl.wp_bearing();
}

float Mission_RTL::crosstrack_error() const
{
    if (submode == SubMode::AUTO) {
        return copter.mode_auto.crosstrack_error();
    }
    return copter.mode_rtl.crosstrack_error();
}

bool Mission_RTL::get_wp(Location &loc) const
{
    if (submode == SubMode::AUTO) {
        return copter.mode_auto.get_wp(loc);
    }
    return copter.mode_rtl.get_wp(loc);
}

bool Mission_RTL::is_taking_off() const
{
    if (submode == SubMode::AUTO) {
        return copter.mode_auto.is_taking_off();
    }
    return copter.mode_rtl.is_taking_off();
}

bool Mission_RTL::is_landing() const
{
    if (submode == SubMode::AUTO) {
        return copter.mode_auto.is_landing();
    }
    return copter.mode_rtl.is_landing();
}

#endif
