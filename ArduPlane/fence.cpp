#include "Plane.h"

// Code to integrate AC_Fence library with main ArduPlane code

#if AP_FENCE_ENABLED

// fence_check - ask fence library to check for breaches and initiate the response
void Plane::fence_check()
{
    const uint8_t orig_breaches = fence.get_breaches();

    // check for new breaches; new_breaches is bitmask of fence types breached
    const uint8_t new_breaches = fence.check();

    if (!fence.enabled()) {
        // Switch back to the chosen control mode if still in
        // GUIDED to the return point
        switch(fence.get_action()) {
            case AC_FENCE_ACTION_GUIDED:
            case AC_FENCE_ACTION_GUIDED_THROTTLE_PASS:
            case AC_FENCE_ACTION_RTL_AND_LAND:
                if (plane.control_mode_reason == ModeReason::FENCE_BREACHED &&
                    control_mode->is_guided_mode()) {
                    set_mode(*previous_mode, ModeReason::FENCE_RETURN_PREVIOUS_MODE);
                }
                break;
            default:
                // No returning to a previous mode, unless our action allows it
                break;
        }
        return;
    }

    // we still don't do anything when disarmed, but we do check for fence breaches.
    // fence pre-arm check actually checks if any fence has been breached
    // that's not ever going to be true if we don't call check on AP_Fence while disarmed
    if (!arming.is_armed()) {
        return;
    }

    // Never trigger a fence breach in the final stage of landing
    if (landing.is_expecting_impact()) {
        return;
    }

    if (in_fence_recovery()) {
        // we have already triggered, don't trigger again until the
        // user disables/re-enables using the fence channel switch
        return;
    }

    if (new_breaches) {
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Fence Breached");

        // if the user wants some kind of response and motors are armed
        const uint8_t fence_act = fence.get_action();
        switch (fence_act) {
        case AC_FENCE_ACTION_REPORT_ONLY:
            break;
        case AC_FENCE_ACTION_GUIDED:
        case AC_FENCE_ACTION_GUIDED_THROTTLE_PASS:
        case AC_FENCE_ACTION_RTL_AND_LAND:
            if (fence_act == AC_FENCE_ACTION_RTL_AND_LAND) {
                if (control_mode == &mode_auto &&
                    mission.get_in_landing_sequence_flag() &&
                    (g.rtl_autoland == RtlAutoland::RTL_THEN_DO_LAND_START ||
                     g.rtl_autoland == RtlAutoland::RTL_IMMEDIATE_DO_LAND_START)) {
                    // already landing
                    return;
                }
                set_mode(mode_rtl, ModeReason::FENCE_BREACHED);
            } else {
                set_mode(mode_guided, ModeReason::FENCE_BREACHED);
            }

            Location loc;
            if (fence.get_return_rally() != 0 || fence_act == AC_FENCE_ACTION_RTL_AND_LAND) {
                loc = calc_best_rally_or_home_location(current_loc, get_RTL_altitude_cm());
            } else {
                //return to fence return point, not a rally point
                if (fence.get_return_altitude() > 0) {
                    // fly to the return point using _retalt
                    loc.alt = home.alt + 100.0f * fence.get_return_altitude();
                } else if (fence.get_safe_alt_min() >= fence.get_safe_alt_max()) {
                    // invalid min/max, use RTL_altitude
                    loc.alt = home.alt + g.RTL_altitude_cm;
                } else {
                    // fly to the return point, with an altitude half way between
                    // min and max
                    loc.alt = home.alt + 100.0f * (fence.get_safe_alt_min() + fence.get_safe_alt_max()) / 2;
                }

                Vector2l return_point;
                if(fence.polyfence().get_return_point(return_point)) {
                    loc.lat = return_point[0];
                    loc.lng = return_point[1];
                } else {
                    // When no fence return point is found (ie. no inclusion fence uploaded, but exclusion is)
                    // we fail to obtain a valid fence return point. In this case, home is considered a safe
                    // return point.
                    loc.lat = home.lat;
                    loc.lng = home.lng;
                }
            }

            if (fence.get_action() != AC_FENCE_ACTION_RTL_AND_LAND) {
                setup_terrain_target_alt(loc);
                set_guided_WP(loc);
            }

            if (fence.get_action() == AC_FENCE_ACTION_GUIDED_THROTTLE_PASS) {
                guided_throttle_passthru = true;
            }
            break;
        }

        AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_FENCE, LogErrorCode(new_breaches));
    } else if (orig_breaches) {
        // record clearing of breach
        AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_FENCE, LogErrorCode::ERROR_RESOLVED);
    }
}

bool Plane::fence_stickmixing(void) const
{
    if (fence.enabled() &&
        fence.get_breaches() &&
        in_fence_recovery())
    {
        // don't mix in user input
        return false;
    }
    // normal mixing rules
    return true;
}

bool Plane::in_fence_recovery() const
{
    const bool current_mode_breach = plane.control_mode_reason == ModeReason::FENCE_BREACHED;
    const bool previous_mode_breach = plane.previous_mode_reason ==  ModeReason::FENCE_BREACHED;
    const bool previous_mode_complete = (plane.control_mode_reason == ModeReason::RTL_COMPLETE_SWITCHING_TO_VTOL_LAND_RTL) ||
                                        (plane.control_mode_reason == ModeReason::RTL_COMPLETE_SWITCHING_TO_FIXEDWING_AUTOLAND) ||
                                        (plane.control_mode_reason == ModeReason::QRTL_INSTEAD_OF_RTL) ||
                                        (plane.control_mode_reason == ModeReason::QLAND_INSTEAD_OF_RTL);

    return current_mode_breach || (previous_mode_breach && previous_mode_complete);
}

#endif
