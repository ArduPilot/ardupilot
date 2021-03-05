#include "Plane.h"

// Code to integrate AC_Fence library with main ArduPlane code

#if AC_FENCE == ENABLED

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

    if (orig_breaches &&
        (control_mode->is_guided_mode()
        || control_mode == &mode_rtl || fence.get_action() == AC_FENCE_ACTION_REPORT_ONLY)) {
        // we have already triggered, don't trigger again until the
        // user disables/re-enables using the fence channel switch
        return;
    }

    if (new_breaches || orig_breaches) {
        // if the user wants some kind of response and motors are armed
        const uint8_t fence_act = fence.get_action();
        switch (fence_act) {
        case AC_FENCE_ACTION_REPORT_ONLY:
            break;
        case AC_FENCE_ACTION_GUIDED:
        case AC_FENCE_ACTION_GUIDED_THROTTLE_PASS:
        case AC_FENCE_ACTION_RTL_AND_LAND:
            // make sure we don't auto trim the surfaces on this mode change
            int8_t saved_auto_trim = g.auto_trim;
            g.auto_trim.set(0);
            if (fence_act == AC_FENCE_ACTION_RTL_AND_LAND) {
                set_mode(mode_rtl, ModeReason::FENCE_BREACHED);
            } else {
                set_mode(mode_guided, ModeReason::FENCE_BREACHED);
            }
            g.auto_trim.set(saved_auto_trim);

            if (fence.get_return_rally() != 0 || fence_act == AC_FENCE_ACTION_RTL_AND_LAND) {
                guided_WP_loc = rally.calc_best_rally_or_home_location(current_loc, get_RTL_altitude());
            } else {
                //return to fence return point, not a rally point
                guided_WP_loc = {};
                if (fence.get_return_altitude() > 0) {
                    // fly to the return point using _retalt
                    guided_WP_loc.alt = home.alt + 100.0f * fence.get_return_altitude();
                } else if (fence.get_safe_alt_min() >= fence.get_safe_alt_max()) {
                    // invalid min/max, use RTL_altitude
                    guided_WP_loc.alt = home.alt + g.RTL_altitude_cm;
                } else {
                    // fly to the return point, with an altitude half way between
                    // min and max
                    guided_WP_loc.alt = home.alt + 100.0f * (fence.get_safe_alt_min() + fence.get_safe_alt_max()) / 2;
                }

                Vector2l return_point;
                if(fence.polyfence().get_return_point(return_point)) {
                    guided_WP_loc.lat = return_point[0];
                    guided_WP_loc.lng = return_point[1];
                } else {
                    // should. not. happen.
                    guided_WP_loc.lat = current_loc.lat;
                    guided_WP_loc.lng = current_loc.lng;
                }
            }

            if (fence.get_action() != AC_FENCE_ACTION_RTL_AND_LAND) {
                setup_terrain_target_alt(guided_WP_loc);
                set_guided_WP();
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
        control_mode->is_guided_mode())
    {
        // don't mix in user input
        return false;
    }
    // normal mixing rules
    return true;
}

#endif