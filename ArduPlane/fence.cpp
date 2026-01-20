#include "Plane.h"

// Code to integrate AC_Fence library with main ArduPlane code

#if AP_FENCE_ENABLED

// async fence checking io callback at 1Khz
void Plane::fence_checks_async()
{
    const uint32_t now = AP_HAL::millis();

    if (!AP_HAL::timeout_expired(fence_breaches.last_check_ms, now, 333U)) { // 3Hz update rate
        return;
    }

    if (fence_breaches.have_updates) {
        return; // wait for the main loop to pick up the new breaches before checking again
    }

    fence_breaches.last_check_ms = now;
    orig_breaches = fence.get_breaches();
    const bool armed = arming.is_armed();

    uint16_t mission_id = plane.mission.get_current_nav_cmd().id;
    bool landing_or_landed = plane.flight_stage == AP_FixedWing::FlightStage::LAND
                         || !armed
#if HAL_QUADPLANE_ENABLED
                         || control_mode->mode_number() == Mode::Number::QLAND
                         || quadplane.in_vtol_land_descent()
#endif
                         || (plane.is_land_command(mission_id) && plane.mission.state() == AP_Mission::MISSION_RUNNING);

    // check for new breaches; new_breaches is bitmask of fence types breached
    fence_breaches.new_breaches = fence.check(landing_or_landed);
    fence_breaches.have_updates = true; // new breach status latched so main loop will now pick it up
}

// fence_check - ask fence library to check for breaches and initiate the response
void Plane::fence_check()
{
    if (!fence_breaches.have_updates) {
        return;
    }
    /*
      if we are either disarmed or we are currently not in breach and
      we are not flying then clear the state associated with the
      previous mode breach handling. This allows the fence state
      machine to reset at the end of a fence breach action such as an
      RTL and autoland
     */
    const bool armed = arming.is_armed();

    if (plane.previous_mode_reason == ModeReason::FENCE_BREACHED) {
        if (!armed || ((fence_breaches.new_breaches == 0 && orig_breaches == 0) && !plane.is_flying())) {
            plane.previous_mode_reason = ModeReason::FENCE_REENABLE;
        }
    }

    if (!fence.enabled()) {
        // Switch back to the chosen control mode if still in
        // GUIDED to the return point
        switch(fence.get_action()) {
            case AC_Fence::Action::GUIDED:
            case AC_Fence::Action::GUIDED_THROTTLE_PASS:
            case AC_Fence::Action::RTL_AND_LAND:
            case AC_Fence::Action::AUTOLAND_OR_RTL:
                if (plane.control_mode_reason == ModeReason::FENCE_BREACHED &&
                    control_mode->is_guided_mode()) {
                    set_mode(*previous_mode, ModeReason::FENCE_RETURN_PREVIOUS_MODE);
                }
                break;
            default:
                // No returning to a previous mode, unless our action allows it
                break;
        }
        /*
          clear mode reasons if they are FENCE_BREACHED to allow AUX
          switch fence disable/enable to re-enable the fence after a breach
         */
        if (plane.previous_mode_reason == ModeReason::FENCE_BREACHED) {
            plane.previous_mode_reason = ModeReason::FENCE_REENABLE;
        }
        if (plane.control_mode_reason == ModeReason::FENCE_BREACHED) {
            plane.control_mode_reason = ModeReason::FENCE_REENABLE;
        }
        goto fence_check_complete;
    }

    // we still don't do anything when disarmed, but we do check for fence breaches.
    // fence pre-arm check actually checks if any fence has been breached
    // that's not ever going to be true if we don't call check on AP_Fence while disarmed
    if (!armed) {
        goto fence_check_complete;
    }

    // Never trigger a fence breach in the final stage of landing
    if (landing.is_expecting_impact()) {
        goto fence_check_complete;
    }

    if (in_fence_recovery()) {
        // we have already triggered, don't trigger again until the
        // user disables/re-enables using the fence channel switch
        goto fence_check_complete;
    }

    if (fence_breaches.new_breaches) {
        fence.print_fence_message("breached", fence_breaches.new_breaches);

        // if the user wants some kind of response and motors are armed
        const auto fence_act = fence.get_action();
        switch (fence_act) {
        case AC_Fence::Action::REPORT_ONLY:
            break;

        case AC_Fence::Action::ALWAYS_LAND:
        case AC_Fence::Action::SMART_RTL:
        case AC_Fence::Action::SMART_RTL_OR_LAND:
        case AC_Fence::Action::BRAKE:
            // invalid enumeration value for Plane
            break;

        case AC_Fence::Action::AUTOLAND_OR_RTL:
        case AC_Fence::Action::RTL_AND_LAND:
            if (control_mode == &mode_auto &&
                mission.get_in_landing_sequence_flag() &&
                (g.rtl_autoland == RtlAutoland::RTL_THEN_DO_LAND_START ||
                    g.rtl_autoland == RtlAutoland::RTL_IMMEDIATE_DO_LAND_START)) {
                // already landing
                goto fence_check_complete;
            }
#if MODE_AUTOLAND_ENABLED
            if (control_mode == &mode_autoland) {
                // Already landing
                return;
            }
            if ((fence_act == AC_Fence::Action::AUTOLAND_OR_RTL) && set_mode(mode_autoland, ModeReason::FENCE_BREACHED)) {
                break;
            }
#endif
            set_mode(mode_rtl, ModeReason::FENCE_BREACHED);
            break;

        case AC_Fence::Action::GUIDED:
        case AC_Fence::Action::GUIDED_THROTTLE_PASS:
            set_mode(mode_guided, ModeReason::FENCE_BREACHED);

            Location loc;
            if (fence.get_return_rally() != 0) {
                loc = calc_best_rally_or_home_location(current_loc, get_RTL_altitude_cm());
            } else {
                //return to fence return point, not a rally point
                if (fence.get_return_altitude() > 0) {
                    // fly to the return point using _retalt
                    loc.set_alt_cm(home.alt + 100.0 * fence.get_return_altitude(),
                                   Location::AltFrame::ABSOLUTE);
                } else if (fence.get_safe_alt_min_m() >= fence.get_safe_alt_max_m()) {
                    // invalid min/max, use RTL_altitude
                    loc.set_alt_cm(home.alt + g.RTL_altitude*100,
                                   Location::AltFrame::ABSOLUTE);
                } else {
                    // fly to the return point, with an altitude half way between
                    // min and max
                    loc.set_alt_cm(home.alt + 100.0f * (fence.get_safe_alt_min_m() + fence.get_safe_alt_max_m()) / 2,
                                   Location::AltFrame::ABSOLUTE);
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

            setup_terrain_target_alt(loc);
            set_guided_WP(loc);

            if (fence.get_action() == AC_Fence::Action::GUIDED_THROTTLE_PASS) {
                guided_throttle_passthru = true;
            }
            break;
        }

        LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_FENCE, LogErrorCode(fence_breaches.new_breaches));
    } else if (orig_breaches && fence.get_breaches() == 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Fence breach cleared");
        // record clearing of breach
        LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_FENCE, LogErrorCode::ERROR_RESOLVED);
    }

fence_check_complete:
    fence_breaches.have_updates = false;
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
    if (control_mode == &mode_auto && !mission.get_in_landing_sequence_flag()) {
        // the user may have changed target WP to be outside the
        // landing sequence
        return false;
    }
    const bool current_mode_breach = plane.control_mode_reason == ModeReason::FENCE_BREACHED;
    const bool previous_mode_breach = plane.previous_mode_reason ==  ModeReason::FENCE_BREACHED;
    const bool previous_mode_complete = (plane.control_mode_reason == ModeReason::RTL_COMPLETE_SWITCHING_TO_VTOL_LAND_RTL) ||
                                        (plane.control_mode_reason == ModeReason::RTL_COMPLETE_SWITCHING_TO_FIXEDWING_AUTOLAND) ||
                                        (plane.control_mode_reason == ModeReason::QRTL_INSTEAD_OF_RTL) ||
                                        (plane.control_mode_reason == ModeReason::QLAND_INSTEAD_OF_RTL);

    return current_mode_breach || (previous_mode_breach && previous_mode_complete);
}

#endif
