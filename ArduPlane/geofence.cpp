// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 *  geo-fencing support
 *  Andrew Tridgell, December 2011
 */

#include "geofence.h"
#include "Plane.h"

#if GEOFENCE_ENABLED == ENABLED

const AP_Param::GroupInfo AP_PolyFence_Plane::var_info_plane[] = {
    // variables from parent fence
    AP_NESTEDGROUPINFO(AP_PolyFence, 0),

    // insert plane-specific fence variables here

    AP_GROUPEND
};

/*
 *  return true if we have breached the geo-fence minimum altiude
 */
bool AP_PolyFence_Plane::check_minalt()
{
    if (g.fence_maxalt <= g.fence_minalt) {
        return false;
    }
    if (g.fence_minalt == 0) {
        return false;
    }
    return (_plane.adjusted_altitude_cm() < (g.fence_minalt*100.0f) + _plane.home.alt);
}

/*
 *  return true if we have breached the geo-fence maximum altiude
 */
bool AP_PolyFence_Plane::check_maxalt()
{
    if (g.fence_maxalt <= g.fence_minalt) {
        return false;
    }
    if (g.fence_maxalt == 0) {
        return false;
    }
    return (_plane.adjusted_altitude_cm() > (g.fence_maxalt*100.0f) + _plane.home.alt);
}


/*
 * return true if the Vehicle's current guided destination is the same
 * as the guided-mode destination we scribbled down when handling a
 * fence breach
 */
bool AP_PolyFence_Plane::guided_destinations_match() const
{
    return (_plane.guided_WP_loc.lat == geofence_state->guided_lat &&
            _plane.guided_WP_loc.lng == geofence_state->guided_lng);
}

bool AP_PolyFence_Plane::vehicle_in_mode_guided() const
{
    return (_plane.control_mode == GUIDED);
}

bool AP_PolyFence_Plane::vehicle_in_mode_rtl() const
{
    return (_plane.control_mode == RTL);
}

/*
 * returns the position the mode switch was in before the current one
 */
uint8_t AP_PolyFence_Plane::oldSwitchPosition() const
{
    return _plane.oldSwitchPosition;
}

/*
 *  check if we have breached the geo-fence
 */
void AP_PolyFence_Plane::check(bool altitude_check_only)
{
    if (!geofence_enabled()) {
        // switch back to the chosen control mode if still in
        // GUIDED to the return point
        if (should_revert_flight_mode()) {
            geofence_state->old_switch_position = 254;
            _plane.set_mode(_plane.get_previous_mode());
        }
        return;
    }

    /* allocate the geo-fence state if need be */
    if (geofence_state == NULL || !geofence_state->boundary_uptodate) {
        geofence_load();
        if (!geofence_enabled()) {
            // may have been disabled by load
            return;
        }
    }

    bool outside = false;
    uint8_t breach_type = FENCE_BREACH_NONE;
    struct Location loc;

    // Never trigger a fence breach in the final stage of landing
    if (_plane.flight_stage == AP_SpdHgtControl::FLIGHT_LAND_FINAL) {
        return;
    }

    if (geofence_state->floor_enabled && geofence_check_minalt()) {
        outside = true;
        breach_type = FENCE_BREACH_MINALT;
    } else if (geofence_check_maxalt()) {
        outside = true;
        breach_type = FENCE_BREACH_MAXALT;
    } else if (!altitude_check_only && _plane.ahrs.get_position(loc)) {
        Vector2l location;
        location.x = loc.lat;
        location.y = loc.lng;
        outside = Polygon_outside(location, &geofence_state->boundary[1], geofence_state->num_points-1);
        if (outside) {
            breach_type = FENCE_BREACH_BOUNDARY;
        }
    }

    if (!outside) {
        if (geofence_state->fence_triggered && !altitude_check_only) {
            // we have moved back inside the fence
            geofence_state->fence_triggered = false;
            gcs_send_text(MAV_SEVERITY_INFO,"Geofence OK");
 #if FENCE_TRIGGERED_PIN > 0
            hal.gpio->pinMode(FENCE_TRIGGERED_PIN, HAL_GPIO_OUTPUT);
            hal.gpio->write(FENCE_TRIGGERED_PIN, 0);
 #endif
            gcs_send_message(MSG_FENCE_STATUS);
        }
        // we're inside, all is good with the world
        return;
    }

    // we are outside the fence
    if (geofence_state->fence_triggered &&
        (_plane.control_mode == GUIDED || _plane.control_mode == RTL || g.fence_action == FENCE_ACTION_REPORT)) {
        // we have already triggered, don't trigger again until the
        // user disables/re-enables using the fence channel switch
        return;
    }

    // we are outside, and have not previously triggered.
    geofence_state->fence_triggered = true;
    geofence_state->breach_count++;
    geofence_state->breach_time = millis();
    geofence_state->breach_type = breach_type;

 #if FENCE_TRIGGERED_PIN > 0
    hal.gpio->pinMode(FENCE_TRIGGERED_PIN, HAL_GPIO_OUTPUT);
    hal.gpio->write(FENCE_TRIGGERED_PIN, 1);
 #endif

    gcs_send_text(MAV_SEVERITY_NOTICE,"Geofence triggered");
    gcs_send_message(MSG_FENCE_STATUS);

    // see what action the user wants
    switch (g.fence_action) {
    case FENCE_ACTION_REPORT:
        break;

    case FENCE_ACTION_GUIDED:
    case FENCE_ACTION_GUIDED_THR_PASS:
    case FENCE_ACTION_RTL:
        // make sure we don't auto trim the surfaces on this mode change
        int8_t saved_auto_trim = _plane.g.auto_trim;
        _plane.g.auto_trim.set(0);
        if (g.fence_action == FENCE_ACTION_RTL) {
            _plane.set_mode(RTL);
        } else {
            _plane.set_mode(GUIDED);
        }
        _plane.g.auto_trim.set(saved_auto_trim);

        if (g.fence_ret_rally != 0 || g.fence_action == FENCE_ACTION_RTL) { //return to a rally point
            _plane.guided_WP_loc = _plane.rally.calc_best_rally_or_home_location(_plane.current_loc, _plane.get_RTL_altitude());

        } else { //return to fence return point, not a rally point
            if (g.fence_retalt > 0) {
                //fly to the return point using fence_retalt
                _plane.guided_WP_loc.alt = _plane.home.alt + 100.0f*g.fence_retalt;
            } else if (g.fence_minalt >= g.fence_maxalt) {
                // invalid min/max, use RTL_altitude
                _plane.guided_WP_loc.alt = _plane.home.alt + _plane.g.RTL_altitude_cm;
            } else {
                // fly to the return point, with an altitude half way between
                // min and max
                _plane.guided_WP_loc.alt = _plane.home.alt + 100.0f*(g.fence_minalt + g.fence_maxalt)/2;
            }
            _plane.guided_WP_loc.options = 0;
            _plane.guided_WP_loc.lat = geofence_state->boundary[0].x;
            _plane.guided_WP_loc.lng = geofence_state->boundary[0].y;
        }
        geofence_state->guided_lat = _plane.guided_WP_loc.lat;
        geofence_state->guided_lng = _plane.guided_WP_loc.lng;
        geofence_state->old_switch_position = _plane.oldSwitchPosition;

        if (g.fence_action != FENCE_ACTION_RTL) { //not needed for RTL mode
            _plane.setup_terrain_target_alt(_plane.guided_WP_loc);
            _plane.set_guided_WP();
        }

        if (g.fence_action == FENCE_ACTION_GUIDED_THR_PASS) {
            _plane.guided_throttle_passthru = true;
        }
        break;
    }

}

/*
 *  return true if geofencing allows stick mixing. When we have
 *  triggered failsafe and are in GUIDED mode then stick mixing is
 *  disabled. Otherwise the aircraft may not be able to recover from
 *  a breach of the geo-fence
 */
bool AP_PolyFence_Plane::stickmixing() {
    if (geofence_enabled() &&
        geofence_state != NULL &&
        geofence_state->fence_triggered &&
        _plane.control_mode == GUIDED) {
        // don't mix in user input
        return false;
    }
    // normal mixing rules
    return true;
}

/* start function wrappers to avoid code churn */
bool Plane::geofence_enabled(void) { return geofence.geofence_enabled(); }
bool Plane::geofence_breached(void) { return geofence.geofence_breached(); }
bool Plane::geofence_present(void) { return geofence.geofence_present(); }
bool Plane::geofence_set_floor_enabled(bool floor_enabled)
{
    return geofence.geofence_set_floor_enabled(floor_enabled);
}
void Plane::geofence_check(bool altitude_check_only)
{
    return geofence.geofence_check(altitude_check_only);
}
bool Plane::geofence_set_enabled(bool enable, GeofenceEnableReason r)
{
    return geofence.geofence_set_enabled(enable, r);
}
bool Plane::geofence_stickmixing(void)
{
    return geofence.geofence_stickmixing();
}
void Plane::geofence_send_status(mavlink_channel_t chan)
{
    return geofence.geofence_send_status(chan);
}
Vector2l Plane::get_fence_point_with_index(unsigned i)
{
    return geofence.get_fence_point_with_index(i);
}
void Plane::set_fence_point_with_index(Vector2l &point, unsigned i)
{
    return geofence.set_fence_point_with_index(point,i);
}

bool AP_PolyFence_Plane::geofence_check_minalt()
{
    return check_minalt();
}
bool AP_PolyFence_Plane::geofence_check_maxalt()
{
    return check_maxalt();
}
bool AP_PolyFence_Plane::geofence_stickmixing()
{
    return stickmixing();
}
void AP_PolyFence_Plane::gcs_send_text(MAV_SEVERITY severity, const char *str)
{
    GCS_MAVLINK::send_statustext(severity, 0xFF, str);
}
void AP_PolyFence_Plane::gcs_send_message(enum ap_message id)
{
    _plane.gcs_send_message(id);
}


/* end function wrappers to avoid code churn */

#else // GEOFENCE_ENABLED

void Plane::geofence_check(bool altitude_check_only) {
}
bool Plane::geofence_stickmixing(void) {
    return true;
}
bool Plane::geofence_enabled(void) {
    return false;
}

bool Plane::geofence_present(void) {
    return false;
}

bool Plane::geofence_set_enabled(bool enable, GeofenceEnableReason r) {
    return false;
}

bool Plane::geofence_set_floor_enabled(bool floor_enable) {
    return false;
}

bool Plane::geofence_breached(void) {
    return false;
}

#endif // GEOFENCE_ENABLED
