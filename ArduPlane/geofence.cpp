// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 *  geo-fencing support
 *  Andrew Tridgell, December 2011
 */

#include "geofence.h"
#include "Plane.h"

#if GEOFENCE_ENABLED == ENABLED

#define MIN_GEOFENCE_POINTS 5 // 3 to define a minimal polygon (triangle)
                              // + 1 for return point and +1 for last
                              // pt (same as first)

static const StorageAccess fence_storage(StorageManager::StorageFence);

/*
  maximum number of fencepoints
 */
uint8_t AP_PolyFence::max_fencepoints()
{
    return MIN(255U, fence_storage.size() / sizeof(Vector2l));
}

/*
 *  fence boundaries fetch/store
 */
Vector2l AP_PolyFence::get_fence_point_with_index(unsigned i)
{
    Vector2l ret;

    if (i > (unsigned)g.fence_total || i >= max_fencepoints()) {
        return Vector2l(0,0);
    }

    // read fence point
    ret.x = fence_storage.read_uint32(i * sizeof(Vector2l));
    ret.y = fence_storage.read_uint32(i * sizeof(Vector2l) + 4);

    return ret;
}

// save a fence point
void AP_PolyFence::set_fence_point_with_index(Vector2l &point, unsigned i)
{
    if (i >= (unsigned)g.fence_total.get() || i >= max_fencepoints()) {
        // not allowed
        return;
    }

    fence_storage.write_uint32(i * sizeof(Vector2l), point.x);
    fence_storage.write_uint32(i * sizeof(Vector2l)+4, point.y);

    if (geofence_state != NULL) {
        geofence_state->boundary_uptodate = false;
    }
}

/*
 *  allocate and fill the geofence state structure
 */
void AP_PolyFence::load()
{
    uint8_t i;

    if (geofence_state == NULL) {
        uint16_t boundary_size = sizeof(Vector2l) * max_fencepoints();
        if (hal.util->available_memory() < 100 + boundary_size + sizeof(struct GeofenceState)) {
            // too risky to enable as we could run out of stack
            goto failed;
        }
        geofence_state = (struct GeofenceState *)calloc(1, sizeof(struct GeofenceState));
        if (geofence_state == NULL) {
            // not much we can do here except disable it
            goto failed;
        }

        geofence_state->boundary = (Vector2l *)calloc(1, boundary_size);
        if (geofence_state->boundary == NULL) {
            free(geofence_state);
            geofence_state = NULL;
            goto failed;
        }
        
        geofence_state->old_switch_position = 254;
    }

    if (g.fence_total <= 0) {
        g.fence_total.set(0);
        return;
    }

    for (i=0; i<g.fence_total; i++) {
        geofence_state->boundary[i] = get_fence_point_with_index(i);
    }
    geofence_state->num_points = i;

    if (!Polygon_complete(&geofence_state->boundary[1], geofence_state->num_points-1)) {
        // first point and last point must be the same
        goto failed;
    }
    if (Polygon_outside(geofence_state->boundary[0], &geofence_state->boundary[1], geofence_state->num_points-1)) {
        // return point needs to be inside the fence
        goto failed;
    }

    geofence_state->boundary_uptodate = true;
    geofence_state->fence_triggered = false;

    gcs_send_text(MAV_SEVERITY_INFO,"Geofence loaded");
    gcs_send_message(MSG_FENCE_STATUS);
    return;

failed:
    g.fence_action.set(FENCE_ACTION_NONE);
    gcs_send_text(MAV_SEVERITY_WARNING,"Geofence setup error");
}

/*
 * return true if a geo-fence has been uploaded and
 * FENCE_ACTION is 1 (not necessarily enabled)
 */
bool AP_PolyFence::present()
{
    //require at least a return point and a triangle
    //to define a geofence area:
    if (g.fence_action == FENCE_ACTION_NONE || g.fence_total < MIN_GEOFENCE_POINTS) {
        return false;
    }
    return true;
}

/*
  check FENCE_CHANNEL and update the is_pwm_enabled state
 */
void AP_PolyFence::update_pwm_enabled_state()
{
    bool is_pwm_enabled;
    if (g.fence_channel == 0) {
        is_pwm_enabled = false;
    } else {
        is_pwm_enabled = (hal.rcin->read(g.fence_channel-1) > FENCE_ENABLE_PWM);
    }
    if (is_pwm_enabled && geofence_state == NULL) {
        // we need to load the fence
        geofence_load();
        return;
    }

    if (geofence_state == NULL) {
        // not loaded
        return;
    }

    geofence_state->previous_is_pwm_enabled = geofence_state->is_pwm_enabled;
    geofence_state->is_pwm_enabled = is_pwm_enabled;

    if (geofence_state->is_pwm_enabled != geofence_state->previous_is_pwm_enabled) {
        geofence_set_enabled(geofence_state->is_pwm_enabled, PWM_TOGGLED);
    }    
}

//return true on success, false on failure
bool AP_PolyFence::set_enabled(bool enable, GeofenceEnableReason r)
{
    if (geofence_state == NULL && enable) {
        geofence_load();
    }
    if (geofence_state == NULL) {
        return false;
    }

    geofence_state->is_enabled = enable;
    if (enable == true) {
        //turn the floor back on if it had been off
        geofence_set_floor_enabled(true);
    }
    geofence_state->enable_reason = r;
    
    return true;
}

/*
 *  return true if geo-fencing is enabled
 */
bool AP_PolyFence::enabled()
{
    geofence_update_pwm_enabled_state();

    if (geofence_state == NULL) {
        return false;
    }

    if (g.fence_action == FENCE_ACTION_NONE ||
        !geofence_present() ||
        (g.fence_action != FENCE_ACTION_REPORT && !geofence_state->is_enabled)) {
        // geo-fencing is disabled
        // re-arm for when the channel trigger is switched on
        geofence_state->fence_triggered = false;
        return false;
    }

    return true;
}

/*
 * Set floor state IF the fence is present.
 * Return false on failure to set floor state.
 */
bool AP_PolyFence::set_floor_enabled(bool floor_enable) {
    if (geofence_state == NULL) {
        return false;
    }
    
    geofence_state->floor_enabled = floor_enable;
    return true;
}

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
 *  check if we have breached the geo-fence
 */
void AP_PolyFence_Plane::check(bool altitude_check_only)
{
    if (!geofence_enabled()) {
        // switch back to the chosen control mode if still in
        // GUIDED to the return point
        if (geofence_state != NULL &&
            (g.fence_action == FENCE_ACTION_GUIDED || g.fence_action == FENCE_ACTION_GUIDED_THR_PASS || g.fence_action == FENCE_ACTION_RTL) &&
            _plane.control_mode == GUIDED &&
            geofence_present() &&
            geofence_state->boundary_uptodate &&
            geofence_state->old_switch_position == _plane.oldSwitchPosition &&
            _plane.guided_WP_loc.lat == geofence_state->guided_lat &&
            _plane.guided_WP_loc.lng == geofence_state->guided_lng) {
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
        int8_t saved_auto_trim = g.auto_trim;
        g.auto_trim.set(0);
        if (g.fence_action == FENCE_ACTION_RTL) {
            _plane.set_mode(RTL);
        } else {
            _plane.set_mode(GUIDED);
        }
        g.auto_trim.set(saved_auto_trim);

        if (g.fence_ret_rally != 0 || g.fence_action == FENCE_ACTION_RTL) { //return to a rally point
            _plane.guided_WP_loc = _plane.rally.calc_best_rally_or_home_location(_plane.current_loc, _plane.get_RTL_altitude());

        } else { //return to fence return point, not a rally point
            if (g.fence_retalt > 0) {
                //fly to the return point using fence_retalt
                _plane.guided_WP_loc.alt = _plane.home.alt + 100.0f*g.fence_retalt;
            } else if (g.fence_minalt >= g.fence_maxalt) {
                // invalid min/max, use RTL_altitude
                _plane.guided_WP_loc.alt = _plane.home.alt + g.RTL_altitude_cm;
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

/*
 *
 */
void AP_PolyFence::send_status(mavlink_channel_t chan)
{
    if (geofence_enabled() && geofence_state != NULL) {
        mavlink_msg_fence_status_send(chan,
                                      (int8_t)geofence_state->fence_triggered,
                                      geofence_state->breach_count,
                                      geofence_state->breach_type,
                                      geofence_state->breach_time);
    }
}

/*
  return true if geofence has been breached
 */
bool AP_PolyFence::breached()
{
    return geofence_state ? geofence_state->fence_triggered : false;
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

bool AP_PolyFence::geofence_enabled(void) { return enabled(); }
bool AP_PolyFence::geofence_present(void) { return present(); }
void AP_PolyFence::geofence_load(void) { load(); }
bool AP_PolyFence::geofence_set_enabled(bool enable, GeofenceEnableReason r)
{
    return set_enabled(enable, r);
}
bool AP_PolyFence::geofence_set_floor_enabled(bool floor_enable)
{
    return set_floor_enabled(floor_enable);
}
bool AP_PolyFence_Plane::geofence_check_minalt()
{
    return check_minalt();
}
bool AP_PolyFence_Plane::geofence_check_maxalt()
{
    return check_maxalt();
}
void AP_PolyFence::geofence_check(bool altitude_check_only)
{
    check(altitude_check_only);
}
bool AP_PolyFence_Plane::geofence_stickmixing()
{
    return stickmixing();
}
void AP_PolyFence::geofence_send_status(mavlink_channel_t chan)
{
    return send_status(chan);
}
bool AP_PolyFence::geofence_breached()
{
    return breached();
}

void AP_PolyFence_Plane::gcs_send_text(MAV_SEVERITY severity, const char *str)
{
    GCS_MAVLINK::send_statustext(severity, 0xFF, str);
}
void AP_PolyFence_Plane::gcs_send_message(enum ap_message id)
{
    _plane.gcs_send_message(id);
}
void AP_PolyFence::geofence_update_pwm_enabled_state()
{
    update_pwm_enabled_state();
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
