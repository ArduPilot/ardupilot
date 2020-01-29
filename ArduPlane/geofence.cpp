/*
 *  geo-fencing support
 *  Andrew Tridgell, December 2011
 */

#include "Plane.h"

#if GEOFENCE_ENABLED == ENABLED

#define MIN_GEOFENCE_POINTS 5 // index [0] for return point, must be inside polygon
                              // index [1 to n-1] to define a polygon, minimum 3 for a triangle
                              // index [n] (must be same as index 1 to close the polygon)

/*
 *  The state of geo-fencing. This structure is dynamically allocated
 *  the first time it is used. This means we only pay for the pointer
 *  and not the structure on systems where geo-fencing is not being
 *  used.
 *
 *  We store a copy of the boundary in memory as we need to access it
 *  very quickly at runtime
 */
static struct GeofenceState {
    Vector2l *boundary;   // point 0 is the return point
    uint32_t breach_time;
    int32_t guided_lat;
    int32_t guided_lng;
    uint16_t breach_count;
    uint8_t breach_type;
    uint8_t old_switch_position;
    uint8_t num_points;
    bool boundary_uptodate;
    bool fence_triggered;
    bool is_pwm_enabled;          //true if above FENCE_ENABLE_PWM threshold
    bool is_enabled;
    bool floor_enabled;          //typically used for landing
} *geofence_state;

static const StorageAccess fence_storage(StorageManager::StorageFence);

/*
  maximum number of fencepoints
 */
uint8_t Plane::max_fencepoints(void) const
{
    return MIN(255U, fence_storage.size() / sizeof(Vector2l));
}

/*
 *  fence boundaries fetch/store
 */
Vector2l Plane::get_fence_point_with_index(uint8_t i) const
{
    if (i > (uint8_t)g.fence_total || i >= max_fencepoints()) {
        return Vector2l(0,0);
    }

    // read fence point
    return Vector2l(fence_storage.read_uint32(i * sizeof(Vector2l)),
                    fence_storage.read_uint32(i * sizeof(Vector2l) + sizeof(int32_t)));
}

// save a fence point
void Plane::set_fence_point_with_index(const Vector2l &point, unsigned i)
{
    if (i >= (unsigned)g.fence_total.get() || i >= max_fencepoints()) {
        // not allowed
        return;
    }

    fence_storage.write_uint32(i * sizeof(Vector2l), point.x);
    fence_storage.write_uint32(i * sizeof(Vector2l) + sizeof(int32_t), point.y);

    if (geofence_state != nullptr) {
        geofence_state->boundary_uptodate = false;
    }
}

/*
 *  allocate and fill the geofence state structure
 */
void Plane::geofence_load(void)
{
    if (geofence_state == nullptr) {
        uint16_t boundary_size = sizeof(Vector2l) * max_fencepoints();
        if (hal.util->available_memory() < 100 + boundary_size + sizeof(struct GeofenceState)) {
            // too risky to enable as we could run out of stack
            geofence_disable_and_send_error_msg("low on memory");
            return;
        }
        geofence_state = (struct GeofenceState *)calloc(1, sizeof(struct GeofenceState));
        if (geofence_state == nullptr) {
            // not much we can do here except disable it
            geofence_disable_and_send_error_msg("failed to init state memory");
            return;
        }

        geofence_state->boundary = (Vector2l *)calloc(1, boundary_size);
        if (geofence_state->boundary == nullptr) {
            free(geofence_state);
            geofence_state = nullptr;
            geofence_disable_and_send_error_msg("failed to init boundary memory");
            return;
        }
        
        geofence_state->old_switch_position = 254;
    }

    if (g.fence_total <= 0) {
        g.fence_total.set(0);
        return;
    }

    for (uint8_t i = 0; i<g.fence_total; i++) {
        geofence_state->boundary[i] = get_fence_point_with_index(i);
    }
    geofence_state->num_points = g.fence_total;

    if (!Polygon_complete(&geofence_state->boundary[1], geofence_state->num_points-1)) {
        geofence_disable_and_send_error_msg("pt[1] and pt[total-1] must match");
        return;
    }
    if (Polygon_outside(geofence_state->boundary[0], &geofence_state->boundary[1], geofence_state->num_points-1)) {
        geofence_disable_and_send_error_msg("pt[0] must be inside fence");
        return;
    }

    geofence_state->boundary_uptodate = true;
    geofence_state->fence_triggered = false;

    gcs().send_text(MAV_SEVERITY_INFO,"Geofence loaded");
    gcs().send_message(MSG_FENCE_STATUS);
}

/*
 *  Disable geofence and send an error message string
 */
void Plane::geofence_disable_and_send_error_msg(const char *errorMsg)
{
    g.fence_action.set(FENCE_ACTION_NONE);
    gcs().send_text(MAV_SEVERITY_WARNING,"Geofence error, %s", errorMsg);
}

/*
 * return true if a geo-fence has been uploaded and
 * FENCE_ACTION is 1 (not necessarily enabled)
 */
bool Plane::geofence_present(void)
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
void Plane::geofence_update_pwm_enabled_state() 
{
    if (rc_failsafe_active()) {
        // do nothing based on the radio channel value as it may be at bind value
        return;
    }

    bool is_pwm_enabled;
    if (g.fence_channel == 0) {
        is_pwm_enabled = false;
    } else {
        is_pwm_enabled = (RC_Channels::get_radio_in(g.fence_channel-1) > FENCE_ENABLE_PWM);
    }
    if (is_pwm_enabled && geofence_state == nullptr) {
        // we need to load the fence
        geofence_load();
        return;
    }

    if (geofence_state == nullptr) {
        // not loaded
        return;
    }

    if (geofence_state->is_pwm_enabled != is_pwm_enabled) {
        geofence_set_enabled(is_pwm_enabled);
        geofence_state->is_pwm_enabled = is_pwm_enabled;
    }    
}

//return true on success, false on failure
bool Plane::geofence_set_enabled(bool enable) 
{
    if (geofence_state == nullptr && enable) {
        geofence_load();
    }
    if (geofence_state == nullptr) {
        return false;
    }

    geofence_state->is_enabled = enable;
    if (enable == true) {
        //turn the floor back on if it had been off
        geofence_set_floor_enabled(true);
    }
    
    return true;
}

/*
 *  return true if geo-fencing is enabled
 */
bool Plane::geofence_enabled(void)
{
    if (g.fence_action == FENCE_ACTION_NONE) {
        if (geofence_state != nullptr) {
            geofence_state->fence_triggered = false;
        }
        return false;
    }

    geofence_update_pwm_enabled_state();

    if (geofence_state == nullptr) {
        return false;
    }

    if (!geofence_present() ||
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
bool Plane::geofence_set_floor_enabled(bool floor_enable) {
    if (geofence_state == nullptr) {
        return false;
    }
    
    geofence_state->floor_enabled = floor_enable;
    return true;
}

/*
 *  return true if we have breached the geo-fence minimum altiude
 */
bool Plane::geofence_check_minalt(void)
{
    if (g.fence_maxalt <= g.fence_minalt) {
        return false;
    }
    if (g.fence_minalt == 0) {
        return false;
    }
    return (adjusted_altitude_cm() < (g.fence_minalt*100.0f) + home.alt);
}

/*
 *  return true if we have breached the geo-fence maximum altiude
 */
bool Plane::geofence_check_maxalt(void)
{
    if (g.fence_maxalt <= g.fence_minalt) {
        return false;
    }
    if (g.fence_maxalt == 0) {
        return false;
    }
    return (adjusted_altitude_cm() > (g.fence_maxalt*100.0f) + home.alt);
}

/*
  pre-arm check for being inside the fence
 */
bool Plane::geofence_prearm_check(void)
{
    if (!geofence_enabled()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "PreArm: Fence not enabled");
        return false;
    }

    /* allocate the geo-fence state if need be */
    if (geofence_state == nullptr || !geofence_state->boundary_uptodate) {
        geofence_load();
        if (!geofence_enabled()) {
            // may have been disabled by load
            gcs().send_text(MAV_SEVERITY_WARNING, "PreArm: Fence load failed");
            return false;
        }
    }

    if (geofence_state->floor_enabled && g.fence_minalt != 0) {
        // can't use minalt with prearm check
        gcs().send_text(MAV_SEVERITY_WARNING, "PreArm: Fence floor enabled");
        return false;
    }
    if (geofence_check_maxalt()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "PreArm: maxalt breached");
        return false;
    }
    struct Location loc;
    if (!ahrs.get_position(loc)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "PreArm: no position available");
        // must have position
        return false;
    }
    Vector2l location;
    location.x = loc.lat;
    location.y = loc.lng;
    bool outside = Polygon_outside(location, &geofence_state->boundary[1], geofence_state->num_points-1);
    if (outside) {
        gcs().send_text(MAV_SEVERITY_WARNING, "PreArm: outside fence");
        return false;
    }
    return true;
}


/*
 *  check if we have breached the geo-fence
 */
void Plane::geofence_check(bool altitude_check_only)
{
    if (!geofence_enabled()) {
        // switch back to the chosen control mode if still in
        // GUIDED to the return point
        if (geofence_state != nullptr &&
            (g.fence_action == FENCE_ACTION_GUIDED || g.fence_action == FENCE_ACTION_GUIDED_THR_PASS || g.fence_action == FENCE_ACTION_RTL) &&
            (control_mode == &mode_guided || control_mode == &mode_avoidADSB) &&
            geofence_present() &&
            geofence_state->boundary_uptodate &&
            geofence_state->old_switch_position == oldSwitchPosition &&
            guided_WP_loc.lat == geofence_state->guided_lat &&
            guided_WP_loc.lng == geofence_state->guided_lng) {
            geofence_state->old_switch_position = 254;
            set_mode(*previous_mode, ModeReason::GCS_COMMAND);
        }
        return;
    }

    /* allocate the geo-fence state if need be */
    if (geofence_state == nullptr || !geofence_state->boundary_uptodate) {
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
    if (landing.is_expecting_impact()) {
        return;
    }

    if (geofence_state->floor_enabled && geofence_check_minalt()) {
        outside = true;
        breach_type = FENCE_BREACH_MINALT;
    } else if (geofence_check_maxalt()) {
        outside = true;
        breach_type = FENCE_BREACH_MAXALT;
    } else if (!altitude_check_only && ahrs.get_position(loc)) {
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
            gcs().send_text(MAV_SEVERITY_INFO,"Geofence OK");
 #if FENCE_TRIGGERED_PIN > 0
            hal.gpio->pinMode(FENCE_TRIGGERED_PIN, HAL_GPIO_OUTPUT);
            hal.gpio->write(FENCE_TRIGGERED_PIN, 0);
 #endif
            gcs().send_message(MSG_FENCE_STATUS);
        }
        // we're inside, all is good with the world
        return;
    }

    // we are outside the fence
    if (geofence_state->fence_triggered &&
        (control_mode == &mode_guided || control_mode == &mode_avoidADSB || control_mode == &mode_rtl || g.fence_action == FENCE_ACTION_REPORT)) {
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

    gcs().send_text(MAV_SEVERITY_NOTICE,"Geofence triggered");
    gcs().send_message(MSG_FENCE_STATUS);

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
            set_mode(mode_rtl, ModeReason::FENCE_BREACHED);
        } else {
            set_mode(mode_guided, ModeReason::FENCE_BREACHED);
        }
        g.auto_trim.set(saved_auto_trim);

        if (g.fence_ret_rally != 0 || g.fence_action == FENCE_ACTION_RTL) { //return to a rally point
            guided_WP_loc = rally.calc_best_rally_or_home_location(current_loc, get_RTL_altitude());

        } else { //return to fence return point, not a rally point
            guided_WP_loc = {};
            if (g.fence_retalt > 0) {
                //fly to the return point using fence_retalt
                guided_WP_loc.alt = home.alt + 100.0f*g.fence_retalt;
            } else if (g.fence_minalt >= g.fence_maxalt) {
                // invalid min/max, use RTL_altitude
                guided_WP_loc.alt = home.alt + g.RTL_altitude_cm;
            } else {
                // fly to the return point, with an altitude half way between
                // min and max
                guided_WP_loc.alt = home.alt + 100.0f*(g.fence_minalt + g.fence_maxalt)/2;
            }
            guided_WP_loc.lat = geofence_state->boundary[0].x;
            guided_WP_loc.lng = geofence_state->boundary[0].y;
        }
        geofence_state->guided_lat = guided_WP_loc.lat;
        geofence_state->guided_lng = guided_WP_loc.lng;
        geofence_state->old_switch_position = oldSwitchPosition;

        if (g.fence_action != FENCE_ACTION_RTL) { //not needed for RTL mode
            setup_terrain_target_alt(guided_WP_loc);
            set_guided_WP();
        }

        if (g.fence_action == FENCE_ACTION_GUIDED_THR_PASS) {
            guided_throttle_passthru = true;
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
bool Plane::geofence_stickmixing(void) {
    if (geofence_enabled() &&
        geofence_state != nullptr &&
        geofence_state->fence_triggered &&
        (control_mode == &mode_guided || control_mode == &mode_avoidADSB)) {
        // don't mix in user input
        return false;
    }
    // normal mixing rules
    return true;
}

/*
 *
 */
void Plane::geofence_send_status(mavlink_channel_t chan)
{
    if (geofence_enabled() && geofence_state != nullptr) {
        mavlink_msg_fence_status_send(chan,
                                      (int8_t)geofence_state->fence_triggered,
                                      geofence_state->breach_count,
                                      geofence_state->breach_type,
                                      geofence_state->breach_time,
                                      0);
    }
}

/*
  return true if geofence has been breached
 */
bool Plane::geofence_breached(void)
{
    return geofence_state ? geofence_state->fence_triggered : false;
}


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

bool Plane::geofence_set_enabled(bool enable) {
    return false;
}

bool Plane::geofence_set_floor_enabled(bool floor_enable) {
    return false;
}

bool Plane::geofence_breached(void)
{
    return false;
}

#endif // GEOFENCE_ENABLED
