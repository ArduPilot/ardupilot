#include "AP_PolyFence.h"

extern const AP_HAL::HAL& hal;

#define MIN_GEOFENCE_POINTS 5 // 3 to define a minimal polygon (triangle)
                              // + 1 for return point and +1 for last
                              // pt (same as first)

const AP_Param::GroupInfo AP_PolyFence::var_info[] = {
    // @Param: _ACTION
    // @DisplayName: Action on geofence breach
    // @Description: What to do on fence breach. If this is set to 0 then no action is taken, and geofencing is disabled. If this is set to 1 then the plane will enter GUIDED mode, with the target waypoint as the fence return point. If this is set to 2 then the fence breach is reported to the ground station, but no other action is taken. If set to 3 then the plane enters guided mode but the pilot retains manual throttle control. If set to 4 the plane enters RTL mode, with the target waypoint as the closest rally point (or home point if there are no rally points).
    // @Values: 0:None,1:GuidedMode,2:ReportOnly,3:GuidedModeThrPass,4:RTL_Mode
    // @User: Standard
    AP_GROUPINFO("_ACTION",  0, AP_PolyFence, g.fence_action, 0 ),

    // @Param: _TOTAL
    // @DisplayName: Fence Total
    // @Description: Number of geofence points currently loaded
    // @User: Advanced
    AP_GROUPINFO("_TOTAL", 1, AP_PolyFence, g.fence_total, 0),

    // @Param: _CHANNEL
    // @DisplayName: Fence Channel
    // @Description: RC Channel to use to enable geofence. PWM input above 1750 enables the geofence
    // @User: Standard
    AP_GROUPINFO("_CHANNEL", 2, AP_PolyFence, g.fence_channel, 0),

    // @Param: _MINALT
    // @DisplayName: Fence Minimum Altitude
    // @Description: Minimum altitude allowed before geofence triggers
    // @Units: meters
    // @Range: 0 32767
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_MINALT", 3, AP_PolyFence, g.fence_minalt, 0),

    // @Param: _MAXALT
    // @DisplayName: Fence Maximum Altitude
    // @Description: Maximum altitude allowed before geofence triggers
    // @Units: meters
    // @Range: 0 32767
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_MAXALT", 4, AP_PolyFence, g.fence_maxalt, 0),

    // @Param: _RETALT
    // @DisplayName: Fence Return Altitude
    // @Description: Altitude the aircraft will transit to when a fence breach occurs.  If FENCE_RETALT is <= 0 then the midpoint between FENCE_MAXALT and FENCE_MINALT is used, unless FENCE_MAXALT < FENCE_MINALT.  If FENCE_MAXALT < FENCE_MINALT AND FENCE_RETALT is <= 0 then ALT_HOLD_RTL is the altitude used on a fence breach.
    // @Units: meters
    // @Range: 0 32767
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_RETALT", 5, AP_PolyFence, g.fence_retalt, 0),

    // @Param: _AUTOENABLE
    // @DisplayName: Fence automatic enable
    // @Description: When set to 1, geofence automatically enables after an auto takeoff and automatically disables at the beginning of an auto landing.  When on the ground before takeoff the fence is disabled.  When set to 2, the fence autoenables after an auto takeoff, but only disables the fence floor during landing. It is highly recommended to not use this option for line of sight flying and use a fence enable channel instead.
    // @Values: 0:NoAutoEnable,1:AutoEnable,2:AutoEnableDisableFloorOnly
    // @User: Standard
    AP_GROUPINFO("_AUTOENABLE", 6, AP_PolyFence, g.fence_autoenable, 0),

    // @Param: _RET_RALLY
    // @DisplayName: Fence Return to Rally
    // @Description: When set to 1: on fence breach the plane will return to the nearest rally point rather than the fence return point.  If no rally points have been defined the plane will return to the home point.  
    // @Values: 0:FenceReturnPoint,1:NearestRallyPoint
    // @User: Standard
    AP_GROUPINFO("_RET_RALLY", 7, AP_PolyFence, g.fence_ret_rally, 0),

    AP_GROUPEND
};


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
bool AP_PolyFence::present() const
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


bool AP_PolyFence::geofence_enabled(void) { return enabled(); }
bool AP_PolyFence::geofence_present(void) const { return present(); }
void AP_PolyFence::geofence_load(void) { load(); }
bool AP_PolyFence::geofence_set_enabled(bool enable, GeofenceEnableReason r)
{
    return set_enabled(enable, r);
}
bool AP_PolyFence::geofence_set_floor_enabled(bool floor_enable)
{
    return set_floor_enabled(floor_enable);
}
void AP_PolyFence::geofence_check(bool altitude_check_only)
{
    check(altitude_check_only);
}
void AP_PolyFence::geofence_send_status(mavlink_channel_t chan)
{
    return send_status(chan);
}
bool AP_PolyFence::geofence_breached()
{
    return breached();
}
void AP_PolyFence::geofence_update_pwm_enabled_state()
{
    update_pwm_enabled_state();
}

bool AP_PolyFence::should_revert_flight_mode() const
{
    // if there is no fence then we didn't change the flight mode:
    if (geofence_state == NULL) {
        return false;
    }
    // only revert if it looks like we might have flipped the mode to guided:
    if (g.fence_action != FENCE_ACTION_GUIDED &&
        g.fence_action != FENCE_ACTION_GUIDED_THR_PASS &&
        g.fence_action != FENCE_ACTION_RTL) {
        return false;
    }
    // only change from guided mode:
    if (!vehicle_in_mode_guided()) {
        return false;
    }
    // if the geofence is not present we probably didn't set the flight mode:
    if (!geofence_present()) {
        return false;
    }
    // if the boundary is not up-to-date we probably didn't set the flight mode:
    if (!geofence_state->boundary_uptodate) {
        return false;
    }
    // if the mode switch position has been fiddled we shouldn't reset the flight mode:
    if (geofence_state->old_switch_position != oldSwitchPosition()) {
        return false;
    }

    // if a new guided position has been set we shouldn't change modes:
    if (!guided_destinations_match()) {
        return false;
    }

    return true;
}

