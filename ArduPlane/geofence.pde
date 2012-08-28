// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 *  geo-fencing support
 *  Andrew Tridgell, December 2011
 */

#if GEOFENCE_ENABLED == ENABLED

/*
 *  The state of geo-fencing. This structure is dynamically allocated
 *  the first time it is used. This means we only pay for the pointer
 *  and not the structure on systems where geo-fencing is not being
 *  used.
 *
 *  We store a copy of the boundary in memory as we need to access it
 *  very quickly at runtime
 */
static struct geofence_state {
    uint8_t num_points;
    bool boundary_uptodate;
    bool fence_triggered;
    uint16_t breach_count;
    uint8_t breach_type;
    uint32_t breach_time;
    byte old_switch_position;
    /* point 0 is the return point */
    Vector2l boundary[MAX_FENCEPOINTS];
} *geofence_state;


/*
 *  fence boundaries fetch/store
 */
static Vector2l get_fence_point_with_index(unsigned i)
{
    intptr_t mem;
    Vector2l ret;

    if (i > (unsigned)g.fence_total) {
        return Vector2l(0,0);
    }

    // read fence point
    mem = FENCE_START_BYTE + (i * FENCE_WP_SIZE);
    ret.x = eeprom_read_dword((uint32_t *)mem);
    mem += sizeof(uint32_t);
    ret.y = eeprom_read_dword((uint32_t *)mem);

    return ret;
}

// save a fence point
static void set_fence_point_with_index(Vector2l &point, unsigned i)
{
    intptr_t mem;

    if (i >= (unsigned)g.fence_total.get()) {
        // not allowed
        return;
    }

    mem = FENCE_START_BYTE + (i * FENCE_WP_SIZE);

    eeprom_write_dword((uint32_t *)mem, point.x);
    mem += sizeof(uint32_t);
    eeprom_write_dword((uint32_t *)mem, point.y);

    if (geofence_state != NULL) {
        geofence_state->boundary_uptodate = false;
    }
}

/*
 *  allocate and fill the geofence state structure
 */
static void geofence_load(void)
{
    uint8_t i;

    if (geofence_state == NULL) {
        if (memcheck_available_memory() < 512 + sizeof(struct geofence_state)) {
            // too risky to enable as we could run out of stack
            goto failed;
        }
        geofence_state = (struct geofence_state *)calloc(1, sizeof(struct geofence_state));
        if (geofence_state == NULL) {
            // not much we can do here except disable it
            goto failed;
        }
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

    gcs_send_text_P(SEVERITY_LOW,PSTR("geo-fence loaded"));
    gcs_send_message(MSG_FENCE_STATUS);
    return;

failed:
    g.fence_action.set(FENCE_ACTION_NONE);
    gcs_send_text_P(SEVERITY_HIGH,PSTR("geo-fence setup error"));
}

/*
 *  return true if geo-fencing is enabled
 */
static bool geofence_enabled(void)
{
    if (g.fence_action == FENCE_ACTION_NONE ||
        g.fence_total < 5 ||
        (g.fence_action != FENCE_ACTION_REPORT &&
         (g.fence_channel == 0 ||
          APM_RC.InputCh(g.fence_channel-1) < FENCE_ENABLE_PWM))) {
        // geo-fencing is disabled
        if (geofence_state != NULL) {
            // re-arm for when the channel trigger is switched on
            geofence_state->fence_triggered = false;
        }
        return false;
    }

    return true;
}


/*
 *  return true if we have breached the geo-fence minimum altiude
 */
static bool geofence_check_minalt(void)
{
    if (g.fence_maxalt <= g.fence_minalt) {
        return false;
    }
    if (g.fence_minalt == 0) {
        return false;
    }
    return (current_loc.alt < (g.fence_minalt*100.0) + home.alt);
}

/*
 *  return true if we have breached the geo-fence maximum altiude
 */
static bool geofence_check_maxalt(void)
{
    if (g.fence_maxalt <= g.fence_minalt) {
        return false;
    }
    if (g.fence_maxalt == 0) {
        return false;
    }
    return (current_loc.alt > (g.fence_maxalt*100.0) + home.alt);
}


/*
 *  check if we have breached the geo-fence
 */
static void geofence_check(bool altitude_check_only)
{
    if (!geofence_enabled()) {
        // switch back to the chosen control mode if still in
        // GUIDED to the return point
        if (geofence_state != NULL &&
            g.fence_action == FENCE_ACTION_GUIDED &&
            g.fence_channel != 0 &&
            control_mode == GUIDED &&
            g.fence_total >= 5 &&
            geofence_state->boundary_uptodate &&
            geofence_state->old_switch_position == oldSwitchPosition &&
            guided_WP.lat == geofence_state->boundary[0].x &&
            guided_WP.lng == geofence_state->boundary[0].y) {
            geofence_state->old_switch_position = 0;
            reset_control_switch();
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

    if (geofence_check_minalt()) {
        outside = true;
        breach_type = FENCE_BREACH_MINALT;
    } else if (geofence_check_maxalt()) {
        outside = true;
        breach_type = FENCE_BREACH_MAXALT;
    } else if (!altitude_check_only && ahrs.get_position(&loc)) {
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
            gcs_send_text_P(SEVERITY_LOW,PSTR("geo-fence OK"));
 #if FENCE_TRIGGERED_PIN > 0
            digitalWrite(FENCE_TRIGGERED_PIN, LOW);
 #endif
            gcs_send_message(MSG_FENCE_STATUS);
        }
        // we're inside, all is good with the world
        return;
    }

    // we are outside the fence
    if (geofence_state->fence_triggered &&
        (control_mode == GUIDED || g.fence_action == FENCE_ACTION_REPORT)) {
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
    digitalWrite(FENCE_TRIGGERED_PIN, HIGH);
 #endif

    gcs_send_text_P(SEVERITY_LOW,PSTR("geo-fence triggered"));
    gcs_send_message(MSG_FENCE_STATUS);

    // see what action the user wants
    switch (g.fence_action) {
    case FENCE_ACTION_REPORT:
        break;

    case FENCE_ACTION_GUIDED:
        // fly to the return point, with an altitude half way between
        // min and max
        if (g.fence_minalt >= g.fence_maxalt) {
            // invalid min/max, use RTL_altitude
            guided_WP.alt = home.alt + g.RTL_altitude_cm;
        } else {
            guided_WP.alt = home.alt + 100.0*(g.fence_minalt + g.fence_maxalt)/2;
        }
        guided_WP.id = 0;
        guided_WP.p1  = 0;
        guided_WP.options = 0;
        guided_WP.lat = geofence_state->boundary[0].x;
        guided_WP.lng = geofence_state->boundary[0].y;

        geofence_state->old_switch_position = oldSwitchPosition;

        if (control_mode == MANUAL && g.auto_trim) {
            // make sure we don't auto trim the surfaces on this change
            control_mode = STABILIZE;
        }

        set_mode(GUIDED);
        break;
    }

}

/*
 *  return true if geofencing allows stick mixing. When we have
 *  triggered failsafe and are in GUIDED mode then stick mixing is
 *  disabled. Otherwise the aircraft may not be able to recover from
 *  a breach of the geo-fence
 */
static bool geofence_stickmixing(void) {
    if (geofence_enabled() &&
        geofence_state != NULL &&
        geofence_state->fence_triggered &&
        control_mode == GUIDED) {
        // don't mix in user input
        return false;
    }
    // normal mixing rules
    return true;
}

/*
 *
 */
static void geofence_send_status(mavlink_channel_t chan)
{
    if (geofence_enabled() && geofence_state != NULL) {
        mavlink_msg_fence_status_send(chan,
                                      (int8_t)geofence_state->fence_triggered,
                                      geofence_state->breach_count,
                                      geofence_state->breach_type,
                                      geofence_state->breach_time);
    }
}

// public function for use in failsafe modules
bool geofence_breached(void)
{
    return geofence_state ? geofence_state->fence_triggered : false;
}


#else // GEOFENCE_ENABLED

static void geofence_check(bool altitude_check_only) {
}
static bool geofence_stickmixing(void) {
    return true;
}
static bool geofence_enabled(void) {
    return false;
}

#endif // GEOFENCE_ENABLED
