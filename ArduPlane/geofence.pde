// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
  geo-fencing support
  Andrew Tridgell, December 2011
 */

#if GEOFENCE_ENABLED == ENABLED

/*
  The state of geo-fencing. This structure is dynamically allocated
  the first time it is used. This means we only pay for the pointer
  and not the structure on systems where geo-fencing is not being
  used.

  We store a copy of the boundary in memory as we need to access it
  very quickly at runtime
 */
static struct geofence_state {
    uint8_t num_points;
    bool boundary_uptodate;
    bool fence_triggered;
    /* point 0 is the return point */
    Vector2f boundary[MAX_FENCEPOINTS];
} *geofence_state;


/*
  fence boundaries fetch/store
 */
static Vector2f get_fence_point_with_index(unsigned i)
{
	uint32_t mem;
    Vector2f ret;

	if (i > (unsigned)g.fence_total) {
        return Vector2f(0,0);
    }

    // read fence point
    mem = FENCE_START_BYTE + (i * FENCE_WP_SIZE);
    eeprom_read_block(&ret.x, (void *)mem, sizeof(float));
    mem += sizeof(float);
    eeprom_read_block(&ret.y, (void *)mem, sizeof(float));

	return ret;
}

// save a fence point
static void set_fence_point_with_index(Vector2f &point, unsigned i)
{
	uint32_t mem;

	if (i >= (unsigned)g.fence_total.get()) {
        // not allowed
        return;
    }

    mem = FENCE_START_BYTE + (i * FENCE_WP_SIZE);

	eeprom_write_block(&point.x, (void *)mem, sizeof(float));
	mem += 4;
	eeprom_write_block(&point.y, (void *)mem, sizeof(float));

    if (geofence_state != NULL) {
        geofence_state->boundary_uptodate = false;
    }
}

/*
  allocate and fill the geofence state structure
 */
static void geofence_load(void)
{
    uint8_t i;

    if (geofence_state == NULL) {
        if (memcheck_available_memory() < 1024 + sizeof(struct geofence_state)) {
            // too risky to enable as we could run out of stack
            goto failed;
        }
        geofence_state = (struct geofence_state *)calloc(1, sizeof(struct geofence_state));
        if (geofence_state == NULL) {
            // not much we can do here except disable it
            goto failed;
        }
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
    return;

failed:
    g.fence_action.set(FENCE_ACTION_NONE);
    gcs_send_text_P(SEVERITY_HIGH,PSTR("geo-fence setup error"));
}

/*
  return true if geo-fencing is enabled
 */
static bool geofence_enabled(void)
{
    if (g.fence_action == FENCE_ACTION_NONE ||
        g.fence_channel == 0 ||
        APM_RC.InputCh(g.fence_channel-1) < FENCE_ENABLE_PWM) {
        // geo-fencing is disabled
        if (geofence_state != NULL) {
            // re-arm for when the channel trigger is switched on
            geofence_state->fence_triggered = false;
        }
        return false;
    }

    if (!g_gps->fix) {
        // we can't do much without a GPS fix
        return false;
    }

    return true;
}


/*
  check if we have breached the geo-fence
 */
static void geofence_check(void)
{
    if (!geofence_enabled()) {
        return;
    }

    /* allocate the geo-fence state if need be */
    if (geofence_state == NULL || !geofence_state->boundary_uptodate) {
        geofence_load();
        if (g.fence_action == FENCE_ACTION_NONE) {
            // may have been disabled by load
            return;
        }
    }

    bool outside = false;

    if (g.fence_maxalt > g.fence_minalt &&
        ((g.fence_minalt != 0 && current_loc.alt < (g.fence_minalt*100) + home.alt) ||
          (current_loc.alt > (g.fence_maxalt*100) + home.alt))) {
        // we are too high or low
        outside = true;
    } else {
        Vector2f location;
        location.x = 1.0e-7 * current_loc.lat;
        location.y = 1.0e-7 * current_loc.lng;
        outside = Polygon_outside(location, &geofence_state->boundary[1], geofence_state->num_points-1);
    }

    if (!outside) {
        if (geofence_state->fence_triggered) {
            // we have moved back inside the fence
            geofence_state->fence_triggered = false;
        }
        // we're inside, all is good with the world
        return;
    }

    // we are outside the fence
    if (geofence_state->fence_triggered) {
        // we have already triggered, don't trigger again until the
        // user disables/re-enables using the fence channel switch
        return;
    }


    // we are outside, and have not previously triggered.
    geofence_state->fence_triggered = true;
    gcs_send_text_P(SEVERITY_LOW,PSTR("geo-fence triggered"));

    // see what action the user wants
    switch (g.fence_action) {
    case FENCE_ACTION_GUIDED:
        // fly to the return point, with an altitude half way between
        // min and max
        if (g.fence_minalt >= g.fence_maxalt) {
            // invalid min/max, use RTL_altitude
            guided_WP.alt = home.alt + (g.RTL_altitude * 100);
        } else {
            guided_WP.alt = home.alt + 100*(g.fence_minalt + g.fence_maxalt)/2;
        }
        guided_WP.id = 0;
        guided_WP.p1  = 0;
        guided_WP.options = 0;
        guided_WP.lat = geofence_state->boundary[0].x * 1.0e7;
        guided_WP.lng = geofence_state->boundary[0].y * 1.0e7;
        set_mode(GUIDED);
        break;
    }

}

/*
  return true if geofencing allows stick mixing. When we have
  triggered failsafe and are in GUIDED mode then stick mixing is
  disabled. Otherwise the aircraft may not be able to recover from
  a breach of the geo-fence
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

#else // GEOFENCE_ENABLED

static void geofence_check(void) { }
static bool geofence_stickmixing(void) { return true; }

#endif // GEOFENCE_ENABLED
