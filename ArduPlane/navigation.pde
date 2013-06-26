// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-


// set the nav_controller pointer to the right controller
static void set_nav_controller(void)
{
    switch ((AP_Navigation::ControllerType)g.nav_controller.get()) {
    case AP_Navigation::CONTROLLER_L1:
        nav_controller = &L1_controller;
        break;
    }
}

/*
  reset the total loiter angle
 */
static void loiter_angle_reset(void)
{
    loiter.sum_cd = 0;
    loiter.total_cd = 0;
}

/*
  update the total angle we have covered in a loiter. Used to support
  commands to do N circles of loiter
 */
static void loiter_angle_update(void)
{
    int32_t target_bearing_cd = nav_controller->target_bearing_cd();
    int32_t loiter_delta_cd;
    if (loiter.sum_cd == 0) {
        // use 1 cd for initial delta
        loiter_delta_cd = 1;
    } else {
        loiter_delta_cd = target_bearing_cd - loiter.old_target_bearing_cd;
    }
    loiter.old_target_bearing_cd = target_bearing_cd;
    loiter_delta_cd = wrap_180_cd(loiter_delta_cd);

    loiter.sum_cd += loiter_delta_cd;
}

//****************************************************************
// Function that will calculate the desired direction to fly and distance
//****************************************************************
static void navigate()
{
    // allow change of nav controller mid-flight
    set_nav_controller();

    // do not navigate with corrupt data
    // ---------------------------------
    if (!have_position) {
        return;
    }

    if (next_WP.lat == 0) {
        return;
    }

    // waypoint distance from plane
    // ----------------------------
    wp_distance = get_distance(&current_loc, &next_WP);

    if (wp_distance < 0) {
        gcs_send_text_P(SEVERITY_HIGH,PSTR("WP error - distance < 0"));
        return;
    }

    // update total loiter angle
    loiter_angle_update();

    // control mode specific updates to navigation demands
    // ---------------------------------------------------
    update_navigation();
}


static void calc_airspeed_errors()
{
    float aspeed_cm = airspeed.get_airspeed_cm();

    // Normal airspeed target
    target_airspeed_cm = g.airspeed_cruise_cm;

    // FBW_B airspeed target
    if (control_mode == FLY_BY_WIRE_B) {
        target_airspeed_cm = ((int32_t)(aparm.flybywire_airspeed_max -
                                        aparm.flybywire_airspeed_min) *
                              channel_throttle->servo_out) +
                             ((int32_t)aparm.flybywire_airspeed_min * 100);
    }

    // Set target to current airspeed + ground speed undershoot,
    // but only when this is faster than the target airspeed commanded
    // above.
    if (control_mode >= FLY_BY_WIRE_B && (g.min_gndspeed_cm > 0)) {
        int32_t min_gnd_target_airspeed = aspeed_cm + groundspeed_undershoot;
        if (min_gnd_target_airspeed > target_airspeed_cm)
            target_airspeed_cm = min_gnd_target_airspeed;
    }

    // Bump up the target airspeed based on throttle nudging
    if (control_mode >= AUTO && airspeed_nudge_cm > 0) {
        target_airspeed_cm += airspeed_nudge_cm;
    }

    // Apply airspeed limit
    if (target_airspeed_cm > (aparm.flybywire_airspeed_max * 100))
        target_airspeed_cm = (aparm.flybywire_airspeed_max * 100);

    airspeed_error_cm = target_airspeed_cm - aspeed_cm;
    airspeed_energy_error = ((target_airspeed_cm * target_airspeed_cm) - (aspeed_cm*aspeed_cm))*0.00005;
}

static void calc_gndspeed_undershoot()
{
 	// Use the component of ground speed in the forward direction
	// This prevents flyaway if wind takes plane backwards
    if (g_gps && g_gps->status() >= GPS::GPS_OK_FIX_2D) {
	    Vector2f gndVel = ahrs.groundspeed_vector();
		const Matrix3f &rotMat = ahrs.get_dcm_matrix();
		Vector2f yawVect = Vector2f(rotMat.a.x,rotMat.b.x);
		yawVect.normalize();
		float gndSpdFwd = yawVect * gndVel;
        groundspeed_undershoot = (g.min_gndspeed_cm > 0) ? (g.min_gndspeed_cm - gndSpdFwd*100) : 0;
    }
}

static void calc_altitude_error()
{
    if (control_mode == FLY_BY_WIRE_B) {
        return;
    }
    if (control_mode == AUTO && offset_altitude_cm != 0) {
        // limit climb rates
        target_altitude_cm = next_WP.alt - (offset_altitude_cm*((float)(wp_distance-30) / (float)(wp_totalDistance-30)));

        // stay within a certain range
        if(prev_WP.alt > next_WP.alt) {
            target_altitude_cm = constrain_int32(target_altitude_cm, next_WP.alt, prev_WP.alt);
        }else{
            target_altitude_cm = constrain_int32(target_altitude_cm, prev_WP.alt, next_WP.alt);
        }
    } else if (non_nav_command_ID != MAV_CMD_CONDITION_CHANGE_ALT) {
        target_altitude_cm = next_WP.alt;
    }

    altitude_error_cm       = target_altitude_cm - adjusted_altitude_cm();
}

static void update_loiter()
{
    nav_controller->update_loiter(next_WP, abs(g.loiter_radius), loiter.direction);
}

