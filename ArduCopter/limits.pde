// Main state machine loop for AP_Limits. Called from slow or superslow loop.

#if AP_LIMITS == ENABLED

uint8_t lim_state = 0, lim_old_state = 0;

void set_recovery_home_alt() {

    uint32_t return_altitude_cm_ahl = 0;     // in centimeters above home level.
    uint32_t amin_meters_ahl, amax_meters_ahl;

    // for flying vehicles only
    if (altitude_limit.enabled()) {

        amin_meters_ahl = (uint32_t) (altitude_limit.min_alt());
        amax_meters_ahl = (uint32_t) (altitude_limit.max_alt());

        // See if we have a meaningful setting
        if (amax_meters_ahl && ((amax_meters_ahl - amin_meters_ahl) > 1)) {
            // there is a max_alt set
            // set a return altitude that is halfway between the minimum and maximum altitude setting.
            // return_altitude is in centimeters, not meters, so we multiply
            return_altitude_cm_ahl = (uint32_t) (home.alt + (100 * (uint16_t) ((amax_meters_ahl - amin_meters_ahl) / 2)));
        }
    } else {
        return_altitude_cm_ahl = (uint32_t) (home.alt + g.RTL_altitude*100);
    }
    // final sanity check
    // if our return is less than 4 meters from ground, set it to 4m, to clear "people" height.
    if ((return_altitude_cm_ahl - (uint32_t) home.alt) < 400) {
        return_altitude_cm_ahl = home.alt + 400;
    }
    guided_WP.id = 0;
    guided_WP.p1 = 0;
    guided_WP.options = 0;
    guided_WP.lat = home.lat;
    guided_WP.lng = home.lng;
    guided_WP.alt = return_altitude_cm_ahl;
}

static void limits_loop() {

    lim_state = limits.state();

    // Use limits channel to determine LIMITS_ENABLED or LIMITS_DISABLED state
    if (lim_state != LIMITS_DISABLED  && limits.channel() !=0 && APM_RC.InputCh(limits.channel()-1) < LIMITS_ENABLE_PWM) {
        limits.set_state(LIMITS_DISABLED);
    }
    else if (lim_state == LIMITS_DISABLED && limits.channel() !=0 && APM_RC.InputCh(limits.channel()-1) >= LIMITS_ENABLE_PWM) {
        limits.set_state(LIMITS_ENABLED);
    }

    if ((uint32_t) millis() - (uint32_t) limits.last_status_update > 1000) {     // more than a second has passed - time for an update
        gcs_send_message(MSG_LIMITS_STATUS);
    }

    if (lim_state != lim_old_state) {     // state changed
        lim_old_state = lim_state;         // we only use lim_oldstate here, for reporting purposes. So, reset it.
        gcs_send_message(MSG_LIMITS_STATUS);

        if (limits.debug()) switch (lim_state) {
            case LIMITS_INIT: gcs_send_text_P(SEVERITY_LOW,PSTR("Limits - State change: INIT")); break;
            case LIMITS_DISABLED: gcs_send_text_P(SEVERITY_LOW,PSTR("Limits - State change: DISABLED")); break;
            case LIMITS_ENABLED: gcs_send_text_P(SEVERITY_LOW,PSTR("Limits - State change: ENABLED")); break;
            case LIMITS_TRIGGERED: gcs_send_text_P(SEVERITY_LOW,PSTR("Limits - State change: TRIGGERED")); break;
            case LIMITS_RECOVERING: gcs_send_text_P(SEVERITY_LOW,PSTR("Limits - State change: RECOVERING")); break;
            case LIMITS_RECOVERED: gcs_send_text_P(SEVERITY_LOW,PSTR("Limits - State change: RECOVERED")); break;
            default: gcs_send_text_P(SEVERITY_LOW,PSTR("Limits - State change: UNKNOWN")); break;
            }
    }

    switch (limits.state()) {

    // have not initialized yet
    case LIMITS_INIT:
        if (limits.init()) {                 // initialize system

            // See what the "master" on/off swith is and go to the appropriate start state
            if (!limits.enabled()) {
                limits.set_state(LIMITS_DISABLED);
            }
            else {
                limits.set_state(LIMITS_ENABLED);
            }
        }
        break;

    // We have been switched off
    case LIMITS_DISABLED:

        // check if we have been switched on
        if (limits.enabled()) {
            limits.set_state(LIMITS_ENABLED);
            break;
        }
        break;

    // Limits module is enabled
    case LIMITS_ENABLED:

        // check if we've been switched off
        if (!limits.enabled()) {
            limits.set_state(LIMITS_DISABLED);
            break;
        }

        // Until motors are armed, do nothing, just wait in ENABLED state
        if (!motors.armed()) {

            // we are waiting for motors to arm
            // do nothing
            break;
        }

        bool required_only;

        required_only = (limits.last_clear == 0);                 // if we haven't yet 'cleared' all limits, check required limits only

        // check if any limits have been breached and trigger if they have
        if  (limits.check_triggered(required_only)) {

            //
            // TRIGGER - BREACH OF LIMITS
            //
            // make a note of which limits triggered, so if we know if we recovered them
            limits.mods_recovering = limits.mods_triggered;

            limits.last_action = 0;
            limits.last_trigger = millis();
            limits.breach_count++;

            limits.set_state(LIMITS_TRIGGERED);
            break;
        }

        if (motors.armed() && limits.enabled() && !limits.mods_triggered) {

            // All clear.
	    if (limits.debug()) gcs_send_text_P(SEVERITY_LOW, PSTR("Limits - All Clear"));
            limits.last_clear = millis();
        }

        break;

    // Limits have been triggered
    case LIMITS_TRIGGERED:

        // check if we've been switched off
        if (!limits.enabled()) {
            limits.set_state(LIMITS_DISABLED);
            break;
        }

#if LIMITS_TRIGGERED_PIN > 0
        digitalWrite(LIMITS_TRIGGERED_PIN, HIGH);
#endif

        if (limits.debug()) {
		if (limits.mods_triggered & LIMIT_GPSLOCK) gcs_send_text_P(SEVERITY_LOW, PSTR("!GPSLock"));
		if (limits.mods_triggered & LIMIT_GEOFENCE) gcs_send_text_P(SEVERITY_LOW, PSTR("!Geofence"));
		if (limits.mods_triggered & LIMIT_ALTITUDE) gcs_send_text_P(SEVERITY_LOW, PSTR("!Altitude"));
        }

        // If the motors are not armed, we have triggered pre-arm checks. Do nothing
        if (motors.armed() == false) {
            limits.set_state(LIMITS_ENABLED);                     // go back to checking limits
            break;
        }

        // If we are triggered but no longer in breach, that means we recovered
        // somehow, via auto recovery or pilot action
        if (!limits.check_all()) {
            limits.last_recovery = millis();
            limits.set_state(LIMITS_RECOVERED);
            break;
        }
        else {
            limits.set_state(LIMITS_RECOVERING);
            limits.last_action = 0;                     // reset timer
            // We are about to take action on a real breach. Make sure we notify immediately
            gcs_send_message(MSG_LIMITS_STATUS);
            break;
        }
        break;

    // Take action to recover
    case LIMITS_RECOVERING:
        // If the motors are not armed, we have triggered pre-arm checks. Do nothing
        if (motors.armed() == false) {
            limits.set_state(LIMITS_ENABLED);                     // go back to checking limits
            break;
        }

        // check if we've been switched off
        if (!limits.enabled() && limits.old_mode_switch == oldSwitchPosition) {
            limits.old_mode_switch = 0;
            reset_control_switch();
            limits.set_state(LIMITS_DISABLED);
            break;
        }

        // Still need action?
        if (limits.check_all() == 0) {                 // all triggers clear
            limits.set_state(LIMITS_RECOVERED);
            break;
        }

        if (limits.mods_triggered != limits.mods_recovering)  {                 // if any *new* triggers, hit the trigger again
            //
            // TRIGGER - BREACH OF LIMITS
            //
            // make a note of which limits triggered, so if we know if we recovered them
            limits.mods_recovering = limits.mods_triggered;

            limits.last_action = 0;
            limits.last_trigger = millis();
            limits.breach_count++;

            limits.set_state(LIMITS_TRIGGERED);
            limits.set_state(LIMITS_TRIGGERED);
            break;
        }

        // Recovery Action
        // if there was no previous action, take action, take note of time  send GCS.
        if (limits.last_action == 0) {

            // save mode switch
            limits.old_mode_switch = oldSwitchPosition;


//				// Take action
//				// This ensures no "radical" RTL, like a full throttle take-off,happens if something triggers at ground level
//				if ((uint32_t) current_loc.alt < ((uint32_t)home.alt * 200) ) { // we're under 2m (200cm), already at "people" height or on the ground
//					if (limits.debug()) gcs_send_text_P(SEVERITY_LOW,PSTR("Limits Action: near ground - do nothing"));
//					// TODO: Will this work for a plane? Does it make sense in general?
//
//					//set_mode(LAND);
//					limits.last_action = millis(); // start counter
//				    gcs_send_message(MSG_LIMITS_STATUS);
//
//					break;
//				}


            // TODO: This applies only to planes - hold for porting
//					if (control_mode == MANUAL && g.auto_trim) {
//					            // make sure we don't auto trim the surfaces on this change
//					            control_mode = STABILIZE;
//					}


            switch (limits.recmode()) {

            case 0:                                     // RTL mode

                if (limits.debug()) gcs_send_text_P(SEVERITY_LOW,PSTR("Limits Action - RTL"));

                set_mode(RTL);
                limits.last_action = millis();
                gcs_send_message(MSG_LIMITS_STATUS);
                break;

            case 1:                                     // Bounce mode

                if (limits.debug()) gcs_send_text_P(SEVERITY_LOW,PSTR("Limits Action - bounce mode, POSITION"));
                // ALT_HOLD gives us yaw hold, roll& pitch hold and throttle hold.
                // It is like position hold, but without manual throttle control.

                //set_recovery_home_alt();
                set_mode(POSITION);
                set_throttle_mode(THROTTLE_AUTO);
                limits.last_action = millis();
                gcs_send_message(MSG_LIMITS_STATUS);
                break;

            }
            break;
        }


        // In bounce mode, take control for 3 seconds, and then wait for the pilot to make us "safe".
        // If the vehicle does not recover, the escalation action will trigger.
        if (limits.recmode() == 1) {

            if (control_mode == POSITION && ((uint32_t)millis() - (uint32_t)limits.last_action) > 3000) {
                if (limits.debug()) gcs_send_text_P(SEVERITY_LOW,PSTR("Limits Recovery Bounce: Returning control to pilot"));
                set_mode(STABILIZE);
            } else if (control_mode == STABILIZE && ((uint32_t)millis() - (uint32_t)limits.last_action) > 6000) {
                // after 3 more seconds, reset action counter to take action again
                limits.last_action = 0;
            }
        }

        // ESCALATE We have not recovered after 2 minutes of recovery action

        if (((uint32_t)millis() - (uint32_t)limits.last_action) > 120000 ) {

            // TODO: Secondary recovery
            if (limits.debug()) gcs_send_text_P(SEVERITY_LOW,PSTR("Limits Recovery Escalation: RTL"));
            set_mode(RTL);
            limits.last_action = millis();
            break;
        }
        break;

    // Have recovered, relinquish control and re-enable
    case LIMITS_RECOVERED:


        // check if we've been switched off
        if (!limits.enabled()) {
            limits.set_state(LIMITS_DISABLED);
            break;
        }

#if LIMITS_TRIGGERED_PIN > 0
        digitalWrite(LIMITS_TRIGGERED_PIN, LOW);
#endif

        // Reset action counter
        limits.last_action = 0;

        if (((uint32_t)millis() - (uint32_t)limits.last_recovery) > (uint32_t)(limits.safetime() * 1000)) {                 // Wait "safetime" seconds of recovery before we give back control

            // Our recovery action worked.
            limits.set_state(LIMITS_ENABLED);

            // Switch to stabilize
            if (limits.debug()) gcs_send_text_P(SEVERITY_LOW,PSTR("Limits - Returning controls"));
            set_mode(STABILIZE);                            limits.last_recovery = millis();

            break;
        }
        break;

    default:
        if (limits.debug()) gcs_send_text_P(SEVERITY_LOW,PSTR("Limits: unknown state"));
        break;
    }
}

// This function below, should really be in the AP_Limits class, but it is impossible to untangle the mavlink includes.

void limits_send_mavlink_status(mavlink_channel_t chan) {

    limits.last_status_update = millis();

    if (limits.enabled()) {
        mavlink_msg_limits_status_send(chan,
                                       (uint8_t) limits.state(),
                                       (uint32_t) limits.last_trigger,
                                       (uint32_t) limits.last_action,
                                       (uint32_t) limits.last_recovery,
                                       (uint32_t) limits.last_clear,
                                       (uint16_t) limits.breach_count,
                                       (LimitModuleBits) limits.mods_enabled,
                                       (LimitModuleBits) limits.mods_required,
                                       (LimitModuleBits) limits.mods_triggered);
    }
}

#endif
