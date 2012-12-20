// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-


static void failsafe_short_on_event(int16_t fstype)
{
    // This is how to handle a short loss of control signal failsafe.
    failsafe = fstype;
    ch3_failsafe_timer = millis();
    gcs_send_text_P(SEVERITY_LOW, PSTR("Failsafe - Short event on, "));
    switch(control_mode)
    {
    case MANUAL:
    case STABILIZE:
    case FLY_BY_WIRE_A:
    case FLY_BY_WIRE_B:
    case TRAINING:
        set_mode(CIRCLE);
        break;

    case AUTO:
    case GUIDED:
    case LOITER:
        if(g.short_fs_action == 1) {
            set_mode(RTL);
        }
        break;

    case CIRCLE:
    case RTL:
    default:
        break;
    }
    gcs_send_text_fmt(PSTR("flight mode = %u"), (unsigned)control_mode);
}

static void failsafe_long_on_event(int16_t fstype)
{
    // This is how to handle a long loss of control signal failsafe.
    gcs_send_text_P(SEVERITY_LOW, PSTR("Failsafe - Long event on, "));
    //  If the GCS is locked up we allow control to revert to RC
    hal.rcin->clear_overrides();
    failsafe = fstype;
    switch(control_mode)
    {
    case MANUAL:
    case STABILIZE:
    case FLY_BY_WIRE_A:
    case FLY_BY_WIRE_B:
    case TRAINING:
    case CIRCLE:
        set_mode(RTL);
        break;

    case AUTO:
    case GUIDED:
    case LOITER:
        if(g.long_fs_action == 1) {
            set_mode(RTL);
        }
        break;

    case RTL:
    default:
        break;
    }
    gcs_send_text_fmt(PSTR("flight mode = %u"), (unsigned)control_mode);
}

static void failsafe_short_off_event()
{
    // We're back in radio contact
    gcs_send_text_P(SEVERITY_LOW, PSTR("Failsafe - Short event off"));
    failsafe = FAILSAFE_NONE;

    // re-read the switch so we can return to our preferred mode
    // --------------------------------------------------------
    if (control_mode == CIRCLE ||
        (g.short_fs_action == 1 && control_mode == RTL)) {
        reset_control_switch();
    }
}

#if BATTERY_EVENT == ENABLED
void low_battery_event(void)
{
    gcs_send_text_P(SEVERITY_HIGH,PSTR("Low Battery!"));
    set_mode(RTL);
    g.throttle_cruise = THROTTLE_CRUISE;
}
#endif

////////////////////////////////////////////////////////////////////////////////
// repeating event control

/*
  update state for MAV_CMD_DO_REPEAT_SERVO and MAV_CMD_DO_REPEAT_RELAY
*/
static void update_events(void)
{
    if (event_state.repeat == 0 || (millis() - event_state.start_time_ms) < event_state.delay_ms) {
        return;
    }

    // event_repeat = -1 means repeat forever
    if (event_state.repeat != 0) {
        event_state.start_time_ms = millis();

        switch (event_state.type) {
        case EVENT_TYPE_SERVO:
            hal.rcout->enable_ch(event_state.rc_channel);
            if (event_state.repeat & 1) {
                hal.rcout->write(event_state.rc_channel, event_state.undo_value);                 
            } else {
                hal.rcout->write(event_state.rc_channel, event_state.servo_value);                 
            }
            break;

        case EVENT_TYPE_RELAY:
#if CONFIG_RELAY == ENABLED
            relay.toggle();
#endif
            break;
        }

        if (event_state.repeat > 0) {
            event_state.repeat--;
        }
    }
}
