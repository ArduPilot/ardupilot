// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if BATTERY_EVENT == ENABLED
void low_battery_event(void)
{
    gcs_send_text_P(SEVERITY_HIGH,PSTR("Low Battery!"));
    set_mode(RTL);
    g.throttle_cruise = THROTTLE_CRUISE;
}
#endif
