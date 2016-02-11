// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Plane.h"

void Plane::failsafe_short_on_event(enum failsafe_state fstype)
{
    // This is how to handle a short loss of control signal failsafe.
    failsafe.state = fstype;
    failsafe.ch3_timer_ms = millis();
    gcs_send_text(MAV_SEVERITY_WARNING, "Failsafe. Short event on, ");
    switch(control_mode)
    {
    case MANUAL:
    case STABILIZE:
    case ACRO:
    case FLY_BY_WIRE_A:
    case AUTOTUNE:
    case FLY_BY_WIRE_B:
    case CRUISE:
    case TRAINING:
        failsafe.saved_mode = control_mode;
        failsafe.saved_mode_set = 1;
        if(g.short_fs_action == 2) {
            set_mode(FLY_BY_WIRE_A);
        } else {
            set_mode(CIRCLE);
        }
        break;

    case QSTABILIZE:
    case QLOITER:
        failsafe.saved_mode = control_mode;
        failsafe.saved_mode_set = 1;
        set_mode(QHOVER);
        break;
        
    case AUTO:
    case GUIDED:
    case LOITER:
        if(g.short_fs_action != 0) {
            failsafe.saved_mode = control_mode;
            failsafe.saved_mode_set = 1;
            if(g.short_fs_action == 2) {
                set_mode(FLY_BY_WIRE_A);
            } else {
                set_mode(CIRCLE);
            }
        }
        break;

    case CIRCLE:
    case RTL:
    case QHOVER:
    default:
        break;
    }
    gcs_send_text_fmt(MAV_SEVERITY_INFO, "Flight mode = %u", (unsigned)control_mode);
}

void Plane::failsafe_long_on_event(enum failsafe_state fstype)
{
    // This is how to handle a long loss of control signal failsafe.
    gcs_send_text(MAV_SEVERITY_WARNING, "Failsafe. Long event on, ");
    //  If the GCS is locked up we allow control to revert to RC
    hal.rcin->clear_overrides();
    failsafe.state = fstype;
    switch(control_mode)
    {
    case MANUAL:
    case STABILIZE:
    case ACRO:
    case FLY_BY_WIRE_A:
    case AUTOTUNE:
    case FLY_BY_WIRE_B:
    case CRUISE:
    case TRAINING:
    case CIRCLE:
        if(g.long_fs_action == 3) {
#if PARACHUTE == ENABLED
            parachute_release();
#endif
        } else if (g.long_fs_action == 2) {
            set_mode(FLY_BY_WIRE_A);
        } else {
            set_mode(RTL);
        }
        break;

    case QSTABILIZE:
    case QLOITER:
        set_mode(QHOVER);
        break;
        
    case AUTO:
    case GUIDED:
    case LOITER:
        if(g.long_fs_action == 3) {
#if PARACHUTE == ENABLED
            parachute_release();
#endif
        } else if (g.long_fs_action == 2) {
            set_mode(FLY_BY_WIRE_A);
        } else if (g.long_fs_action == 1) {
            set_mode(RTL);
        }
        break;

    case RTL:
    case QHOVER:
    default:
        break;
    }
    if (fstype == FAILSAFE_GCS) {
        gcs_send_text(MAV_SEVERITY_CRITICAL, "No GCS heartbeat");
    }
    gcs_send_text_fmt(MAV_SEVERITY_INFO, "Flight mode = %u", (unsigned)control_mode);
}

void Plane::failsafe_short_off_event()
{
    // We're back in radio contact
    gcs_send_text(MAV_SEVERITY_WARNING, "Failsafe. Short event off");
    failsafe.state = FAILSAFE_NONE;

    // re-read the switch so we can return to our preferred mode
    // --------------------------------------------------------
    if (control_mode == CIRCLE && failsafe.saved_mode_set) {
        failsafe.saved_mode_set = 0;
        set_mode(failsafe.saved_mode);
    }
}

void Plane::low_battery_event(void)
{
    if (failsafe.low_battery) {
        return;
    }
    gcs_send_text_fmt(MAV_SEVERITY_WARNING, "Low battery %.2fV used %.0f mAh",
                      (double)battery.voltage(), (double)battery.current_total_mah());
    if (flight_stage != AP_SpdHgtControl::FLIGHT_LAND_FINAL &&
        flight_stage != AP_SpdHgtControl::FLIGHT_LAND_PREFLARE &&
        flight_stage != AP_SpdHgtControl::FLIGHT_LAND_APPROACH) {
    	set_mode(RTL);
    	aparm.throttle_cruise.load();
    }
    failsafe.low_battery = true;
    AP_Notify::flags.failsafe_battery = true;
}

void Plane::update_events(void)
{
    ServoRelayEvents.update_events();
}
