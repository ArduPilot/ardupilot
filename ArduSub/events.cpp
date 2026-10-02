#include "Sub.h"

/*
 * events.cpp
 * Failsafe event handlers, called by failsafe.cpp for an action
 */

// Battery failsafe handler
void Sub::handle_battery_failsafe(const char* type_str, const int8_t action)
{
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_BATT, LogErrorCode::FAILSAFE_OCCURRED);

    switch((Failsafe_Action)action) {
        case Failsafe_Action_Surface:
            set_mode(Mode::Number::SURFACE, ModeReason::BATTERY_FAILSAFE);
            break;
        case Failsafe_Action_Disarm:
            arming.disarm(AP_Arming::Method::BATTERYFAILSAFE);
            break;
        case Failsafe_Action_Warn:
        case Failsafe_Action_None:
            break;
    }
}

void Sub::failsafe_terrain_on_event()
{
    failsafe.terrain = true;
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_TERRAIN, LogErrorCode::FAILSAFE_OCCURRED);

    if (!rangefinder_state.enabled || !sub.mode_auto.auto_terrain_recover_start()) {
        failsafe_terrain_act();
    }


}

// Recovery failed, take action
void Sub::failsafe_terrain_act()
{
    switch (g.failsafe_terrain) {
    case FS_TERRAIN_HOLD:
        if (!set_mode(Mode::Number::POSHOLD, ModeReason::TERRAIN_FAILSAFE)) {
            set_mode(Mode::Number::ALT_HOLD, ModeReason::TERRAIN_FAILSAFE);
        }
        AP_Notify::events.failsafe_mode_change = 1;
        break;

    case FS_TERRAIN_SURFACE:
        set_mode(Mode::Number::SURFACE, ModeReason::TERRAIN_FAILSAFE);
        AP_Notify::events.failsafe_mode_change = 1;
        break;

    case FS_TERRAIN_DISARM:
    default:
        arming.disarm(AP_Arming::Method::TERRAINFAILSAFE);
    }
}

#if AP_SUB_RC_ENABLED
// failsafe_radio_on_event - RC contact lost
void Sub::failsafe_radio_on_event()
{
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_RADIO, LogErrorCode::FAILSAFE_OCCURRED);
    gcs().send_text(MAV_SEVERITY_WARNING, "RC Failsafe");
        switch(g.failsafe_throttle) {
        case FS_THR_SURFACE:
            set_mode(Mode::Number::SURFACE, ModeReason::RADIO_FAILSAFE);
            break;
        case FS_THR_WARN:
            set_neutral_controls();
            break;
        case FS_THR_DISABLED:
            break;
    }    
}

// failsafe_radio_off event- respond to radio contact being regained
void Sub::failsafe_radio_off_event()
{
    // no need to do anything except log the error as resolved
    // user can now override roll, pitch, yaw and throttle and even use flight mode switch to restore previous flight mode
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_RADIO, LogErrorCode::FAILSAFE_RESOLVED);
    gcs().send_text(MAV_SEVERITY_WARNING, "Radio Failsafe Cleared");
}
#endif
