#include "Plane.h"

#include "quadplane.h"
#include "qautotune.h"

Mode *Plane::mode_from_mode_num(const enum Mode::Number num)
{
    Mode *ret = nullptr;
    switch (num) {
    case Mode::Number::MANUAL:
        ret = &mode_manual;
        break;
    case Mode::Number::CIRCLE:
        ret = &mode_circle;
        break;
    case Mode::Number::STABILIZE:
        ret = &mode_stabilize;
        break;
    case Mode::Number::TRAINING:
        ret = &mode_training;
        break;
    case Mode::Number::ACRO:
        ret = &mode_acro;
        break;
    case Mode::Number::FLY_BY_WIRE_A:
        ret = &mode_fbwa;
        break;
    case Mode::Number::FLY_BY_WIRE_B:
        ret = &mode_fbwb;
        break;
    case Mode::Number::CRUISE:
        ret = &mode_cruise;
        break;
    case Mode::Number::AUTOTUNE:
        ret = &mode_autotune;
        break;
    case Mode::Number::AUTO:
        ret = &mode_auto;
        break;
    case Mode::Number::RTL:
        ret = &mode_rtl;
        break;
    case Mode::Number::LOITER:
        ret = &mode_loiter;
        break;
    case Mode::Number::AVOID_ADSB:
#if HAL_ADSB_ENABLED
        ret = &mode_avoidADSB;
        break;
#endif
    // if ADSB is not compiled in then fallthrough to guided
    case Mode::Number::GUIDED:
        ret = &mode_guided;
        break;
    case Mode::Number::INITIALISING:
        ret = &mode_initializing;
        break;
#if HAL_QUADPLANE_ENABLED
    case Mode::Number::QSTABILIZE:
        ret = &mode_qstabilize;
        break;
    case Mode::Number::QHOVER:
        ret = &mode_qhover;
        break;
    case Mode::Number::QLOITER:
        ret = &mode_qloiter;
        break;
    case Mode::Number::QLAND:
        ret = &mode_qland;
        break;
    case Mode::Number::QRTL:
        ret = &mode_qrtl;
        break;
    case Mode::Number::QACRO:
        ret = &mode_qacro;
        break;
#if QAUTOTUNE_ENABLED
    case Mode::Number::QAUTOTUNE:
        ret = &mode_qautotune;
        break;
#endif
#endif  // HAL_QUADPLANE_ENABLED
    case Mode::Number::TAKEOFF:
        ret = &mode_takeoff;
        break;
    case Mode::Number::THERMAL:
#if HAL_SOARING_ENABLED
        ret = &mode_thermal;
#endif
        break;
#if HAL_QUADPLANE_ENABLED
    case Mode::Number::LOITER_ALT_QLAND:
        ret = &mode_loiter_qland;
        break;
#endif  // HAL_QUADPLANE_ENABLED

    }
    return ret;
}

void Plane::read_control_switch()
{
    static bool switch_debouncer;
    uint8_t switchPosition = readSwitch();

    // If switchPosition = 255 this indicates that the mode control channel input was out of range
    // If we get this value we do not want to change modes.
    if(switchPosition == 255) return;

    if (!rc().has_valid_input()) {
        // ignore the mode switch channel if there is no valid RC input
        return;
    }

    if (millis() - failsafe.last_valid_rc_ms > 100) {
        // only use signals that are less than 0.1s old.
        return;
    }

    if (oldSwitchPosition != switchPosition) {

        if (switch_debouncer == false) {
            // this ensures that mode switches only happen if the
            // switch changes for 2 reads. This prevents momentary
            // spikes in the mode control channel from causing a mode
            // switch
            switch_debouncer = true;
            return;
        }

        set_mode_by_number((enum Mode::Number)flight_modes[switchPosition].get(), ModeReason::RC_COMMAND);

        oldSwitchPosition = switchPosition;
    }

    switch_debouncer = false;

}

uint8_t Plane::readSwitch(void) const
{
    uint16_t pulsewidth = RC_Channels::get_radio_in(g.flight_mode_channel - 1);
    if (pulsewidth <= 900 || pulsewidth >= 2200) return 255;            // This is an error condition
    if (pulsewidth <= 1230) return 0;
    if (pulsewidth <= 1360) return 1;
    if (pulsewidth <= 1490) return 2;
    if (pulsewidth <= 1620) return 3;
    if (pulsewidth <= 1749) return 4;              // Software Manual
    return 5;                                                           // Hardware Manual
}

void Plane::reset_control_switch()
{
    oldSwitchPosition = 254;
    read_control_switch();
}

/*
  called when entering autotune
 */
void Plane::autotune_start(void)
{
    const bool tune_roll = g2.axis_bitmask.get() & int8_t(AutoTuneAxis::ROLL);
    const bool tune_pitch = g2.axis_bitmask.get() & int8_t(AutoTuneAxis::PITCH);
    const bool tune_yaw = g2.axis_bitmask.get() & int8_t(AutoTuneAxis::YAW);
    if (tune_roll || tune_pitch || tune_yaw) {
        gcs().send_text(MAV_SEVERITY_INFO, "Started autotune");
        if (tune_roll) { 
            rollController.autotune_start();
        }
        if (tune_pitch) { 
            pitchController.autotune_start();
        }
        if (tune_yaw) { 
            yawController.autotune_start();
        }
        autotuning = true;
        gcs().send_text(MAV_SEVERITY_INFO, "Autotuning %s%s%s", tune_roll?"roll ":"", tune_pitch?"pitch ":"", tune_yaw?"yaw":"");
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "No axis selected for tuning!");
    }        
}

/*
  called when exiting autotune
 */
void Plane::autotune_restore(void)
{
    rollController.autotune_restore();
    pitchController.autotune_restore();
    yawController.autotune_restore();
    if (autotuning) {
        autotuning = false;
        gcs().send_text(MAV_SEVERITY_INFO, "Stopped autotune");
    }
}

/*
  enable/disable autotune for AUTO modes
 */
void Plane::autotune_enable(bool enable)
{
    if (enable) {
        autotune_start();
    } else {
        autotune_restore();
    }
}

/*
  are we flying inverted?
 */
bool Plane::fly_inverted(void)
{
    if (control_mode == &plane.mode_manual) {
        return false;
    }
    if (inverted_flight) {
        // controlled with aux switch
        return true;
    }
    if (control_mode == &mode_auto && auto_state.inverted_flight) {
        return true;
    }
    return false;
}
