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

void RC_Channels_Plane::read_mode_switch()
{
    if (millis() - plane.failsafe.last_valid_rc_ms > 100) {
        // only use signals that are less than 0.1s old.
        return;
    }
    RC_Channels::read_mode_switch();
}

void RC_Channel_Plane::mode_switch_changed(modeswitch_pos_t new_pos)
{
    if (new_pos < 0 || (uint8_t)new_pos > plane.num_flight_modes) {
        // should not have been called
        return;
    }

    plane.set_mode_by_number((Mode::Number)plane.flight_modes[new_pos].get(), ModeReason::RC_COMMAND);
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
