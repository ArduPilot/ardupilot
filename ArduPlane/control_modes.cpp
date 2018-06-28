#include "Plane.h"

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
        ret = &mode_avoidADSB;
        break;
    case Mode::Number::GUIDED:
        ret = &mode_guided;
        break;
    case Mode::Number::INITIALISING:
        ret = &mode_initializing;
        break;
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
    case Mode::Number::QAUTOTUNE:
        ret = &mode_qautotune;
        break;
    default:
        break;
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

    if (failsafe.rc_failsafe || failsafe.throttle_counter > 0) {
        // when we are in rc_failsafe mode then RC input is not
        // working, and we need to ignore the mode switch channel
        return;
    }

    if (millis() - failsafe.last_valid_rc_ms > 100) {
        // only use signals that are less than 0.1s old.
        return;
    }

    // we look for changes in the switch position. If the
    // RST_SWITCH_CH parameter is set, then it is a switch that can be
    // used to force re-reading of the control switch. This is useful
    // when returning to the previous mode after a failsafe or fence
    // breach. This channel is best used on a momentary switch (such
    // as a spring loaded trainer switch).
    if (oldSwitchPosition != switchPosition ||
        (g.reset_switch_chan != 0 &&
         RC_Channels::get_radio_in(g.reset_switch_chan-1) > RESET_SWITCH_CHAN_PWM)) {

        if (switch_debouncer == false) {
            // this ensures that mode switches only happen if the
            // switch changes for 2 reads. This prevents momentary
            // spikes in the mode control channel from causing a mode
            // switch
            switch_debouncer = true;
            return;
        }

        Mode *new_mode = mode_from_mode_num((enum Mode::Number)modes[switchPosition].get());
        if (new_mode != nullptr) {
            set_mode(*new_mode, MODE_REASON_TX_COMMAND);
        }

        oldSwitchPosition = switchPosition;
    }

    if (g.reset_mission_chan != 0 &&
        RC_Channels::get_radio_in(g.reset_mission_chan-1) > RESET_SWITCH_CHAN_PWM) {
        mission.start();
        prev_WP_loc = current_loc;
    }

    switch_debouncer = false;

#if PARACHUTE == ENABLED
    if (g.parachute_channel > 0) {
        if (RC_Channels::get_radio_in(g.parachute_channel-1) >= 1700) {
            parachute_manual_release();
        }
    }
#endif
    
#if HAVE_PX4_MIXER
    if (g.override_channel > 0) {
        // if the user has configured an override channel then check it
        bool override_requested = (RC_Channels::get_radio_in(g.override_channel-1) >= PX4IO_OVERRIDE_PWM);
        if (override_requested && !px4io_override_enabled) {
            if (hal.util->get_soft_armed() || (last_mixer_crc != -1)) {
                px4io_override_enabled = true;
                // disable output channels to force PX4IO override
                gcs().send_text(MAV_SEVERITY_WARNING, "PX4IO override enabled");
            } else {
                // we'll let the one second loop reconfigure the mixer. The
                // PX4IO code sometimes rejects a mixer, probably due to it
                // being busy in some way?
                gcs().send_text(MAV_SEVERITY_WARNING, "PX4IO override enable failed");
            }
        } else if (!override_requested && px4io_override_enabled) {
            px4io_override_enabled = false;
            SRV_Channels::enable_aux_servos();
            gcs().send_text(MAV_SEVERITY_WARNING, "PX4IO override disabled");
        }
        if (px4io_override_enabled && 
            hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_ARMED &&
            g.override_safety == 1) {
            // we force safety off, so that if this override is used
            // with a in-flight reboot it gives a way for the pilot to
            // re-arm and take manual control
            hal.rcout->force_safety_off();
        }
    }
#endif // HAVE_PX4_MIXER
}

uint8_t Plane::readSwitch(void)
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
    rollController.autotune_start();
    pitchController.autotune_start();
}

/*
  called when exiting autotune
 */
void Plane::autotune_restore(void)
{
    rollController.autotune_restore();
    pitchController.autotune_restore();
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
