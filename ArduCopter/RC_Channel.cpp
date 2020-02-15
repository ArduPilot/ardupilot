#include "Copter.h"

#include "RC_Channel.h"

// defining these two macros and including the RC_Channels_VarInfo header defines the parameter information common to all vehicle types
#define RC_CHANNELS_SUBCLASS RC_Channels_Copter
#define RC_CHANNEL_SUBCLASS RC_Channel_Copter

#include <RC_Channel/RC_Channels_VarInfo.h>

int8_t RC_Channels_Copter::flight_mode_channel_number() const
{
    return copter.g.flight_mode_chan.get();
}

void RC_Channel_Copter::mode_switch_changed(modeswitch_pos_t new_pos)
{
    if (new_pos < 0 || (uint8_t)new_pos > copter.num_flight_modes) {
        // should not have been called
        return;
    }

    if (!copter.set_mode((Mode::Number)copter.flight_modes[new_pos].get(), ModeReason::RC_COMMAND)) {
        // alert user to mode change failure
        if (copter.ap.initialised) {
            AP_Notify::events.user_mode_change_failed = 1;
        }
        return;
    }

    // play a tone
    // alert user to mode change (except if autopilot is just starting up)
    if (copter.ap.initialised) {
        AP_Notify::events.user_mode_change = 1;
    }

    if (!rc().find_channel_for_option(AUX_FUNC::SIMPLE_MODE) &&
        !rc().find_channel_for_option(AUX_FUNC::SUPERSIMPLE_MODE)) {
        // if none of the Aux Switches are set to Simple or Super Simple Mode then
        // set Simple Mode using stored parameters from EEPROM
        if (BIT_IS_SET(copter.g.super_simple, new_pos)) {
            copter.set_simple_mode(2);
        } else {
            copter.set_simple_mode(BIT_IS_SET(copter.g.simple_modes, new_pos));
        }
    }
}

bool RC_Channels_Copter::has_valid_input() const
{
    if (copter.failsafe.radio) {
        return false;
    }
    if (copter.failsafe.radio_counter != 0) {
        return false;
    }
    return true;
}


// init_aux_switch_function - initialize aux functions
void RC_Channel_Copter::init_aux_function(const aux_func_t ch_option, const aux_switch_pos_t ch_flag)
{
    // init channel options
    switch(ch_option) {
    // the following functions do not need to be initialised:
    case AUX_FUNC::ALTHOLD:
    case AUX_FUNC::AUTO:
    case AUX_FUNC::AUTOTUNE:
    case AUX_FUNC::BRAKE:
    case AUX_FUNC::CIRCLE:
    case AUX_FUNC::DRIFT:
    case AUX_FUNC::FLIP:
    case AUX_FUNC::FLOWHOLD:
    case AUX_FUNC::FOLLOW:
    case AUX_FUNC::GUIDED:
    case AUX_FUNC::LAND:
    case AUX_FUNC::LOITER:
    case AUX_FUNC::PARACHUTE_RELEASE:
    case AUX_FUNC::POSHOLD:
    case AUX_FUNC::RESETTOARMEDYAW:
    case AUX_FUNC::RTL:
    case AUX_FUNC::SAVE_TRIM:
    case AUX_FUNC::SAVE_WP:
    case AUX_FUNC::SMART_RTL:
    case AUX_FUNC::STABILIZE:
    case AUX_FUNC::THROW:
    case AUX_FUNC::USER_FUNC1:
    case AUX_FUNC::USER_FUNC2:
    case AUX_FUNC::USER_FUNC3:
    case AUX_FUNC::WINCH_CONTROL:
    case AUX_FUNC::ZIGZAG:
    case AUX_FUNC::ZIGZAG_SaveWP:
        break;
    case AUX_FUNC::ACRO_TRAINER:
    case AUX_FUNC::ATTCON_ACCEL_LIM:
    case AUX_FUNC::ATTCON_FEEDFWD:
    case AUX_FUNC::INVERTED:
    case AUX_FUNC::MOTOR_INTERLOCK:
    case AUX_FUNC::PARACHUTE_3POS:      // we trust the vehicle will be disarmed so even if switch is in release position the chute will not release
    case AUX_FUNC::PARACHUTE_ENABLE:
    case AUX_FUNC::PRECISION_LOITER:
    case AUX_FUNC::RANGEFINDER:
    case AUX_FUNC::RETRACT_MOUNT:
    case AUX_FUNC::SIMPLE_MODE:
    case AUX_FUNC::STANDBY:
    case AUX_FUNC::SUPERSIMPLE_MODE:
    case AUX_FUNC::SURFACE_TRACKING:
    case AUX_FUNC::WINCH_ENABLE:
        do_aux_function(ch_option, ch_flag);
        break;
    default:
        RC_Channel::init_aux_function(ch_option, ch_flag);
        break;
    }
}

// do_aux_function_change_mode - change mode based on an aux switch
// being moved
void RC_Channel_Copter::do_aux_function_change_mode(const Mode::Number mode,
                                                     const aux_switch_pos_t ch_flag)
{
    switch(ch_flag) {
    case HIGH: {
        // engage mode (if not possible we remain in current flight mode)
        const bool success = copter.set_mode(mode, ModeReason::RC_COMMAND);
        if (copter.ap.initialised) {
            if (success) {
                AP_Notify::events.user_mode_change = 1;
            } else {
                AP_Notify::events.user_mode_change_failed = 1;
            }
        }
        break;
    }
    default:
        // return to flight mode switch's flight mode if we are currently
        // in this mode
        if (copter.control_mode == mode) {
            rc().reset_mode_switch();
        }
    }
}

void RC_Channel_Copter::do_aux_function_armdisarm(const aux_switch_pos_t ch_flag)
{
    RC_Channel::do_aux_function_armdisarm(ch_flag);
    if (copter.arming.is_armed()) {
        // remember that we are using an arming switch, for use by
        // set_throttle_zero_flag
        copter.ap.armed_with_switch = true;
    }
}

// do_aux_function - implement the function invoked by auxiliary switches
void RC_Channel_Copter::do_aux_function(const aux_func_t ch_option, const aux_switch_pos_t ch_flag)
{
    switch(ch_option) {
        case AUX_FUNC::FLIP:
            // flip if switch is on, positive throttle and we're actually flying
            if (ch_flag == aux_switch_pos_t::HIGH) {
                copter.set_mode(Mode::Number::FLIP, ModeReason::RC_COMMAND);
            }
            break;

        case AUX_FUNC::SIMPLE_MODE:
            // low = simple mode off, middle or high position turns simple mode on
            copter.set_simple_mode(ch_flag == HIGH || ch_flag == MIDDLE);
            break;

        case AUX_FUNC::SUPERSIMPLE_MODE:
            // low = simple mode off, middle = simple mode, high = super simple mode
            copter.set_simple_mode(ch_flag);
            break;

        case AUX_FUNC::RTL:
#if MODE_RTL_ENABLED == ENABLED
            do_aux_function_change_mode(Mode::Number::RTL, ch_flag);
#endif
            break;

        case AUX_FUNC::SAVE_TRIM:
            if ((ch_flag == HIGH) && (copter.control_mode <= Mode::Number::ACRO) && (copter.channel_throttle->get_control_in() == 0)) {
                copter.save_trim();
            }
            break;

        case AUX_FUNC::SAVE_WP:
#if MODE_AUTO_ENABLED == ENABLED
            // save waypoint when switch is brought high
            if (ch_flag == HIGH) {

                // do not allow saving new waypoints while we're in auto or disarmed
                if (copter.control_mode == Mode::Number::AUTO || !copter.motors->armed()) {
                    return;
                }

                // do not allow saving the first waypoint with zero throttle
                if ((copter.mode_auto.mission.num_commands() == 0) && (copter.channel_throttle->get_control_in() == 0)) {
                    return;
                }

                // create new mission command
                AP_Mission::Mission_Command cmd  = {};

                // if the mission is empty save a takeoff command
                if (copter.mode_auto.mission.num_commands() == 0) {
                    // set our location ID to 16, MAV_CMD_NAV_WAYPOINT
                    cmd.id = MAV_CMD_NAV_TAKEOFF;
                    cmd.content.location.alt = MAX(copter.current_loc.alt,100);

                    // use the current altitude for the target alt for takeoff.
                    // only altitude will matter to the AP mission script for takeoff.
                    if (copter.mode_auto.mission.add_cmd(cmd)) {
                        // log event
                        AP::logger().Write_Event(LogEvent::SAVEWP_ADD_WP);
                    }
                }

                // set new waypoint to current location
                cmd.content.location = copter.current_loc;

                // if throttle is above zero, create waypoint command
                if (copter.channel_throttle->get_control_in() > 0) {
                    cmd.id = MAV_CMD_NAV_WAYPOINT;
                } else {
                    // with zero throttle, create LAND command
                    cmd.id = MAV_CMD_NAV_LAND;
                }

                // save command
                if (copter.mode_auto.mission.add_cmd(cmd)) {
                    // log event
                    AP::logger().Write_Event(LogEvent::SAVEWP_ADD_WP);
                }
            }
#endif
            break;

        case AUX_FUNC::AUTO:
#if MODE_AUTO_ENABLED == ENABLED
            do_aux_function_change_mode(Mode::Number::AUTO, ch_flag);
#endif
            break;

        case AUX_FUNC::RANGEFINDER:
            // enable or disable the rangefinder
#if RANGEFINDER_ENABLED == ENABLED
            if ((ch_flag == HIGH) && copter.rangefinder.has_orientation(ROTATION_PITCH_270)) {
                copter.rangefinder_state.enabled = true;
            } else {
                copter.rangefinder_state.enabled = false;
            }
#endif
            break;

        case AUX_FUNC::ACRO_TRAINER:
#if MODE_ACRO_ENABLED == ENABLED
            switch(ch_flag) {
                case LOW:
                    copter.g.acro_trainer = (uint8_t)ModeAcro::Trainer::OFF;
                    AP::logger().Write_Event(LogEvent::ACRO_TRAINER_OFF);
                    break;
                case MIDDLE:
                    copter.g.acro_trainer = (uint8_t)ModeAcro::Trainer::LEVELING;
                    AP::logger().Write_Event(LogEvent::ACRO_TRAINER_LEVELING);
                    break;
                case HIGH:
                    copter.g.acro_trainer = (uint8_t)ModeAcro::Trainer::LIMITED;
                    AP::logger().Write_Event(LogEvent::ACRO_TRAINER_LIMITED);
                    break;
            }
#endif
            break;

        case AUX_FUNC::AUTOTUNE:
#if AUTOTUNE_ENABLED == ENABLED
            do_aux_function_change_mode(Mode::Number::AUTOTUNE, ch_flag);
#endif
            break;

        case AUX_FUNC::LAND:
            do_aux_function_change_mode(Mode::Number::LAND, ch_flag);
            break;

        case AUX_FUNC::GUIDED:
            do_aux_function_change_mode(Mode::Number::GUIDED, ch_flag);
            break;

        case AUX_FUNC::LOITER:
            do_aux_function_change_mode(Mode::Number::LOITER, ch_flag);
            break;

        case AUX_FUNC::FOLLOW:
            do_aux_function_change_mode(Mode::Number::FOLLOW, ch_flag);
            break;

        case AUX_FUNC::PARACHUTE_ENABLE:
#if PARACHUTE == ENABLED
            // Parachute enable/disable
            copter.parachute.enabled(ch_flag == HIGH);
#endif
            break;

        case AUX_FUNC::PARACHUTE_RELEASE:
#if PARACHUTE == ENABLED
            if (ch_flag == HIGH) {
                copter.parachute_manual_release();
            }
#endif
            break;

        case AUX_FUNC::PARACHUTE_3POS:
#if PARACHUTE == ENABLED
            // Parachute disable, enable, release with 3 position switch
            switch (ch_flag) {
                case LOW:
                    copter.parachute.enabled(false);
                    AP::logger().Write_Event(LogEvent::PARACHUTE_DISABLED);
                    break;
                case MIDDLE:
                    copter.parachute.enabled(true);
                    AP::logger().Write_Event(LogEvent::PARACHUTE_ENABLED);
                    break;
                case HIGH:
                    copter.parachute.enabled(true);
                    copter.parachute_manual_release();
                    break;
            }
#endif
            break;

        case AUX_FUNC::ATTCON_FEEDFWD:
            // enable or disable feed forward
            copter.attitude_control->bf_feedforward(ch_flag == HIGH);
            break;

        case AUX_FUNC::ATTCON_ACCEL_LIM:
            // enable or disable accel limiting by restoring defaults
            copter.attitude_control->accel_limiting(ch_flag == HIGH);
            break;

        case AUX_FUNC::RETRACT_MOUNT:
#if MOUNT == ENABLE
            switch (ch_flag) {
                case HIGH:
                    copter.camera_mount.set_mode(MAV_MOUNT_MODE_RETRACT);
                    break;
                case MIDDLE:
                    // nothing
                    break;
                case LOW:
                    copter.camera_mount.set_mode_to_default();
                    break;
            }
#endif
            break;

        case AUX_FUNC::MOTOR_INTERLOCK:
#if FRAME_CONFIG == HELI_FRAME
            // The interlock logic for ROTOR_CONTROL_MODE_SPEED_PASSTHROUGH is handled 
            // in heli_update_rotor_speed_targets.  Otherwise turn on when above low.
            if (copter.motors->get_rsc_mode() != ROTOR_CONTROL_MODE_SPEED_PASSTHROUGH) {
                copter.ap.motor_interlock_switch = (ch_flag == HIGH || ch_flag == MIDDLE);
            }
#else
            copter.ap.motor_interlock_switch = (ch_flag == HIGH || ch_flag == MIDDLE);
#endif
            break;

        case AUX_FUNC::BRAKE:
#if MODE_BRAKE_ENABLED == ENABLED
            do_aux_function_change_mode(Mode::Number::BRAKE, ch_flag);
#endif
            break;

        case AUX_FUNC::THROW:
#if MODE_THROW_ENABLED == ENABLED
            do_aux_function_change_mode(Mode::Number::THROW, ch_flag);
#endif
            break;

        case AUX_FUNC::PRECISION_LOITER:
#if PRECISION_LANDING == ENABLED && MODE_LOITER_ENABLED == ENABLED
            switch (ch_flag) {
                case HIGH:
                    copter.mode_loiter.set_precision_loiter_enabled(true);
                    break;
                case MIDDLE:
                    // nothing
                    break;
                case LOW:
                    copter.mode_loiter.set_precision_loiter_enabled(false);
                    break;
            }
#endif
            break;

        case AUX_FUNC::SMART_RTL:
#if MODE_SMARTRTL_ENABLED == ENABLED
            do_aux_function_change_mode(Mode::Number::SMART_RTL, ch_flag);
#endif
            break;

        case AUX_FUNC::INVERTED:
#if FRAME_CONFIG == HELI_FRAME
            switch (ch_flag) {
            case HIGH:
                copter.motors->set_inverted_flight(true);
                copter.attitude_control->set_inverted_flight(true);
                copter.heli_flags.inverted_flight = true;
                break;
            case MIDDLE:
                // nothing
                break;
            case LOW:
                copter.motors->set_inverted_flight(false);
                copter.attitude_control->set_inverted_flight(false);
                copter.heli_flags.inverted_flight = false;
                break;
            }
#endif
            break;

        case AUX_FUNC::WINCH_ENABLE:
#if WINCH_ENABLED == ENABLED
            switch (ch_flag) {
                case HIGH:
                    // high switch maintains current position
                    copter.g2.winch.release_length(0.0f);
                    AP::logger().Write_Event(LogEvent::WINCH_LENGTH_CONTROL);
                    break;
                default:
                    // all other position relax winch
                    copter.g2.winch.relax();
                    AP::logger().Write_Event(LogEvent::WINCH_RELAXED);
                    break;
                }
#endif
            break;

        case AUX_FUNC::WINCH_CONTROL:
#if WINCH_ENABLED == ENABLED
            switch (ch_flag) {
                case LOW:
                    // raise winch at maximum speed
                    copter.g2.winch.set_desired_rate(-copter.g2.winch.get_rate_max());
                    break;
                case HIGH:
                    // lower winch at maximum speed
                    copter.g2.winch.set_desired_rate(copter.g2.winch.get_rate_max());
                    break;
                case MIDDLE:
                    copter.g2.winch.set_desired_rate(0.0f);
                    break;
                }
#endif
            break;

#ifdef USERHOOK_AUXSWITCH
        case AUX_FUNC::USER_FUNC1:
            copter.userhook_auxSwitch1(ch_flag);
            break;

        case AUX_FUNC::USER_FUNC2:
            copter.userhook_auxSwitch2(ch_flag);
            break;

        case AUX_FUNC::USER_FUNC3:
            copter.userhook_auxSwitch3(ch_flag);
            break;
#endif

        case AUX_FUNC::ZIGZAG:
#if MODE_ZIGZAG_ENABLED == ENABLED
            do_aux_function_change_mode(Mode::Number::ZIGZAG, ch_flag);
#endif
            break;

        case AUX_FUNC::ZIGZAG_SaveWP:
#if MODE_ZIGZAG_ENABLED == ENABLED
            if (copter.flightmode == &copter.mode_zigzag) {
                switch (ch_flag) {
                    case LOW:
                        copter.mode_zigzag.save_or_move_to_destination(0);
                        break;
                    case MIDDLE:
                        copter.mode_zigzag.return_to_manual_control(false);
                        break;
                    case HIGH:
                        copter.mode_zigzag.save_or_move_to_destination(1);
                        break;
                }
            }
#endif
            break;

        case AUX_FUNC::STABILIZE:
            do_aux_function_change_mode(Mode::Number::STABILIZE, ch_flag);
            break;

        case AUX_FUNC::POSHOLD:
#if MODE_POSHOLD_ENABLED == ENABLED
            do_aux_function_change_mode(Mode::Number::POSHOLD, ch_flag);
#endif
            break;

        case AUX_FUNC::ALTHOLD:
            do_aux_function_change_mode(Mode::Number::ALT_HOLD, ch_flag);
            break;

	case AUX_FUNC::FLOWHOLD:
#if OPTFLOW == ENABLED
            do_aux_function_change_mode(Mode::Number::FLOWHOLD, ch_flag);
#endif
            break;

        case AUX_FUNC::CIRCLE:
#if MODE_CIRCLE_ENABLED == ENABLED
            do_aux_function_change_mode(Mode::Number::CIRCLE, ch_flag);
#endif
            break;

        case AUX_FUNC::DRIFT:
#if MODE_DRIFT_ENABLED == ENABLED
            do_aux_function_change_mode(Mode::Number::DRIFT, ch_flag);
#endif
            break;

        case AUX_FUNC::STANDBY: {
            switch (ch_flag) {
                case HIGH:
                    copter.standby_active = true;
                    AP::logger().Write_Event(LogEvent::STANDBY_ENABLE);
                    gcs().send_text(MAV_SEVERITY_INFO, "Stand By Enabled");
                    break;
                default:
                    copter.standby_active = false;
                    AP::logger().Write_Event(LogEvent::STANDBY_DISABLE);
                    gcs().send_text(MAV_SEVERITY_INFO, "Stand By Disabled");
                    break;
                }
            break;
        }

        case AUX_FUNC::SURFACE_TRACKING:
            switch (ch_flag) {
            case LOW:
                copter.surface_tracking.set_surface(Copter::SurfaceTracking::Surface::GROUND);
                break;
            case MIDDLE:
                copter.surface_tracking.set_surface(Copter::SurfaceTracking::Surface::NONE);
                break;
            case HIGH:
                copter.surface_tracking.set_surface(Copter::SurfaceTracking::Surface::CEILING);
                break;
            }
            break;

    default:
        RC_Channel::do_aux_function(ch_option, ch_flag);
        break;
    }
}

// save_trim - adds roll and pitch trims from the radio to ahrs
void Copter::save_trim()
{
    // save roll and pitch trim
    float roll_trim = ToRad((float)channel_roll->get_control_in()/100.0f);
    float pitch_trim = ToRad((float)channel_pitch->get_control_in()/100.0f);
    ahrs.add_trim(roll_trim, pitch_trim);
    AP::logger().Write_Event(LogEvent::SAVE_TRIM);
    gcs().send_text(MAV_SEVERITY_INFO, "Trim saved");
}

// auto_trim - slightly adjusts the ahrs.roll_trim and ahrs.pitch_trim towards the current stick positions
// meant to be called continuously while the pilot attempts to keep the copter level
void Copter::auto_trim()
{
    if (auto_trim_counter > 0) {
        auto_trim_counter--;

        // flash the leds
        AP_Notify::flags.save_trim = true;

        // calculate roll trim adjustment
        float roll_trim_adjustment = ToRad((float)channel_roll->get_control_in() / 4000.0f);

        // calculate pitch trim adjustment
        float pitch_trim_adjustment = ToRad((float)channel_pitch->get_control_in() / 4000.0f);

        // add trim to ahrs object
        // save to eeprom on last iteration
        ahrs.add_trim(roll_trim_adjustment, pitch_trim_adjustment, (auto_trim_counter == 0));

        // on last iteration restore leds and accel gains to normal
        if (auto_trim_counter == 0) {
            AP_Notify::flags.save_trim = false;
        }
    }
}
