#include "Copter.h"

#include "AP_AuxFunc.h"

// init_aux_switch_function - initialize aux functions
bool AP_AuxFunc_Copter::init_function(const Function function,
                                      const SwitchPos pos)
{
    // init channel options
    switch(function) {
    // the following functions do not need to be initialised:
    case Function::ALTHOLD:
    case Function::AUTO:
    case Function::AUTOTUNE:
    case Function::BRAKE:
    case Function::CIRCLE:
    case Function::DRIFT:
    case Function::FLIP:
    case Function::FLOWHOLD:
    case Function::FOLLOW:
    case Function::GUIDED:
    case Function::LAND:
    case Function::LOITER:
    case Function::PARACHUTE_RELEASE:
    case Function::POSHOLD:
    case Function::RESETTOARMEDYAW:
    case Function::RTL:
    case Function::SAVE_TRIM:
    case Function::SAVE_WP:
    case Function::SMART_RTL:
    case Function::STABILIZE:
    case Function::THROW:
    case Function::USER_FUNC1:
    case Function::USER_FUNC2:
    case Function::USER_FUNC3:
    case Function::WINCH_CONTROL:
    case Function::ZIGZAG:
    case Function::ZIGZAG_Auto:
    case Function::ZIGZAG_SaveWP:
    case Function::ACRO:
        return true;
    case Function::ACRO_TRAINER:
    case Function::ATTCON_ACCEL_LIM:
    case Function::ATTCON_FEEDFWD:
    case Function::INVERTED:
    case Function::MOTOR_INTERLOCK:
    case Function::PARACHUTE_3POS:      // we trust the vehicle will be disarmed so even if switch is in release position the chute will not release
    case Function::PARACHUTE_ENABLE:
    case Function::PRECISION_LOITER:
    case Function::RANGEFINDER:
    case Function::SIMPLE_MODE:
    case Function::STANDBY:
    case Function::SUPERSIMPLE_MODE:
    case Function::SURFACE_TRACKING:
    case Function::WINCH_ENABLE:
    case Function::AIRMODE:
        return run_function(function, pos, TriggerSource::INIT);
    default:
        return AP_AuxFunc::init_function(function, pos);
    }
}

// do_function_change_mode - change mode based on an aux switch
// being moved
void AP_AuxFunc_Copter::do_function_change_mode(const Mode::Number mode,
                                                const SwitchPos pos)
{
    switch(pos) {
    case SwitchPos::HIGH: {
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
        if (copter.flightmode->mode_number() == mode) {
            rc().reset_mode_switch();
        }
    }
}

void AP_AuxFunc_Copter::do_function_armdisarm(const SwitchPos pos)
{
    AP_AuxFunc::do_function_armdisarm(pos);
    if (copter.arming.is_armed()) {
        // remember that we are using an arming switch, for use by
        // set_throttle_zero_flag
        copter.ap.armed_with_switch = true;
    }
}

// do_function - implement the function invoked by auxiliary switches
bool AP_AuxFunc_Copter::do_function(const Function function, const SwitchPos pos)
{
    switch(function) {
        case Function::FLIP:
            // flip if switch is on, positive throttle and we're actually flying
            if (pos == SwitchPos::HIGH) {
                copter.set_mode(Mode::Number::FLIP, ModeReason::RC_COMMAND);
            }
            break;

        case Function::SIMPLE_MODE:
            // low = simple mode off, middle or high position turns simple mode on
            copter.set_simple_mode((pos == SwitchPos::LOW) ? Copter::SimpleMode::NONE : Copter::SimpleMode::SIMPLE);
            break;

        case Function::SUPERSIMPLE_MODE: {
            Copter::SimpleMode newmode = Copter::SimpleMode::NONE;
            switch (pos) {
            case SwitchPos::LOW:
                break;
            case SwitchPos::MIDDLE:
                newmode = Copter::SimpleMode::SIMPLE;
                break;
            case SwitchPos::HIGH:
                newmode = Copter::SimpleMode::SUPERSIMPLE;
                break;
            }
            copter.set_simple_mode(newmode);
            break;
        }

        case Function::RTL:
#if MODE_RTL_ENABLED == ENABLED
            do_function_change_mode(Mode::Number::RTL, pos);
#endif
            break;

        case Function::SAVE_TRIM:
            if ((pos == SwitchPos::HIGH) &&
                (copter.flightmode->allows_save_trim()) &&
                (copter.channel_throttle->get_control_in() == 0)) {
                copter.save_trim();
            }
            break;

        case Function::SAVE_WP:
#if MODE_AUTO_ENABLED == ENABLED
            // save waypoint when switch is brought high
            if (pos == SwitchPos::HIGH) {

                // do not allow saving new waypoints while we're in auto or disarmed
                if (copter.flightmode->mode_number() == Mode::Number::AUTO || !copter.motors->armed()) {
                    break;
                }

                // do not allow saving the first waypoint with zero throttle
                if ((copter.mode_auto.mission.num_commands() == 0) && (copter.channel_throttle->get_control_in() == 0)) {
                    break;
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

        case Function::AUTO:
#if MODE_AUTO_ENABLED == ENABLED
            do_function_change_mode(Mode::Number::AUTO, pos);
#endif
            break;

        case Function::RANGEFINDER:
            // enable or disable the rangefinder
#if RANGEFINDER_ENABLED == ENABLED
            if ((pos == SwitchPos::HIGH) &&
                copter.rangefinder.has_orientation(ROTATION_PITCH_270)) {
                copter.rangefinder_state.enabled = true;
            } else {
                copter.rangefinder_state.enabled = false;
            }
#endif
            break;

        case Function::ACRO_TRAINER:
#if MODE_ACRO_ENABLED == ENABLED
            switch(pos) {
                case SwitchPos::LOW:
                    copter.g.acro_trainer = (uint8_t)ModeAcro::Trainer::OFF;
                    AP::logger().Write_Event(LogEvent::ACRO_TRAINER_OFF);
                    break;
                case SwitchPos::MIDDLE:
                    copter.g.acro_trainer = (uint8_t)ModeAcro::Trainer::LEVELING;
                    AP::logger().Write_Event(LogEvent::ACRO_TRAINER_LEVELING);
                    break;
                case SwitchPos::HIGH:
                    copter.g.acro_trainer = (uint8_t)ModeAcro::Trainer::LIMITED;
                    AP::logger().Write_Event(LogEvent::ACRO_TRAINER_LIMITED);
                    break;
            }
#endif
            break;

        case Function::AUTOTUNE:
#if AUTOTUNE_ENABLED == ENABLED
            do_function_change_mode(Mode::Number::AUTOTUNE, pos);
#endif
            break;

        case Function::LAND:
            do_function_change_mode(Mode::Number::LAND, pos);
            break;

        case Function::GUIDED:
            do_function_change_mode(Mode::Number::GUIDED, pos);
            break;

        case Function::LOITER:
            do_function_change_mode(Mode::Number::LOITER, pos);
            break;

        case Function::FOLLOW:
            do_function_change_mode(Mode::Number::FOLLOW, pos);
            break;

        case Function::PARACHUTE_ENABLE:
#if PARACHUTE == ENABLED
            // Parachute enable/disable
            copter.parachute.enabled(pos == SwitchPos::HIGH);
#endif
            break;

        case Function::PARACHUTE_RELEASE:
#if PARACHUTE == ENABLED
            if (pos == SwitchPos::HIGH) {
                copter.parachute_manual_release();
            }
#endif
            break;

        case Function::PARACHUTE_3POS:
#if PARACHUTE == ENABLED
            // Parachute disable, enable, release with 3 position switch
            switch (pos) {
                case SwitchPos::LOW:
                    copter.parachute.enabled(false);
                    AP::logger().Write_Event(LogEvent::PARACHUTE_DISABLED);
                    break;
                case SwitchPos::MIDDLE:
                    copter.parachute.enabled(true);
                    AP::logger().Write_Event(LogEvent::PARACHUTE_ENABLED);
                    break;
                case SwitchPos::HIGH:
                    copter.parachute.enabled(true);
                    copter.parachute_manual_release();
                    break;
            }
#endif
            break;

        case Function::ATTCON_FEEDFWD:
            // enable or disable feed forward
            copter.attitude_control->bf_feedforward(pos == SwitchPos::HIGH);
            break;

        case Function::ATTCON_ACCEL_LIM:
            // enable or disable accel limiting by restoring defaults
            copter.attitude_control->accel_limiting(pos == SwitchPos::HIGH);
            break;

        case Function::MOTOR_INTERLOCK:
#if FRAME_CONFIG == HELI_FRAME
            // The interlock logic for ROTOR_CONTROL_MODE_SPEED_PASSTHROUGH is handled 
            // in heli_update_rotor_speed_targets.  Otherwise turn on when above low.
            if (copter.motors->get_rsc_mode() != ROTOR_CONTROL_MODE_SPEED_PASSTHROUGH) {
                copter.ap.motor_interlock_switch = (pos == SwitchPos::HIGH || pos == SwitchPos::MIDDLE);
            }
#else
            copter.ap.motor_interlock_switch = (pos == SwitchPos::HIGH || pos == SwitchPos::MIDDLE);
#endif
            break;

        case Function::BRAKE:
#if MODE_BRAKE_ENABLED == ENABLED
            do_function_change_mode(Mode::Number::BRAKE, pos);
#endif
            break;

        case Function::THROW:
#if MODE_THROW_ENABLED == ENABLED
            do_function_change_mode(Mode::Number::THROW, pos);
#endif
            break;

        case Function::PRECISION_LOITER:
#if PRECISION_LANDING == ENABLED && MODE_LOITER_ENABLED == ENABLED
            switch (pos) {
                case SwitchPos::HIGH:
                    copter.mode_loiter.set_precision_loiter_enabled(true);
                    break;
                case SwitchPos::MIDDLE:
                    // nothing
                    break;
                case SwitchPos::LOW:
                    copter.mode_loiter.set_precision_loiter_enabled(false);
                    break;
            }
#endif
            break;

        case Function::SMART_RTL:
#if MODE_SMARTRTL_ENABLED == ENABLED
            do_function_change_mode(Mode::Number::SMART_RTL, pos);
#endif
            break;

        case Function::INVERTED:
#if FRAME_CONFIG == HELI_FRAME
            switch (pos) {
            case SwitchPos::HIGH:
                copter.motors->set_inverted_flight(true);
                copter.attitude_control->set_inverted_flight(true);
                copter.heli_flags.inverted_flight = true;
                break;
            case SwitchPos::MIDDLE:
                // nothing
                break;
            case SwitchPos::LOW:
                copter.motors->set_inverted_flight(false);
                copter.attitude_control->set_inverted_flight(false);
                copter.heli_flags.inverted_flight = false;
                break;
            }
#endif
            break;

        case Function::WINCH_ENABLE:
#if WINCH_ENABLED == ENABLED
            switch (pos) {
                case SwitchPos::HIGH:
                    // high switch position stops winch using rate control
                    copter.g2.winch.set_desired_rate(0.0f);
                    break;
                case SwitchPos::MIDDLE:
                case SwitchPos::LOW:
                    // all other position relax winch
                    copter.g2.winch.relax();
                    break;
                }
#endif
            break;

        case Function::WINCH_CONTROL:
            // do nothing, used to control the rate of the winch and is processed within AP_Winch
            break;

#ifdef USERHOOK_AUXSWITCH
        case Function::USER_FUNC1:
            copter.userhook_auxSwitch1(pos);
            break;

        case Function::USER_FUNC2:
            copter.userhook_auxSwitch2(pos);
            break;

        case Function::USER_FUNC3:
            copter.userhook_auxSwitch3(pos);
            break;
#endif

        case Function::ZIGZAG:
#if MODE_ZIGZAG_ENABLED == ENABLED
            do_function_change_mode(Mode::Number::ZIGZAG, pos);
#endif
            break;

        case Function::ZIGZAG_SaveWP:
#if MODE_ZIGZAG_ENABLED == ENABLED
            if (copter.flightmode == &copter.mode_zigzag) {
                // initialize zigzag auto
                copter.mode_zigzag.init_auto();
                switch (pos) {
                    case SwitchPos::LOW:
                        copter.mode_zigzag.save_or_move_to_destination(ModeZigZag::Destination::A);
                        break;
                    case SwitchPos::MIDDLE:
                        copter.mode_zigzag.return_to_manual_control(false);
                        break;
                    case SwitchPos::HIGH:
                        copter.mode_zigzag.save_or_move_to_destination(ModeZigZag::Destination::B);
                        break;
                }
            }
#endif
            break;

        case Function::STABILIZE:
            do_function_change_mode(Mode::Number::STABILIZE, pos);
            break;

        case Function::POSHOLD:
#if MODE_POSHOLD_ENABLED == ENABLED
            do_function_change_mode(Mode::Number::POSHOLD, pos);
#endif
            break;

        case Function::ALTHOLD:
            do_function_change_mode(Mode::Number::ALT_HOLD, pos);
            break;


        case Function::ACRO:
#if MODE_ACRO_ENABLED == ENABLED
            do_function_change_mode(Mode::Number::ACRO, pos);
#endif
            break;

        case Function::FLOWHOLD:
#if OPTFLOW == ENABLED
            do_function_change_mode(Mode::Number::FLOWHOLD, pos);
#endif
            break;

        case Function::CIRCLE:
#if MODE_CIRCLE_ENABLED == ENABLED
            do_function_change_mode(Mode::Number::CIRCLE, pos);
#endif
            break;

        case Function::DRIFT:
#if MODE_DRIFT_ENABLED == ENABLED
            do_function_change_mode(Mode::Number::DRIFT, pos);
#endif
            break;

        case Function::STANDBY: {
            switch (pos) {
                case SwitchPos::HIGH:
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

        case Function::SURFACE_TRACKING:
            switch (pos) {
            case SwitchPos::LOW:
                copter.surface_tracking.set_surface(Copter::SurfaceTracking::Surface::GROUND);
                break;
            case SwitchPos::MIDDLE:
                copter.surface_tracking.set_surface(Copter::SurfaceTracking::Surface::NONE);
                break;
            case SwitchPos::HIGH:
                copter.surface_tracking.set_surface(Copter::SurfaceTracking::Surface::CEILING);
                break;
            }
            break;

        case Function::ZIGZAG_Auto:
#if MODE_ZIGZAG_ENABLED == ENABLED
            if (copter.flightmode == &copter.mode_zigzag) {
                switch (pos) {
                case SwitchPos::HIGH:
                    copter.mode_zigzag.run_auto();
                    break;
                default:
                    copter.mode_zigzag.suspend_auto();
                    break;
                }
            }
#endif
            break;

        case Function::AIRMODE:
            do_function_change_air_mode(pos);
#if MODE_ACRO_ENABLED == ENABLED && FRAME_CONFIG != HELI_FRAME
            copter.mode_acro.air_mode_aux_changed();
#endif
            break;

    default:
        return AP_AuxFunc::do_function(function, pos);
    }
    return true;
}

// change air-mode status
void AP_AuxFunc_Copter::do_function_change_air_mode(const SwitchPos pos)
{
    switch (pos) {
    case SwitchPos::HIGH:
        copter.air_mode = AirMode::AIRMODE_ENABLED;
        break;
    case SwitchPos::MIDDLE:
        break;
    case SwitchPos::LOW:
        copter.air_mode = AirMode::AIRMODE_DISABLED;
        break;
    }
}
