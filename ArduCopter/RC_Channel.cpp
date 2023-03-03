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
        return;
    }

    if (!rc().find_channel_for_option(AUX_FUNC::SIMPLE_MODE) &&
        !rc().find_channel_for_option(AUX_FUNC::SUPERSIMPLE_MODE)) {
        // if none of the Aux Switches are set to Simple or Super Simple Mode then
        // set Simple Mode using stored parameters from EEPROM
        if (BIT_IS_SET(copter.g.super_simple, new_pos)) {
            copter.set_simple_mode(Copter::SimpleMode::SUPERSIMPLE);
        } else {
            copter.set_simple_mode(BIT_IS_SET(copter.g.simple_modes, new_pos) ? Copter::SimpleMode::SIMPLE : Copter::SimpleMode::NONE);
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

// returns true if throttle arming checks should be run
bool RC_Channels_Copter::arming_check_throttle() const {
    if ((copter.g.throttle_behavior & THR_BEHAVE_FEEDBACK_FROM_MID_STICK) != 0) {
        // center sprung throttle configured, dont run AP_Arming check
        // Copter already checks this case in its own arming checks
        return false;
    }
    return RC_Channels::arming_check_throttle();
}

RC_Channel * RC_Channels_Copter::get_arming_channel(void) const
{
    return copter.channel_yaw;
}

// init_aux_switch_function - initialize aux functions
void RC_Channel_Copter::init_aux_function(const aux_func_t ch_option, const AuxSwitchPos ch_flag)
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
    case AUX_FUNC::ZIGZAG_Auto:
    case AUX_FUNC::ZIGZAG_SaveWP:
    case AUX_FUNC::ACRO:
    case AUX_FUNC::AUTO_RTL:
    case AUX_FUNC::TURTLE:
    case AUX_FUNC::SIMPLE_HEADING_RESET:
    case AUX_FUNC::ARMDISARM_AIRMODE:
    case AUX_FUNC::TURBINE_START:
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
    case AUX_FUNC::SIMPLE_MODE:
    case AUX_FUNC::STANDBY:
    case AUX_FUNC::SUPERSIMPLE_MODE:
    case AUX_FUNC::SURFACE_TRACKING:
    case AUX_FUNC::WINCH_ENABLE:
    case AUX_FUNC::AIRMODE:
    case AUX_FUNC::FORCEFLYING:
    case AUX_FUNC::CUSTOM_CONTROLLER:
    case AUX_FUNC::WEATHER_VANE_ENABLE:
        run_aux_function(ch_option, ch_flag, AuxFuncTriggerSource::INIT);
        break;
    default:
        RC_Channel::init_aux_function(ch_option, ch_flag);
        break;
    }
}

// do_aux_function_change_mode - change mode based on an aux switch
// being moved
void RC_Channel_Copter::do_aux_function_change_mode(const Mode::Number mode,
                                                    const AuxSwitchPos ch_flag)
{
    switch(ch_flag) {
    case AuxSwitchPos::HIGH: {
        // engage mode (if not possible we remain in current flight mode)
        copter.set_mode(mode, ModeReason::RC_COMMAND);
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

// do_aux_function - implement the function invoked by auxiliary switches
bool RC_Channel_Copter::do_aux_function(const aux_func_t ch_option, const AuxSwitchPos ch_flag)
{
    switch(ch_option) {
        case AUX_FUNC::FLIP:
            // flip if switch is on, positive throttle and we're actually flying
            if (ch_flag == AuxSwitchPos::HIGH) {
                copter.set_mode(Mode::Number::FLIP, ModeReason::RC_COMMAND);
            }
            break;

        case AUX_FUNC::SIMPLE_MODE:
            // low = simple mode off, middle or high position turns simple mode on
            copter.set_simple_mode((ch_flag == AuxSwitchPos::LOW) ? Copter::SimpleMode::NONE : Copter::SimpleMode::SIMPLE);
            break;

        case AUX_FUNC::SUPERSIMPLE_MODE: {
            Copter::SimpleMode newmode = Copter::SimpleMode::NONE;
            switch (ch_flag) {
            case AuxSwitchPos::LOW:
                break;
            case AuxSwitchPos::MIDDLE:
                newmode = Copter::SimpleMode::SIMPLE;
                break;
            case AuxSwitchPos::HIGH:
                newmode = Copter::SimpleMode::SUPERSIMPLE;
                break;
            }
            copter.set_simple_mode(newmode);
            break;
        }

        case AUX_FUNC::RTL:
#if MODE_RTL_ENABLED == ENABLED
            do_aux_function_change_mode(Mode::Number::RTL, ch_flag);
#endif
            break;

        case AUX_FUNC::SAVE_TRIM:
            if ((ch_flag == AuxSwitchPos::HIGH) &&
                (copter.flightmode->allows_save_trim()) &&
                (copter.channel_throttle->get_control_in() == 0)) {
                copter.save_trim();
            }
            break;

        case AUX_FUNC::SAVE_WP:
#if MODE_AUTO_ENABLED == ENABLED
            // save waypoint when switch is brought high
            if (ch_flag == RC_Channel::AuxSwitchPos::HIGH) {

                // do not allow saving new waypoints while we're in auto or disarmed
                if (copter.flightmode == &copter.mode_auto || !copter.motors->armed()) {
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

        case AUX_FUNC::AUTO:
#if MODE_AUTO_ENABLED == ENABLED
            do_aux_function_change_mode(Mode::Number::AUTO, ch_flag);
#endif
            break;

        case AUX_FUNC::RANGEFINDER:
            // enable or disable the rangefinder
#if RANGEFINDER_ENABLED == ENABLED
            if ((ch_flag == AuxSwitchPos::HIGH) &&
                copter.rangefinder.has_orientation(ROTATION_PITCH_270)) {
                copter.rangefinder_state.enabled = true;
            } else {
                copter.rangefinder_state.enabled = false;
            }
#endif
            break;

        case AUX_FUNC::ACRO_TRAINER:
#if MODE_ACRO_ENABLED == ENABLED
            switch(ch_flag) {
                case AuxSwitchPos::LOW:
                    copter.g.acro_trainer.set((uint8_t)ModeAcro::Trainer::OFF);
                    AP::logger().Write_Event(LogEvent::ACRO_TRAINER_OFF);
                    break;
                case AuxSwitchPos::MIDDLE:
                    copter.g.acro_trainer.set((uint8_t)ModeAcro::Trainer::LEVELING);
                    AP::logger().Write_Event(LogEvent::ACRO_TRAINER_LEVELING);
                    break;
                case AuxSwitchPos::HIGH:
                    copter.g.acro_trainer.set((uint8_t)ModeAcro::Trainer::LIMITED);
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
            copter.parachute.enabled(ch_flag == AuxSwitchPos::HIGH);
#endif
            break;

        case AUX_FUNC::PARACHUTE_RELEASE:
#if PARACHUTE == ENABLED
            if (ch_flag == AuxSwitchPos::HIGH) {
                copter.parachute_manual_release();
            }
#endif
            break;

        case AUX_FUNC::PARACHUTE_3POS:
#if PARACHUTE == ENABLED
            // Parachute disable, enable, release with 3 position switch
            switch (ch_flag) {
                case AuxSwitchPos::LOW:
                    copter.parachute.enabled(false);
                    break;
                case AuxSwitchPos::MIDDLE:
                    copter.parachute.enabled(true);
                    break;
                case AuxSwitchPos::HIGH:
                    copter.parachute.enabled(true);
                    copter.parachute_manual_release();
                    break;
            }
#endif
            break;

        case AUX_FUNC::ATTCON_FEEDFWD:
            // enable or disable feed forward
            copter.attitude_control->bf_feedforward(ch_flag == AuxSwitchPos::HIGH);
            break;

        case AUX_FUNC::ATTCON_ACCEL_LIM:
            // enable or disable accel limiting by restoring defaults
            copter.attitude_control->accel_limiting(ch_flag == AuxSwitchPos::HIGH);
            break;

        case AUX_FUNC::MOTOR_INTERLOCK:
#if FRAME_CONFIG == HELI_FRAME
            // The interlock logic for ROTOR_CONTROL_MODE_PASSTHROUGH is handled 
            // in heli_update_rotor_speed_targets.  Otherwise turn on when above low.
            if (copter.motors->get_rsc_mode() != ROTOR_CONTROL_MODE_PASSTHROUGH) {
                copter.ap.motor_interlock_switch = (ch_flag == AuxSwitchPos::HIGH || ch_flag == AuxSwitchPos::MIDDLE);
            }
#else
            copter.ap.motor_interlock_switch = (ch_flag == AuxSwitchPos::HIGH || ch_flag == AuxSwitchPos::MIDDLE);
#endif
            break;
			
        case AUX_FUNC::TURBINE_START:
#if FRAME_CONFIG == HELI_FRAME     
           switch (ch_flag) {
                case AuxSwitchPos::HIGH:
                    copter.motors->set_turb_start(true);
                    break;
                case AuxSwitchPos::MIDDLE:
                    // nothing
                    break;
                case AuxSwitchPos::LOW:
                    copter.motors->set_turb_start(false);
                    break;
           }
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
                case AuxSwitchPos::HIGH:
                    copter.mode_loiter.set_precision_loiter_enabled(true);
                    break;
                case AuxSwitchPos::MIDDLE:
                    // nothing
                    break;
                case AuxSwitchPos::LOW:
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
            case AuxSwitchPos::HIGH:
                copter.motors->set_inverted_flight(true);
                copter.attitude_control->set_inverted_flight(true);
                copter.heli_flags.inverted_flight = true;
                break;
            case AuxSwitchPos::MIDDLE:
                // nothing
                break;
            case AuxSwitchPos::LOW:
                copter.motors->set_inverted_flight(false);
                copter.attitude_control->set_inverted_flight(false);
                copter.heli_flags.inverted_flight = false;
                break;
            }
#endif
            break;

        case AUX_FUNC::WINCH_ENABLE:
#if AP_WINCH_ENABLED
            switch (ch_flag) {
                case AuxSwitchPos::HIGH:
                    // high switch position stops winch using rate control
                    copter.g2.winch.set_desired_rate(0.0f);
                    break;
                case AuxSwitchPos::MIDDLE:
                case AuxSwitchPos::LOW:
                    // all other position relax winch
                    copter.g2.winch.relax();
                    break;
                }
#endif
            break;

        case AUX_FUNC::WINCH_CONTROL:
            // do nothing, used to control the rate of the winch and is processed within AP_Winch
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
                // initialize zigzag auto
                copter.mode_zigzag.init_auto();
                switch (ch_flag) {
                    case AuxSwitchPos::LOW:
                        copter.mode_zigzag.save_or_move_to_destination(ModeZigZag::Destination::A);
                        break;
                    case AuxSwitchPos::MIDDLE:
                        copter.mode_zigzag.return_to_manual_control(false);
                        break;
                    case AuxSwitchPos::HIGH:
                        copter.mode_zigzag.save_or_move_to_destination(ModeZigZag::Destination::B);
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


        case AUX_FUNC::ACRO:
#if MODE_ACRO_ENABLED == ENABLED
            do_aux_function_change_mode(Mode::Number::ACRO, ch_flag);
#endif
            break;

        case AUX_FUNC::FLOWHOLD:
#if MODE_FLOWHOLD_ENABLED
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
                case AuxSwitchPos::HIGH:
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
            case AuxSwitchPos::LOW:
                copter.surface_tracking.set_surface(Copter::SurfaceTracking::Surface::GROUND);
                break;
            case AuxSwitchPos::MIDDLE:
                copter.surface_tracking.set_surface(Copter::SurfaceTracking::Surface::NONE);
                break;
            case AuxSwitchPos::HIGH:
                copter.surface_tracking.set_surface(Copter::SurfaceTracking::Surface::CEILING);
                break;
            }
            break;

        case AUX_FUNC::ZIGZAG_Auto:
#if MODE_ZIGZAG_ENABLED == ENABLED
            if (copter.flightmode == &copter.mode_zigzag) {
                switch (ch_flag) {
                case AuxSwitchPos::HIGH:
                    copter.mode_zigzag.run_auto();
                    break;
                default:
                    copter.mode_zigzag.suspend_auto();
                    break;
                }
            }
#endif
            break;

        case AUX_FUNC::AIRMODE:
            do_aux_function_change_air_mode(ch_flag);
#if MODE_ACRO_ENABLED == ENABLED && FRAME_CONFIG != HELI_FRAME
            copter.mode_acro.air_mode_aux_changed();
#endif
            break;

        case AUX_FUNC::FORCEFLYING:
            do_aux_function_change_force_flying(ch_flag);
            break;

        case AUX_FUNC::AUTO_RTL:
#if MODE_AUTO_ENABLED == ENABLED
            do_aux_function_change_mode(Mode::Number::AUTO_RTL, ch_flag);
#endif
            break;

        case AUX_FUNC::TURTLE:
#if MODE_TURTLE_ENABLED == ENABLED
            do_aux_function_change_mode(Mode::Number::TURTLE, ch_flag);
#endif
            break;

        case AUX_FUNC::SIMPLE_HEADING_RESET:
            if (ch_flag == AuxSwitchPos::HIGH) {
                copter.init_simple_bearing();
                gcs().send_text(MAV_SEVERITY_INFO, "Simple heading reset");
            }
            break;

        case AUX_FUNC::ARMDISARM_AIRMODE:
            RC_Channel::do_aux_function_armdisarm(ch_flag);
            if (copter.arming.is_armed()) {
                copter.ap.armed_with_airmode_switch = true;
            }
            break;

#if AC_CUSTOMCONTROL_MULTI_ENABLED == ENABLED
        case AUX_FUNC::CUSTOM_CONTROLLER:
            copter.custom_control.set_custom_controller(ch_flag == AuxSwitchPos::HIGH);
            break;
#endif

#if WEATHERVANE_ENABLED == ENABLED
    case AUX_FUNC::WEATHER_VANE_ENABLE: {
        switch (ch_flag) {
            case AuxSwitchPos::HIGH:
                copter.g2.weathervane.allow_weathervaning(true);
                break;
            case AuxSwitchPos::MIDDLE:
                break;
            case AuxSwitchPos::LOW:
                copter.g2.weathervane.allow_weathervaning(false);
                break;
        }
        break;
    }
#endif

    default:
        return RC_Channel::do_aux_function(ch_option, ch_flag);
    }
    return true;
}

// change air-mode status
void RC_Channel_Copter::do_aux_function_change_air_mode(const AuxSwitchPos ch_flag)
{
    switch (ch_flag) {
    case AuxSwitchPos::HIGH:
        copter.air_mode = AirMode::AIRMODE_ENABLED;
        break;
    case AuxSwitchPos::MIDDLE:
        break;
    case AuxSwitchPos::LOW:
        copter.air_mode = AirMode::AIRMODE_DISABLED;
        break;
    }
}

// change force flying status
void RC_Channel_Copter::do_aux_function_change_force_flying(const AuxSwitchPos ch_flag)
{
    switch (ch_flag) {
    case AuxSwitchPos::HIGH:
        copter.force_flying = true;
        break;
    case AuxSwitchPos::MIDDLE:
        break;
    case AuxSwitchPos::LOW:
        copter.force_flying = false;
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
void Copter::auto_trim_cancel()
{
    auto_trim_counter = 0;
    AP_Notify::flags.save_trim = false;
    gcs().send_text(MAV_SEVERITY_INFO, "AutoTrim cancelled");
}

void Copter::auto_trim()
{
    if (auto_trim_counter > 0) {
        if (copter.flightmode != &copter.mode_stabilize ||
            !copter.motors->armed()) {
            auto_trim_cancel();
            return;
        }

        // flash the leds
        AP_Notify::flags.save_trim = true;

        if (!auto_trim_started) {
            if (ap.land_complete) {
                // haven't taken off yet
                return;
            }
            auto_trim_started = true;
        }

        if (ap.land_complete) {
            // landed again.
            auto_trim_cancel();
            return;
        }

        auto_trim_counter--;

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
            gcs().send_text(MAV_SEVERITY_INFO, "AutoTrim: Trims saved");
        }
    }
}
