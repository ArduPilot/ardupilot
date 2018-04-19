#include "Copter.h"

#include "AP_RCSwitch.h"

#define CONTROL_SWITCH_DEBOUNCE_TIME_MS  200

void Copter::read_control_switch()
{
    if (g.flight_mode_chan <= 0) {
        // no flight mode channel
        return;
    }
    
    uint32_t tnow_ms = millis();

    // calculate position of flight mode switch
    int8_t switch_position;
    uint16_t mode_in = RC_Channels::rc_channel(g.flight_mode_chan-1)->get_radio_in();
    if      (mode_in < 1231) switch_position = 0;
    else if (mode_in < 1361) switch_position = 1;
    else if (mode_in < 1491) switch_position = 2;
    else if (mode_in < 1621) switch_position = 3;
    else if (mode_in < 1750) switch_position = 4;
    else switch_position = 5;

    // store time that switch last moved
    if (control_switch_state.last_switch_position != switch_position) {
        control_switch_state.last_edge_time_ms = tnow_ms;
    }

    // debounce switch
    bool control_switch_changed = control_switch_state.debounced_switch_position != switch_position;
    bool sufficient_time_elapsed = tnow_ms - control_switch_state.last_edge_time_ms > CONTROL_SWITCH_DEBOUNCE_TIME_MS;
    bool failsafe_disengaged = !failsafe.radio && failsafe.radio_counter == 0;

    if (control_switch_changed && sufficient_time_elapsed && failsafe_disengaged) {
        // set flight mode and simple mode setting
        if (copter.set_mode((control_mode_t)flight_modes[switch_position].get(), MODE_REASON_TX_COMMAND)) {
            // play a tone
            if (control_switch_state.debounced_switch_position != -1) {
                // alert user to mode change (except if autopilot is just starting up)
                if (ap.initialised) {
                    AP_Notify::events.user_mode_change = 1;
                }
            }

            if (!rcswitch.find_channel_for_option(AP_RCSwitch::aux_func::SIMPLE_MODE) &&
                !rcswitch.find_channel_for_option(AP_RCSwitch::aux_func::SUPERSIMPLE_MODE)) {
                // if none of the Aux Switches are set to Simple or Super Simple Mode then
                // set Simple Mode using stored parameters from EEPROM
                if (BIT_IS_SET(g.super_simple, switch_position)) {
                    set_simple_mode(2);
                } else {
                    set_simple_mode(BIT_IS_SET(g.simple_modes, switch_position));
                }
            }

        } else if (control_switch_state.last_switch_position != -1) {
            // alert user to mode change failure
            AP_Notify::events.user_mode_change_failed = 1;
        }

        // set the debounced switch position
        control_switch_state.debounced_switch_position = switch_position;
    }

    control_switch_state.last_switch_position = switch_position;
}

bool AP_RCSwitch_Copter::in_rc_failsafe() const
{
    if (copter.failsafe.radio || copter.failsafe.radio_counter != 0) {
        return true;
    }
    return false;
}

void AP_RCSwitch_Copter::reset_control_switch()
{
    copter.control_switch_state.last_switch_position = copter.control_switch_state.debounced_switch_position = -1;
    copter.read_control_switch();
}

// init_aux_switch_function - initialize aux functions
void AP_RCSwitch_Copter::init_aux_function(const aux_func_t ch_option, const aux_switch_pos_t ch_flag)
{
    // init channel options
    switch(ch_option) {
    case SIMPLE_MODE:
    case RANGEFINDER:
    case FENCE:
    case SUPERSIMPLE_MODE:
    case ACRO_TRAINER:
    case GRIPPER:
    case SPRAYER:
    case PARACHUTE_ENABLE:
    case PARACHUTE_3POS:      // we trust the vehicle will be disarmed so even if switch is in release position the chute will not release
    case RETRACT_MOUNT:
    case MISSION_RESET:
    case ATTCON_FEEDFWD:
    case ATTCON_ACCEL_LIM:
    case MOTOR_ESTOP:
    case MOTOR_INTERLOCK:
    case AVOID_ADSB:
    case PRECISION_LOITER:
    case AVOID_PROXIMITY:
    case INVERTED:
    case WINCH_ENABLE:
    case RC_OVERRIDE_ENABLE:
        do_aux_function(ch_option, ch_flag);
        break;
    // the following functions to not need to be initialised:
    case FLIP:
    case RTL:
    case SAVE_TRIM:
    case SAVE_WP:
    case RESETTOARMEDYAW:
    case AUTO:
        break;
    default:
        AP_RCSwitch::init_aux_function(ch_option, ch_flag);
        break;
    }
}

// do_aux_switch_function - implement the function invoked by the ch7 or ch8 switch
void AP_RCSwitch_Copter::do_aux_function_change_mode(const control_mode_t mode,
                                                     const aux_switch_pos_t ch_flag)
{
    switch(ch_flag) {
    case HIGH:
        // engage mode (if not possible we remain in current flight mode)
        copter.set_mode(mode, MODE_REASON_TX_COMMAND);
        break;
    default:
        // return to flight mode switch's flight mode if we are currently
        // in this mode
        if (copter.control_mode == mode) {
            reset_control_switch();
        }
    }
}

void AP_RCSwitch_Copter::do_aux_function(const aux_func_t ch_option, const aux_switch_pos_t ch_flag)
{
    switch(ch_option) {
        case FLIP:
            // flip if switch is on, positive throttle and we're actually flying
            if (ch_flag == aux_switch_pos::HIGH) {
                copter.set_mode(control_mode_t::FLIP, MODE_REASON_TX_COMMAND);
            }
            break;

        case SIMPLE_MODE:
            // low = simple mode off, middle or high position turns simple mode on
            copter.set_simple_mode(ch_flag == HIGH || ch_flag == MIDDLE);
            break;

        case SUPERSIMPLE_MODE:
            // low = simple mode off, middle = simple mode, high = super simple mode
            copter.set_simple_mode(ch_flag);
            break;

        case RTL:
#if MODE_RTL_ENABLED == ENABLED
            do_aux_function_change_mode(control_mode_t::RTL, ch_flag);
#endif
            break;

        case SAVE_TRIM:
            if ((ch_flag == HIGH) && (copter.control_mode <= control_mode_t::ACRO) && (copter.channel_throttle->get_control_in() == 0)) {
                copter.save_trim();
            }
            break;

        case SAVE_WP:
#if MODE_AUTO_ENABLED == ENABLED
            // save waypoint when switch is brought high
            if (ch_flag == HIGH) {

                // do not allow saving new waypoints while we're in auto or disarmed
                if (copter.control_mode == control_mode_t::AUTO || !copter.motors->armed()) {
                    return;
                }

                // do not allow saving the first waypoint with zero throttle
                if ((copter.mission.num_commands() == 0) && (copter.channel_throttle->get_control_in() == 0)) {
                    return;
                }

                // create new mission command
                AP_Mission::Mission_Command cmd  = {};

                // if the mission is empty save a takeoff command
                if (copter.mission.num_commands() == 0) {
                    // set our location ID to 16, MAV_CMD_NAV_WAYPOINT
                    cmd.id = MAV_CMD_NAV_TAKEOFF;
                    cmd.content.location.options = 0;
                    cmd.p1 = 0;
                    cmd.content.location.lat = 0;
                    cmd.content.location.lng = 0;
                    cmd.content.location.alt = MAX(copter.current_loc.alt,100);

                    // use the current altitude for the target alt for takeoff.
                    // only altitude will matter to the AP mission script for takeoff.
                    if (copter.mission.add_cmd(cmd)) {
                        // log event
                        copter.Log_Write_Event(DATA_SAVEWP_ADD_WP);
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
                if (copter.mission.add_cmd(cmd)) {
                    // log event
                    copter.Log_Write_Event(DATA_SAVEWP_ADD_WP);
                }
            }
#endif
            break;

        case MISSION_RESET:
#if MODE_AUTO_ENABLED == ENABLED
            if (ch_flag == HIGH) {
                copter.mission.reset();
            }
#endif
            break;

        case AUTO:
#if MODE_AUTO_ENABLED == ENABLED
            do_aux_function_change_mode(control_mode_t::AUTO, ch_flag);
#endif
            break;

        case RANGEFINDER:
            // enable or disable the rangefinder
#if RANGEFINDER_ENABLED == ENABLED
            if ((ch_flag == HIGH) && copter.rangefinder.has_orientation(ROTATION_PITCH_270)) {
                copter.rangefinder_state.enabled = true;
            } else {
                copter.rangefinder_state.enabled = false;
            }
#endif
            break;

        case FENCE:
#if AC_FENCE == ENABLED
            // enable or disable the fence
            if (ch_flag == HIGH) {
                copter.fence.enable(true);
                copter.Log_Write_Event(DATA_FENCE_ENABLE);
            } else {
                copter.fence.enable(false);
                copter.Log_Write_Event(DATA_FENCE_DISABLE);
            }
#endif
            break;

        case ACRO_TRAINER:
#if MODE_ACRO_ENABLED == ENABLED
            switch(ch_flag) {
                case LOW:
                    copter.g.acro_trainer = ACRO_TRAINER_DISABLED;
                    copter.Log_Write_Event(DATA_ACRO_TRAINER_DISABLED);
                    break;
                case MIDDLE:
                    copter.g.acro_trainer = ACRO_TRAINER_LEVELING;
                    copter.Log_Write_Event(DATA_ACRO_TRAINER_LEVELING);
                    break;
                case HIGH:
                    copter.g.acro_trainer = ACRO_TRAINER_LIMITED;
                    copter.Log_Write_Event(DATA_ACRO_TRAINER_LIMITED);
                    break;
            }
#endif
            break;

        case GRIPPER:
#if GRIPPER_ENABLED == ENABLED
            switch(ch_flag) {
                case LOW:
                    copter.g2.gripper.release();
                    copter.Log_Write_Event(DATA_GRIPPER_RELEASE);
                    break;
                case MIDDLE:
                    // nothing
                    break;
                case HIGH:
                    copter.g2.gripper.grab();
                    copter.Log_Write_Event(DATA_GRIPPER_GRAB);
                    break;
            }
#endif
            break;

        case SPRAYER:
#if SPRAYER_ENABLED == ENABLED
            copter.sprayer.run(ch_flag == HIGH);
            // if we are disarmed the pilot must want to test the pump
            copter.sprayer.test_pump((ch_flag == HIGH) && !copter.motors->armed());
#endif
            break;

        case AUTOTUNE:
#if AUTOTUNE_ENABLED == ENABLED
            do_aux_function_change_mode(control_mode_t::AUTOTUNE, ch_flag);
#endif
            break;

        case LAND:
            do_aux_function_change_mode(control_mode_t::LAND, ch_flag);
            break;

        case GUIDED:
            do_aux_function_change_mode(control_mode_t::GUIDED, ch_flag);
            break;

        case PARACHUTE_ENABLE:
#if PARACHUTE == ENABLED
            // Parachute enable/disable
            copter.parachute.enabled(ch_flag == HIGH);
#endif
            break;

        case PARACHUTE_RELEASE:
#if PARACHUTE == ENABLED
            if (ch_flag == HIGH) {
                copter.parachute_manual_release();
            }
#endif
            break;

        case PARACHUTE_3POS:
#if PARACHUTE == ENABLED
            // Parachute disable, enable, release with 3 position switch
            switch (ch_flag) {
                case LOW:
                    copter.parachute.enabled(false);
                    copter.Log_Write_Event(DATA_PARACHUTE_DISABLED);
                    break;
                case MIDDLE:
                    copter.parachute.enabled(true);
                    copter.Log_Write_Event(DATA_PARACHUTE_ENABLED);
                    break;
                case HIGH:
                    copter.parachute.enabled(true);
                    copter.parachute_manual_release();
                    break;
            }
#endif
            break;

        case ATTCON_FEEDFWD:
            // enable or disable feed forward
            copter.attitude_control->bf_feedforward(ch_flag == HIGH);
            break;

        case ATTCON_ACCEL_LIM:
            // enable or disable accel limiting by restoring defaults
            copter.attitude_control->accel_limiting(ch_flag == HIGH);
            break;

        case RETRACT_MOUNT:
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

       case LANDING_GEAR:
            switch (ch_flag) {
                case LOW:
                    copter.landinggear.set_position(AP_LandingGear::LandingGear_Deploy);
                    break;
                case MIDDLE:
                    // nothing
                    break;
                case HIGH:
                    copter.landinggear.set_position(AP_LandingGear::LandingGear_Retract);
                    break;
            }
            break;

        case LOST_COPTER_SOUND:
            switch (ch_flag) {
                case HIGH:
                    AP_Notify::flags.vehicle_lost = true;
                    break;
                case MIDDLE:
                    // nothing
                    break;
                case LOW:
                    AP_Notify::flags.vehicle_lost = false;
                    break;
            }
            break;

        case MOTOR_ESTOP:
            // Turn on Emergency Stop logic when channel is high
            copter.set_motor_emergency_stop(ch_flag == HIGH);
            break;

        case MOTOR_INTERLOCK:
            // Turn on when above LOW, because channel will also be used for speed
            // control signal in tradheli
            copter.ap.motor_interlock_switch = (ch_flag == HIGH || ch_flag == MIDDLE);
            break;

        case BRAKE:
#if MODE_BRAKE_ENABLED == ENABLED
            do_aux_function_change_mode(control_mode_t::BRAKE, ch_flag);
#endif
            break;

        case THROW:
#if MODE_THROW_ENABLED == ENABLED
            do_aux_function_change_mode(control_mode_t::THROW, ch_flag);
#endif
            break;

        case AVOID_ADSB:
#if ADSB_ENABLED == ENABLED
            // enable or disable AP_Avoidance
            if (ch_flag == HIGH) {
                copter.avoidance_adsb.enable();
                copter.Log_Write_Event(DATA_AVOIDANCE_ADSB_ENABLE);
            } else {
                copter.avoidance_adsb.disable();
                copter.Log_Write_Event(DATA_AVOIDANCE_ADSB_DISABLE);
            }
#endif
            break;

        case PRECISION_LOITER:
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

        case AVOID_PROXIMITY:
#if PROXIMITY_ENABLED == ENABLED && AC_AVOID_ENABLED == ENABLED
            switch (ch_flag) {
                case HIGH:
                    copter.avoid.proximity_avoidance_enable(true);
                    copter.Log_Write_Event(DATA_AVOIDANCE_PROXIMITY_ENABLE);
                    break;
                case MIDDLE:
                    // nothing
                    break;
                case LOW:
                    copter.avoid.proximity_avoidance_enable(false);
                    copter.Log_Write_Event(DATA_AVOIDANCE_PROXIMITY_DISABLE);
                    break;
            }
#endif
            break;
        case ARMDISARM:
            // arm or disarm the vehicle
            switch (ch_flag) {
            case HIGH:
                copter.init_arm_motors(false);
                break;
            case MIDDLE:
                // nothing
                break;
            case LOW:
                copter.init_disarm_motors();
                break;
            }
            break;

        case SMART_RTL:
#if MODE_SMARTRTL_ENABLED == ENABLED
            do_aux_function_change_mode(control_mode_t::SMART_RTL, ch_flag);
#endif
            break;

        case INVERTED:
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

        case WINCH_ENABLE:
#if WINCH_ENABLED == ENABLED
            switch (ch_flag) {
                case HIGH:
                    // high switch maintains current position
                    copter.g2.winch.release_length(0.0f);
                    copter.Log_Write_Event(DATA_WINCH_LENGTH_CONTROL);
                    break;
                default:
                    // all other position relax winch
                    copter.g2.winch.relax();
                    copter.Log_Write_Event(DATA_WINCH_RELAXED);
                    break;
                }
#endif
            break;

        case WINCH_CONTROL:
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

        case RC_OVERRIDE_ENABLE:
            // Allow or disallow RC_Override
            switch (ch_flag) {
                case HIGH: {
                    copter.ap.rc_override_enable = true;
                    break;
                }
                case MIDDLE:
                    // nothing
                    break;
                case LOW: {
                    copter.ap.rc_override_enable = false;
                    break;
                }
            }
            break;
    default:
        AP_RCSwitch::do_aux_function(ch_option, ch_flag);
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
    Log_Write_Event(DATA_SAVE_TRIM);
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

