/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

#define CONTROL_SWITCH_DEBOUNCE_TIME_MS  200

//Documentation of Aux Switch Flags:
static union {
    struct {
        uint8_t CH6_flag            : 2; // 0, 1    // ch6 aux switch : 0 is low or false, 1 is center or true, 2 is high
        uint8_t CH7_flag            : 2; // 2, 3    // ch7 aux switch : 0 is low or false, 1 is center or true, 2 is high
        uint8_t CH8_flag            : 2; // 4, 5    // ch8 aux switch : 0 is low or false, 1 is center or true, 2 is high
        uint8_t CH9_flag            : 2; // 6, 7    // ch9 aux switch : 0 is low or false, 1 is center or true, 2 is high
        uint8_t CH10_flag           : 2; // 8, 9    // ch10 aux switch : 0 is low or false, 1 is center or true, 2 is high
        uint8_t CH11_flag           : 2; // 10,11   // ch11 aux switch : 0 is low or false, 1 is center or true, 2 is high
        uint8_t CH12_flag           : 2; // 12,13   // ch12 aux switch : 0 is low or false, 1 is center or true, 2 is high
    };
    uint32_t value;
} aux_con;

void Copter::read_control_switch()
{
    uint32_t tnow_ms = millis();

    // calculate position of flight mode switch
    int8_t switch_position;
    if      (g.rc_5.get_radio_in() < 1231) switch_position = 0;
    else if (g.rc_5.get_radio_in() < 1361) switch_position = 1;
    else if (g.rc_5.get_radio_in() < 1491) switch_position = 2;
    else if (g.rc_5.get_radio_in() < 1621) switch_position = 3;
    else if (g.rc_5.get_radio_in() < 1750) switch_position = 4;
    else switch_position = 5;

    // store time that switch last moved
    if(control_switch_state.last_switch_position != switch_position) {
        control_switch_state.last_edge_time_ms = tnow_ms;
    }

    // debounce switch
    bool control_switch_changed = control_switch_state.debounced_switch_position != switch_position;
    bool sufficient_time_elapsed = tnow_ms - control_switch_state.last_edge_time_ms > CONTROL_SWITCH_DEBOUNCE_TIME_MS;
    bool failsafe_disengaged = !failsafe.radio && failsafe.radio_counter == 0;

    if (control_switch_changed && sufficient_time_elapsed && failsafe_disengaged) {
        // set flight mode and simple mode setting
        if (set_mode((control_mode_t)flight_modes[switch_position].get(), MODE_REASON_TX_COMMAND)) {
            // play a tone
            if (control_switch_state.debounced_switch_position != -1) {
                // alert user to mode change failure (except if autopilot is just starting up)
                if (ap.initialised) {
                    AP_Notify::events.user_mode_change = 1;
                }
            }

            if(!check_if_auxsw_mode_used(AUXSW_SIMPLE_MODE) && !check_if_auxsw_mode_used(AUXSW_SUPERSIMPLE_MODE)) {
                // if none of the Aux Switches are set to Simple or Super Simple Mode then
                // set Simple Mode using stored parameters from EEPROM
                if (BIT_IS_SET(g.super_simple, switch_position)) {
                    set_simple_mode(2);
                }else{
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

// check_if_auxsw_mode_used - Check to see if any of the Aux Switches are set to a given mode.
bool Copter::check_if_auxsw_mode_used(uint8_t auxsw_mode_check)
{
    bool ret = g.ch7_option == auxsw_mode_check || g.ch8_option == auxsw_mode_check || g.ch9_option == auxsw_mode_check 
                || g.ch10_option == auxsw_mode_check || g.ch11_option == auxsw_mode_check || g.ch12_option == auxsw_mode_check;

    return ret;
}

// check_duplicate_auxsw - Check to see if any Aux Switch Functions are duplicated
bool Copter::check_duplicate_auxsw(void)
{
    bool ret = ((g.ch7_option != AUXSW_DO_NOTHING) && (g.ch7_option == g.ch8_option ||
                g.ch7_option == g.ch9_option || g.ch7_option == g.ch10_option ||
                g.ch7_option == g.ch11_option || g.ch7_option == g.ch12_option));

    ret = ret || ((g.ch8_option != AUXSW_DO_NOTHING) && (g.ch8_option == g.ch9_option ||
                    g.ch8_option == g.ch10_option || g.ch8_option == g.ch11_option ||
                    g.ch8_option == g.ch12_option));

    ret = ret || ((g.ch9_option != AUXSW_DO_NOTHING) && (g.ch9_option == g.ch10_option ||
                    g.ch9_option == g.ch11_option || g.ch9_option == g.ch12_option));

    ret = ret || ((g.ch10_option != AUXSW_DO_NOTHING) && (g.ch10_option == g.ch11_option ||
                    g.ch10_option == g.ch12_option));

    ret = ret || ((g.ch11_option != AUXSW_DO_NOTHING) && (g.ch11_option == g.ch12_option));

    return ret;
}

void Copter::reset_control_switch()
{
    control_switch_state.last_switch_position = control_switch_state.debounced_switch_position = -1;
    read_control_switch();
}

// read_3pos_switch
uint8_t Copter::read_3pos_switch(int16_t radio_in)
{
    if (radio_in < AUX_SWITCH_PWM_TRIGGER_LOW) return AUX_SWITCH_LOW;      // switch is in low position
    if (radio_in > AUX_SWITCH_PWM_TRIGGER_HIGH) return AUX_SWITCH_HIGH;    // switch is in high position
    return AUX_SWITCH_MIDDLE;                                       // switch is in middle position
}

// read_aux_switches - checks aux switch positions and invokes configured actions
void Copter::read_aux_switches()
{
    uint8_t switch_position;

    // exit immediately during radio failsafe
    if (failsafe.radio || failsafe.radio_counter != 0) {
        return;
    }

    // check if ch7 switch has changed position
    switch_position = read_3pos_switch(g.rc_7.get_radio_in());
    if (aux_con.CH7_flag != switch_position) {
        // set the CH7 flag
        aux_con.CH7_flag = switch_position;

        // invoke the appropriate function
        do_aux_switch_function(g.ch7_option, aux_con.CH7_flag);
    }

    // check if Ch8 switch has changed position
    switch_position = read_3pos_switch(g.rc_8.get_radio_in());
    if (aux_con.CH8_flag != switch_position) {
        // set the CH8 flag
        aux_con.CH8_flag = switch_position;

        // invoke the appropriate function
        do_aux_switch_function(g.ch8_option, aux_con.CH8_flag);
    }

    // check if Ch9 switch has changed position
    switch_position = read_3pos_switch(g.rc_9.get_radio_in());
    if (aux_con.CH9_flag != switch_position) {
        // set the CH9 flag
        aux_con.CH9_flag = switch_position;

        // invoke the appropriate function
        do_aux_switch_function(g.ch9_option, aux_con.CH9_flag);
    }

    // check if Ch10 switch has changed position
    switch_position = read_3pos_switch(g.rc_10.get_radio_in());
    if (aux_con.CH10_flag != switch_position) {
        // set the CH10 flag
        aux_con.CH10_flag = switch_position;

        // invoke the appropriate function
        do_aux_switch_function(g.ch10_option, aux_con.CH10_flag);
    }

    // check if Ch11 switch has changed position
    switch_position = read_3pos_switch(g.rc_11.get_radio_in());
    if (aux_con.CH11_flag != switch_position) {
        // set the CH11 flag
        aux_con.CH11_flag = switch_position;

        // invoke the appropriate function
        do_aux_switch_function(g.ch11_option, aux_con.CH11_flag);
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    // check if Ch12 switch has changed position
    switch_position = read_3pos_switch(g.rc_12.get_radio_in());
    if (aux_con.CH12_flag != switch_position) {
        // set the CH12 flag
        aux_con.CH12_flag = switch_position;

        // invoke the appropriate function
        do_aux_switch_function(g.ch12_option, aux_con.CH12_flag);
    }
#endif
}

// init_aux_switches - invoke configured actions at start-up for aux function where it is safe to do so
void Copter::init_aux_switches()
{
    // set the CH7 ~ CH12 flags
    aux_con.CH7_flag = read_3pos_switch(g.rc_7.get_radio_in());
    aux_con.CH8_flag = read_3pos_switch(g.rc_8.get_radio_in());
    aux_con.CH10_flag = read_3pos_switch(g.rc_10.get_radio_in());
    aux_con.CH11_flag = read_3pos_switch(g.rc_11.get_radio_in());

    // ch9, ch12 only supported on some boards
    aux_con.CH9_flag = read_3pos_switch(g.rc_9.get_radio_in());
    aux_con.CH12_flag = read_3pos_switch(g.rc_12.get_radio_in());

    // initialise functions assigned to switches
    init_aux_switch_function(g.ch7_option, aux_con.CH7_flag);
    init_aux_switch_function(g.ch8_option, aux_con.CH8_flag);
    init_aux_switch_function(g.ch10_option, aux_con.CH10_flag);
    init_aux_switch_function(g.ch11_option, aux_con.CH11_flag);

    // ch9, ch12 only supported on some boards
    init_aux_switch_function(g.ch9_option, aux_con.CH9_flag);
    init_aux_switch_function(g.ch12_option, aux_con.CH12_flag);
}

// init_aux_switch_function - initialize aux functions
void Copter::init_aux_switch_function(int8_t ch_option, uint8_t ch_flag)
{    
    // init channel options
    switch(ch_option) {
        case AUXSW_SIMPLE_MODE:
        case AUXSW_RANGEFINDER:
        case AUXSW_FENCE:
        case AUXSW_RESETTOARMEDYAW:
        case AUXSW_SUPERSIMPLE_MODE:
        case AUXSW_ACRO_TRAINER:
        case AUXSW_EPM:
        case AUXSW_SPRAYER:
        case AUXSW_PARACHUTE_ENABLE:
        case AUXSW_PARACHUTE_3POS:      // we trust the vehicle will be disarmed so even if switch is in release position the chute will not release
        case AUXSW_RETRACT_MOUNT:
        case AUXSW_MISSION_RESET:
        case AUXSW_ATTCON_FEEDFWD:
        case AUXSW_ATTCON_ACCEL_LIM:
        case AUXSW_LANDING_GEAR:
        case AUXSW_MOTOR_ESTOP:
        case AUXSW_MOTOR_INTERLOCK:
            do_aux_switch_function(ch_option, ch_flag);
            break;
    }
}

// do_aux_switch_function - implement the function invoked by the ch7 or ch8 switch
void Copter::do_aux_switch_function(int8_t ch_function, uint8_t ch_flag)
{

    switch(ch_function) {
        case AUXSW_FLIP:
            // flip if switch is on, positive throttle and we're actually flying
            if(ch_flag == AUX_SWITCH_HIGH) {
                set_mode(FLIP, MODE_REASON_TX_COMMAND);
            }
            break;

        case AUXSW_SIMPLE_MODE:
            // low = simple mode off, middle or high position turns simple mode on
            set_simple_mode(ch_flag == AUX_SWITCH_HIGH || ch_flag == AUX_SWITCH_MIDDLE);
            break;

        case AUXSW_SUPERSIMPLE_MODE:
            // low = simple mode off, middle = simple mode, high = super simple mode
            set_simple_mode(ch_flag);
            break;

        case AUXSW_RTL:
            if (ch_flag == AUX_SWITCH_HIGH) {
                // engage RTL (if not possible we remain in current flight mode)
                set_mode(RTL, MODE_REASON_TX_COMMAND);
            }else{
                // return to flight mode switch's flight mode if we are currently in RTL
                if (control_mode == RTL) {
                    reset_control_switch();
                }
            }
            break;

        case AUXSW_SAVE_TRIM:
            if ((ch_flag == AUX_SWITCH_HIGH) && (control_mode <= ACRO) && (channel_throttle->get_control_in() == 0)) {
                save_trim();
            }
            break;

        case AUXSW_SAVE_WP:
            // save waypoint when switch is brought high
            if (ch_flag == AUX_SWITCH_HIGH) {

                // do not allow saving new waypoints while we're in auto or disarmed
                if(control_mode == AUTO || !motors.armed()) {
                    return;
                }

                // do not allow saving the first waypoint with zero throttle
                if((mission.num_commands() == 0) && (channel_throttle->get_control_in() == 0)){
                    return;
                }

                // create new mission command
                AP_Mission::Mission_Command cmd  = {};

                // if the mission is empty save a takeoff command
                if(mission.num_commands() == 0) {
                    // set our location ID to 16, MAV_CMD_NAV_WAYPOINT
                    cmd.id = MAV_CMD_NAV_TAKEOFF;
                    cmd.content.location.options = 0;
                    cmd.p1 = 0;
                    cmd.content.location.lat = 0;
                    cmd.content.location.lng = 0;
                    cmd.content.location.alt = MAX(current_loc.alt,100);

                    // use the current altitude for the target alt for takeoff.
                    // only altitude will matter to the AP mission script for takeoff.
                    if(mission.add_cmd(cmd)) {
                        // log event
                        Log_Write_Event(DATA_SAVEWP_ADD_WP);
                    }
                }

                // set new waypoint to current location
                cmd.content.location = current_loc;

                // if throttle is above zero, create waypoint command
                if(channel_throttle->get_control_in() > 0) {
                    cmd.id = MAV_CMD_NAV_WAYPOINT;
                }else{
                    // with zero throttle, create LAND command
                    cmd.id = MAV_CMD_NAV_LAND;
                }

                // save command
                if(mission.add_cmd(cmd)) {
                    // log event
                    Log_Write_Event(DATA_SAVEWP_ADD_WP);
                }
            }
            break;

#if CAMERA == ENABLED
        case AUXSW_CAMERA_TRIGGER:
            if (ch_flag == AUX_SWITCH_HIGH) {
                do_take_picture();
            }
            break;
#endif

        case AUXSW_RANGEFINDER:
            // enable or disable the rangefinder
#if RANGEFINDER_ENABLED == ENABLED
            if ((ch_flag == AUX_SWITCH_HIGH) && (rangefinder.num_sensors() >= 1)) {
                rangefinder_state.enabled = true;
            }else{
                rangefinder_state.enabled = false;
            }
#endif
            break;

#if AC_FENCE == ENABLED
        case AUXSW_FENCE:
            // enable or disable the fence
            if (ch_flag == AUX_SWITCH_HIGH) {
                fence.enable(true);
                Log_Write_Event(DATA_FENCE_ENABLE);
            }else{
                fence.enable(false);
                Log_Write_Event(DATA_FENCE_DISABLE);
            }
            break;
#endif
        // To-Do: add back support for this feature
        //case AUXSW_RESETTOARMEDYAW:
        //    if (ch_flag == AUX_SWITCH_HIGH) {
        //        set_yaw_mode(YAW_RESETTOARMEDYAW);
        //    }else{
        //        set_yaw_mode(YAW_HOLD);
        //    }
        //    break;

        case AUXSW_ACRO_TRAINER:
            switch(ch_flag) {
                case AUX_SWITCH_LOW:
                    g.acro_trainer = ACRO_TRAINER_DISABLED;
                    Log_Write_Event(DATA_ACRO_TRAINER_DISABLED);
                    break;
                case AUX_SWITCH_MIDDLE:
                    g.acro_trainer = ACRO_TRAINER_LEVELING;
                    Log_Write_Event(DATA_ACRO_TRAINER_LEVELING);
                    break;
                case AUX_SWITCH_HIGH:
                    g.acro_trainer = ACRO_TRAINER_LIMITED;
                    Log_Write_Event(DATA_ACRO_TRAINER_LIMITED);
                    break;
            }
            break;
#if EPM_ENABLED == ENABLED
        case AUXSW_EPM:
            switch(ch_flag) {
                case AUX_SWITCH_LOW:
                    epm.release();
                    Log_Write_Event(DATA_EPM_RELEASE);
                    break;
                case AUX_SWITCH_HIGH:
                    epm.grab();
                    Log_Write_Event(DATA_EPM_GRAB);
                    break;
            }
            break;
#endif
#if SPRAYER == ENABLED
        case AUXSW_SPRAYER:
            sprayer.enable(ch_flag == AUX_SWITCH_HIGH);
            // if we are disarmed the pilot must want to test the pump
            sprayer.test_pump((ch_flag == AUX_SWITCH_HIGH) && !motors.armed());
            break;
#endif

        case AUXSW_AUTO:
            if (ch_flag == AUX_SWITCH_HIGH) {
                set_mode(AUTO, MODE_REASON_TX_COMMAND);
            }else{
                // return to flight mode switch's flight mode if we are currently in AUTO
                if (control_mode == AUTO) {
                    reset_control_switch();
                }
            }
            break;

#if AUTOTUNE_ENABLED == ENABLED
        case AUXSW_AUTOTUNE:
            // turn on auto tuner
            switch(ch_flag) {
                case AUX_SWITCH_LOW:
                case AUX_SWITCH_MIDDLE:
                    // restore flight mode based on flight mode switch position
                    if (control_mode == AUTOTUNE) {
                        reset_control_switch();
                    }
                    break;
                case AUX_SWITCH_HIGH:
                    // start an autotuning session
                    set_mode(AUTOTUNE, MODE_REASON_TX_COMMAND);
                    break;
            }
            break;
#endif

        case AUXSW_LAND:
            if (ch_flag == AUX_SWITCH_HIGH) {
                set_mode(LAND, MODE_REASON_TX_COMMAND);
            }else{
                // return to flight mode switch's flight mode if we are currently in LAND
                if (control_mode == LAND) {
                    reset_control_switch();
                }
            }
            break;

#if PARACHUTE == ENABLED
        case AUXSW_PARACHUTE_ENABLE:
            // Parachute enable/disable
            parachute.enabled(ch_flag == AUX_SWITCH_HIGH);
            break;

        case AUXSW_PARACHUTE_RELEASE:
            if (ch_flag == AUX_SWITCH_HIGH) {
                parachute_manual_release();
            }
            break;

        case AUXSW_PARACHUTE_3POS:
            // Parachute disable, enable, release with 3 position switch
            switch (ch_flag) {
                case AUX_SWITCH_LOW:
                    parachute.enabled(false);
                    Log_Write_Event(DATA_PARACHUTE_DISABLED);
                    break;
                case AUX_SWITCH_MIDDLE:
                    parachute.enabled(true);
                    Log_Write_Event(DATA_PARACHUTE_ENABLED);
                    break;
                case AUX_SWITCH_HIGH:
                    parachute.enabled(true);
                    parachute_manual_release();
                    break;
            }
            break;
#endif

        case AUXSW_MISSION_RESET:
            if (ch_flag == AUX_SWITCH_HIGH) {
                mission.reset();
            }
            break;

        case AUXSW_ATTCON_FEEDFWD:
            // enable or disable feed forward
            attitude_control.bf_feedforward(ch_flag == AUX_SWITCH_HIGH);
            break;

        case AUXSW_ATTCON_ACCEL_LIM:
            // enable or disable accel limiting by restoring defaults
            attitude_control.accel_limiting(ch_flag == AUX_SWITCH_HIGH);
            break;
        
#if MOUNT == ENABLE
        case AUXSW_RETRACT_MOUNT:
            switch (ch_flag) {
                case AUX_SWITCH_HIGH:
                    camera_mount.set_mode(MAV_MOUNT_MODE_RETRACT);
                    break;
                case AUX_SWITCH_LOW:
                    camera_mount.set_mode_to_default();
                    break;
            }
            break;
#endif

        case AUXSW_RELAY:
            ServoRelayEvents.do_set_relay(0, ch_flag == AUX_SWITCH_HIGH);
            break;

        case AUXSW_RELAY2:
            ServoRelayEvents.do_set_relay(1, ch_flag == AUX_SWITCH_HIGH);
            break;

        case AUXSW_RELAY3:
            ServoRelayEvents.do_set_relay(2, ch_flag == AUX_SWITCH_HIGH);
            break;

	   case AUXSW_RELAY4:
            ServoRelayEvents.do_set_relay(3, ch_flag == AUX_SWITCH_HIGH);
            break;

		case AUXSW_LANDING_GEAR:
            switch (ch_flag) {
                case AUX_SWITCH_LOW:
                    landinggear.set_cmd_mode(LandingGear_Deploy);
                    break;
                case AUX_SWITCH_MIDDLE:
                    landinggear.set_cmd_mode(LandingGear_Auto);
                    break;
                case AUX_SWITCH_HIGH:
                    landinggear.set_cmd_mode(LandingGear_Retract);
                    break;
            }
            break;

        case AUXSW_LOST_COPTER_SOUND:
            switch (ch_flag) {
                case AUX_SWITCH_HIGH:
                    AP_Notify::flags.vehicle_lost = true;
                    break;
                case AUX_SWITCH_LOW:
                    AP_Notify::flags.vehicle_lost = false;
                    break;
            }
            break;

        case AUXSW_MOTOR_ESTOP:
            // Turn on Emergency Stop logic when channel is high
            set_motor_emergency_stop(ch_flag == AUX_SWITCH_HIGH);
            break;

        case AUXSW_MOTOR_INTERLOCK:
            // Turn on when above LOW, because channel will also be used for speed
            // control signal in tradheli
            motors.set_interlock(ch_flag == AUX_SWITCH_HIGH || ch_flag == AUX_SWITCH_MIDDLE);

            // remember the current value of the motor interlock so that this condition can be restored if we exit the throw mode early
            throw_early_exit_interlock = motors.get_interlock();

            // Log new status
            if (motors.get_interlock()){
                Log_Write_Event(DATA_MOTORS_INTERLOCK_ENABLED);
            } else {
                Log_Write_Event(DATA_MOTORS_INTERLOCK_DISABLED);
            }
            break;

        case AUXSW_BRAKE:
            // brake flight mode
            if (ch_flag == AUX_SWITCH_HIGH) {
                set_mode(BRAKE, MODE_REASON_TX_COMMAND);
            }else{
                // return to flight mode switch's flight mode if we are currently in BRAKE
                if (control_mode == BRAKE) {
                    reset_control_switch();
                }
            }
            break;

        case AUXSW_THROW:
            // throw flight mode
            if (ch_flag == AUX_SWITCH_HIGH) {
                set_mode(THROW, MODE_REASON_TX_COMMAND);
            } else {
                // return to flight mode switch's flight mode if we are currently in throw mode
                if (control_mode == THROW) {
                    reset_control_switch();
                }
            }
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
    gcs_send_text(MAV_SEVERITY_INFO, "Trim saved");
}

// auto_trim - slightly adjusts the ahrs.roll_trim and ahrs.pitch_trim towards the current stick positions
// meant to be called continuously while the pilot attempts to keep the copter level
void Copter::auto_trim()
{
    if(auto_trim_counter > 0) {
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
        if(auto_trim_counter == 0) {
            AP_Notify::flags.save_trim = false;
        }
    }
}

