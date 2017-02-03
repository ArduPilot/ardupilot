/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Sub.h"

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

// check_if_auxsw_mode_used - Check to see if any of the Aux Switches are set to a given mode.
bool Sub::check_if_auxsw_mode_used(uint8_t auxsw_mode_check)
{
    bool ret = g.ch7_option == auxsw_mode_check || g.ch8_option == auxsw_mode_check || g.ch9_option == auxsw_mode_check
               || g.ch10_option == auxsw_mode_check || g.ch11_option == auxsw_mode_check || g.ch12_option == auxsw_mode_check;

    return ret;
}

// check_duplicate_auxsw - Check to see if any Aux Switch Functions are duplicated
bool Sub::check_duplicate_auxsw(void)
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

// read_3pos_switch
uint8_t Sub::read_3pos_switch(int16_t radio_in)
{
    if (radio_in < AUX_SWITCH_PWM_TRIGGER_LOW) {
        return AUX_SWITCH_LOW;    // switch is in low position
    }
    if (radio_in > AUX_SWITCH_PWM_TRIGGER_HIGH) {
        return AUX_SWITCH_HIGH;    // switch is in high position
    }
    return AUX_SWITCH_MIDDLE;                                       // switch is in middle position
}

// can't take reference to a bitfield member, thus a #define:
#define read_aux_switch(chan, flag, option)                           \
    do {                                                            \
        switch_position = read_3pos_switch(chan); \
        if (flag != switch_position) {                              \
            flag = switch_position;                                 \
            do_aux_switch_function(option, flag);                   \
        }                                                           \
    } while (false)

// read_aux_switches - checks aux switch positions and invokes configured actions
void Sub::read_aux_switches()
{
    uint8_t switch_position;

    // exit immediately during radio failsafe
    if (failsafe.manual_control) {
        return;
    }

    read_aux_switch(CH_7, aux_con.CH7_flag, g.ch7_option);
    read_aux_switch(CH_8, aux_con.CH8_flag, g.ch8_option);
    read_aux_switch(CH_9, aux_con.CH9_flag, g.ch9_option);
    read_aux_switch(CH_10, aux_con.CH10_flag, g.ch10_option);
    read_aux_switch(CH_11, aux_con.CH11_flag, g.ch11_option);

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    read_aux_switch(CH_12, aux_con.CH12_flag, g.ch12_option);
#endif
}

#undef read_aux_switch

// init_aux_switches - invoke configured actions at start-up for aux function where it is safe to do so
void Sub::init_aux_switches()
{
    // set the CH7 ~ CH12 flags
    aux_con.CH7_flag = read_3pos_switch(CH_7);
    aux_con.CH8_flag = read_3pos_switch(CH_8);
    aux_con.CH10_flag = read_3pos_switch(CH_10);
    aux_con.CH11_flag = read_3pos_switch(CH_11);

    // ch9, ch12 only supported on some boards
    aux_con.CH9_flag = read_3pos_switch(CH_9);
    aux_con.CH12_flag = read_3pos_switch(CH_12);

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
void Sub::init_aux_switch_function(int8_t ch_option, uint8_t ch_flag)
{
    // init channel options
    switch (ch_option) {
    case AUXSW_SIMPLE_MODE:
    case AUXSW_RANGEFINDER:
    case AUXSW_FENCE:
    case AUXSW_RESETTOARMEDYAW:
    case AUXSW_SUPERSIMPLE_MODE:
    case AUXSW_ACRO_TRAINER:
    case AUXSW_GRIPPER:
    case AUXSW_RETRACT_MOUNT:
    case AUXSW_MISSION_RESET:
    case AUXSW_ATTCON_FEEDFWD:
    case AUXSW_ATTCON_ACCEL_LIM:
    case AUXSW_RELAY:
    case AUXSW_MOTOR_ESTOP:
    case AUXSW_MOTOR_INTERLOCK:
        do_aux_switch_function(ch_option, ch_flag);
        break;
    }
}

// do_aux_switch_function - implement the function invoked by the ch7 or ch8 switch
void Sub::do_aux_switch_function(int8_t ch_function, uint8_t ch_flag)
{

    switch (ch_function) {

    case AUXSW_SIMPLE_MODE:
        // low = simple mode off, middle or high position turns simple mode on
        set_simple_mode(ch_flag == AUX_SWITCH_HIGH || ch_flag == AUX_SWITCH_MIDDLE);
        break;

    case AUXSW_SUPERSIMPLE_MODE:
        // low = simple mode off, middle = simple mode, high = super simple mode
        set_simple_mode(ch_flag);
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
            if (control_mode == AUTO || !motors.armed()) {
                return;
            }

            // do not allow saving the first waypoint with zero throttle
            if ((mission.num_commands() == 0) && (channel_throttle->get_control_in() == 0)) {
                return;
            }

            // create new mission command
            AP_Mission::Mission_Command cmd  = {};

            // set new waypoint to current location
            cmd.content.location = current_loc;

            // if throttle is above zero, create waypoint command
            if (channel_throttle->get_control_in() > 0) {
                cmd.id = MAV_CMD_NAV_WAYPOINT;
            } else {
                // with zero throttle, create LAND command
                cmd.id = MAV_CMD_NAV_LAND;
            }

            // save command
            if (mission.add_cmd(cmd)) {
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
        // enable or disable the sonar
#if RANGEFINDER_ENABLED == ENABLED
        if ((ch_flag == AUX_SWITCH_HIGH) && (rangefinder.num_sensors() >= 1)) {
            rangefinder_state.enabled = true;
        } else {
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
        } else {
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
        switch (ch_flag) {
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
#if GRIPPER_ENABLED == ENABLED
    case AUXSW_GRIPPER:
        switch (ch_flag) {
        case AUX_SWITCH_LOW:
            g2.gripper.release();
            Log_Write_Event(DATA_GRIPPER_RELEASE);
            break;
        case AUX_SWITCH_HIGH:
            g2.gripper.grab();
            Log_Write_Event(DATA_GRIPPER_GRAB);
            break;
        }
        break;
#endif

    case AUXSW_AUTO:
        if (ch_flag == AUX_SWITCH_HIGH) {
            set_mode(AUTO, MODE_REASON_TX_COMMAND);
        } else {
            // return to flight mode switch's flight mode if we are currently in AUTO
            if (control_mode == AUTO) {
                //                    reset_control_switch();
            }
        }
        break;

#if AUTOTUNE_ENABLED == ENABLED
    case AUXSW_AUTOTUNE:
        // turn on auto tuner
        switch (ch_flag) {
        case AUX_SWITCH_LOW:
        case AUX_SWITCH_MIDDLE:
            // restore flight mode based on flight mode switch position
            if (control_mode == AUTOTUNE) {
                //                        reset_control_switch();
            }
            break;
        case AUX_SWITCH_HIGH:
            // start an autotuning session
            set_mode(AUTOTUNE);
            break;
        }
        break;
#endif

    case AUXSW_LAND:
        // Do nothing for Sub

        //            if (ch_flag == AUX_SWITCH_HIGH) {
        //                set_mode(LAND, MODE_REASON_TX_COMMAND);
        //            }else{
        //                // return to flight mode switch's flight mode if we are currently in LAND
        //                if (control_mode == LAND) {
        //                    reset_control_switch();
        //                }
        //            }
        break;

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

        // Log new status
        if (motors.get_interlock()) {
            Log_Write_Event(DATA_MOTORS_INTERLOCK_ENABLED);
        } else {
            Log_Write_Event(DATA_MOTORS_INTERLOCK_DISABLED);
        }
        break;
    }
}

// save_trim - adds roll and pitch trims from the radio to ahrs
void Sub::save_trim()
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
void Sub::auto_trim()
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

