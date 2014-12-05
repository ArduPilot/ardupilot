/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// read_3pos_switch
static uint8_t read_3pos_switch(int16_t radio_in){
    if (radio_in < AUX_SWITCH_PWM_TRIGGER_LOW) return AUX_SWITCH_LOW;      // switch is in low position
    if (radio_in > AUX_SWITCH_PWM_TRIGGER_HIGH) return AUX_SWITCH_HIGH;    // switch is in high position
    return AUX_SWITCH_MIDDLE;                                       // switch is in middle position
}

// read_aux_switches - checks aux switch positions and invokes configured actions
static void read_aux_switches()
{
    uint8_t switch_position;

    // exit immediately during radio failsafe
    if (failsafe.state != FAILSAFE_NONE) {
        return;
    }

    // check if ch5 switch has changed position
    switch_position = read_3pos_switch(g.rc_5.radio_in);
    if (aux.CH5_flag != switch_position) {
        // set the CH5 flag
        aux.CH5_flag = switch_position;

        // invoke the appropriate function
        do_aux_switch_function(g.ch5_option, aux.CH5_flag);
    }

    // check if Ch6 switch has changed position
    switch_position = read_3pos_switch(g.rc_6.radio_in);
    if (aux.CH6_flag != switch_position) {
        // set the CH6 flag
        aux.CH6_flag = switch_position;

        // invoke the appropriate function
        do_aux_switch_function(g.ch6_option, aux.CH6_flag);
    }
    
    // check if Ch7 switch has changed position
    switch_position = read_3pos_switch(g.rc_7.radio_in);
    if (aux.CH7_flag != switch_position) {
        // set the CH7 flag
        aux.CH7_flag = switch_position;

        // invoke the appropriate function
        do_aux_switch_function(g.ch7_option, aux.CH7_flag);
    }    
}

// init_aux_switches - invoke configured actions at start-up for aux function where it is safe to do so
static void init_aux_switches()
{
    // set the CH5, CH6 and CH7 flags
    aux.CH5_flag = read_3pos_switch(g.rc_5.radio_in);
    aux.CH6_flag = read_3pos_switch(g.rc_6.radio_in);
    aux.CH7_flag = read_3pos_switch(g.rc_7.radio_in);
    
    // init channel 5 options
    switch(g.ch5_option) {
        case AUX_SWITCH_CAMERA_TRIGGER:
        case AUX_SWITCH_PARACHUTE_ENABLE:
        case AUX_SWITCH_PARACHUTE_RELEASE:
        case AUX_SWITCH_PARACHUTE_3POS:	    // we trust the vehicle will be disarmed so even if switch is in release position the chute will not release
        case AUX_SWITCH_RETRACT_MOUNT:
        case AUX_SWITCH_RELAY_1:
        case AUX_SWITCH_RELAY_2:
        case AUX_SWITCH_RELAY_3:
        case AUX_SWITCH_RELAY_4:        
            do_aux_switch_function(g.ch5_option, aux.CH5_flag);
            break;
    }

    // init channel 6 option
    switch(g.ch6_option) {
        case AUX_SWITCH_CAMERA_TRIGGER:      
        case AUX_SWITCH_PARACHUTE_ENABLE:
        case AUX_SWITCH_PARACHUTE_RELEASE:
        case AUX_SWITCH_PARACHUTE_3POS:     // we trust the vehicle will be disarmed so even if switch is in release position the chute will not release      
        case AUX_SWITCH_RETRACT_MOUNT:
        case AUX_SWITCH_RELAY_1:
        case AUX_SWITCH_RELAY_2:
        case AUX_SWITCH_RELAY_3:
        case AUX_SWITCH_RELAY_4:
            do_aux_switch_function(g.ch6_option, aux.CH6_flag);
            break;
    }
    
    // init channel 7 option
    switch(g.ch7_option) {
        case AUX_SWITCH_CAMERA_TRIGGER:      
        case AUX_SWITCH_PARACHUTE_ENABLE:
        case AUX_SWITCH_PARACHUTE_RELEASE:
        case AUX_SWITCH_PARACHUTE_3POS:     // we trust the vehicle will be disarmed so even if switch is in release position the chute will not release
        case AUX_SWITCH_RETRACT_MOUNT:
        case AUX_SWITCH_RELAY_1:
        case AUX_SWITCH_RELAY_2:
        case AUX_SWITCH_RELAY_3:
        case AUX_SWITCH_RELAY_4:
            do_aux_switch_function(g.ch7_option, aux.CH7_flag);
            break;
    }    
}

// do_aux_switch_function - implement the function invoked by the ch5, ch6 or ch7 switch
static void do_aux_switch_function(int8_t ch_function, uint8_t ch_flag)
{
    int8_t tmp_function = ch_function;

    switch(tmp_function) {

#if CAMERA == ENABLED
        case AUX_SWITCH_CAMERA_TRIGGER:
            if (ch_flag == AUX_SWITCH_HIGH) {
                do_take_picture();
            }
            break;
#endif

#if PARACHUTE == ENABLED
    case AUX_SWITCH_PARACHUTE_ENABLE:
        // Parachute enable/disable
        parachute.enabled(ch_flag == AUX_SWITCH_HIGH);
        break;

    case AUX_SWITCH_PARACHUTE_RELEASE:
        if (ch_flag == AUX_SWITCH_HIGH) {
            parachute_manual_release();
        }
        break;

    case AUX_SWITCH_PARACHUTE_3POS:
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
        
#if MOUNT == ENABLE
    case AUX_SWITCH_RETRACT_MOUNT:
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

    case AUX_SWITCH_RELAY_1:
        ServoRelayEvents.do_set_relay(0, ch_flag == AUX_SWITCH_HIGH);
        break;
        
    case AUX_SWITCH_RELAY_2:
        ServoRelayEvents.do_set_relay(1, ch_flag == AUX_SWITCH_HIGH);
        break;
        
    case AUX_SWITCH_RELAY_3:
        ServoRelayEvents.do_set_relay(2, ch_flag == AUX_SWITCH_HIGH);
        break;
        
    case AUX_SWITCH_RELAY_4:
        ServoRelayEvents.do_set_relay(3, ch_flag == AUX_SWITCH_HIGH);
        break;        
    }
}

