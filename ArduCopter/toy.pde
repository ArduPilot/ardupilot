////////////////////////////////////////////////////////////////////////////////
// Toy Mode - THOR
////////////////////////////////////////////////////////////////////////////////
static bool CH7_toy_flag;

#if TOY_MIXER == TOY_LOOKUP_TABLE
static const int16_t toy_lookup[] = {
    186,    373,    558,    745,
    372,    745,    1117,   1490,
    558,    1118,   1675,   2235,
    743,    1490,   2233,   2980,
    929,    1863,   2792,   3725,
    1115,   2235,   3350,   4470,
    1301,   2608,   3908,   4500,
    1487,   2980,   4467,   4500,
    1673,   3353,   4500,   4500
};
#endif

//called at 10hz
void update_toy_throttle()
{
    /*
     *  // Disabled, now handled by TOY_A (Alt hold) and TOY_M (Manual throttle)
     *  if (false == CH6_toy_flag && g.rc_6.radio_in >= CH_6_PWM_TRIGGER){
     *       CH6_toy_flag = true;
     *       throttle_mode  = THROTTLE_MANUAL;
     *
     *  }else if (CH6_toy_flag && g.rc_6.radio_in < CH_6_PWM_TRIGGER){
     *       CH6_toy_flag = false;
     *       throttle_mode  = THROTTLE_AUTO;
     *       set_new_altitude(current_loc.alt);
     *       saved_toy_throttle = g.rc_3.control_in;
     *  }*/

    // look for a change in throttle position to exit throttle hold
    if(abs(g.rc_3.control_in - saved_toy_throttle) > 40) {
        throttle_mode   = THROTTLE_MANUAL;
    }
}

#define TOY_ALT_SMALL 25
#define TOY_ALT_LARGE 100

//called at 10hz
void update_toy_altitude()
{
    int16_t input = g.rc_3.radio_in;     // throttle
    //int16_t input = g.rc_7.radio_in;

    // Trigger upward alt change
    if(false == CH7_toy_flag && input > 1666) {
        CH7_toy_flag = true;
        // go up
        if(next_WP.alt >= 400) {
            force_new_altitude(next_WP.alt + TOY_ALT_LARGE);
        }else{
            force_new_altitude(next_WP.alt + TOY_ALT_SMALL);
        }

        // Trigger downward alt change
    }else if(false == CH7_toy_flag && input < 1333) {
        CH7_toy_flag = true;
        // go down
        if(next_WP.alt >= (400 + TOY_ALT_LARGE)) {
            force_new_altitude(next_WP.alt - TOY_ALT_LARGE);
        }else if(next_WP.alt >= TOY_ALT_SMALL) {
            force_new_altitude(next_WP.alt - TOY_ALT_SMALL);
        }else if(next_WP.alt < TOY_ALT_SMALL) {
            force_new_altitude(0);
        }

        // clear flag
    }else if (CH7_toy_flag && ((input < 1666) && (input > 1333))) {
        CH7_toy_flag = false;
    }
}

// called at 50 hz from all flight modes
#if TOY_EDF == ENABLED
void edf_toy()
{
    // EDF control:
    g.rc_8.radio_out = 1000 + ((abs(g.rc_2.control_in) << 1) / 9);
    if(g.rc_8.radio_out < 1050)
        g.rc_8.radio_out = 1000;

    // output throttle to EDF
    if(motors.armed()) {
        APM_RC.OutputCh(CH_8, g.rc_8.radio_out);
    }else{
        APM_RC.OutputCh(CH_8, 1000);
    }

    // output Servo direction
    if(g.rc_2.control_in > 0) {
        APM_RC.OutputCh(CH_6, 1000);         // 1000 : 2000
    }else{
        APM_RC.OutputCh(CH_6, 2000);         // less than 1450
    }
}
#endif

// The function call for managing the flight mode Toy
void roll_pitch_toy()
{
#if TOY_MIXER == TOY_LOOKUP_TABLE || TOY_MIXER == TOY_LINEAR_MIXER
    int16_t yaw_rate = g.rc_1.control_in / g.toy_yaw_rate;

    if(g.rc_1.control_in != 0) {    // roll
        get_acro_yaw(yaw_rate/2);
        ap_system.yaw_stopped = false;
        yaw_timer = 150;

    }else if (!ap_system.yaw_stopped) {
        get_acro_yaw(0);
        yaw_timer--;

        if((yaw_timer == 0) || (fabs(omega.z) < .17)) {
            ap_system.yaw_stopped = true;
            nav_yaw = ahrs.yaw_sensor;
        }
    }else{
        if(motors.armed() == false || g.rc_3.control_in == 0)
            nav_yaw = ahrs.yaw_sensor;

        get_stabilize_yaw(nav_yaw);
    }
#endif

    // roll_rate is the outcome of the linear equation or lookup table
    // based on speed and Yaw rate
    int16_t roll_rate = 0;

#if TOY_MIXER == TOY_LOOKUP_TABLE
    uint8_t xx, yy;
    // Lookup value
    //xx	= g_gps->ground_speed / 200;
    xx      = abs(g.rc_2.control_in / 1000);
    yy      = abs(yaw_rate / 500);

    // constrain to lookup Array range
    xx = constrain(xx, 0, 3);
    yy = constrain(yy, 0, 8);

    roll_rate = toy_lookup[yy * 4 + xx];

    if(yaw_rate == 0) {
        roll_rate = 0;
    }else if(yaw_rate < 0) {
        roll_rate = -roll_rate;
    }

    int16_t roll_limit = 4500 / g.toy_yaw_rate;
    roll_rate = constrain(roll_rate, -roll_limit, roll_limit);

#elif TOY_MIXER == TOY_LINEAR_MIXER
    roll_rate = -((int32_t)g.rc_2.control_in * (yaw_rate/100)) /30;
    //cliSerial->printf("roll_rate: %d\n",roll_rate);
    roll_rate = constrain(roll_rate, -2000, 2000);

#elif TOY_MIXER == TOY_EXTERNAL_MIXER
    // JKR update to allow external roll/yaw mixing
    roll_rate = g.rc_1.control_in;
#endif

#if TOY_EDF == ENABLED
    // Output the attitude
    //g.rc_1.servo_out = get_stabilize_roll(roll_rate);
    //g.rc_2.servo_out = get_stabilize_pitch(g.rc_6.control_in);             // use CH6 to trim pitch
    get_stabilize_roll(roll_rate);
    get_stabilize_pitch(g.rc_6.control_in);             // use CH6 to trim pitch

#else
    // Output the attitude
    //g.rc_1.servo_out = get_stabilize_roll(roll_rate);
    //g.rc_2.servo_out = get_stabilize_pitch(g.rc_2.control_in);
    get_stabilize_roll(roll_rate);
    get_stabilize_pitch(g.rc_2.control_in);
#endif

}
