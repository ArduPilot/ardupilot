// 2010 Jose Julio
// 2011 Adapted and updated for AC2 by Jason Short
//
// Automatic Acrobatic Procedure (AAP) v1 : Roll flip
// State machine aproach:
//    Some states are fixed commands (for a fixed time)
//    Some states are fixed commands (until some IMU condition)
//    Some states include controls inside
uint8_t flip_timer;
uint8_t flip_state;

#define AAP_THR_INC 170
#define AAP_THR_DEC 120
#define AAP_ROLL_OUT 2000

static int8_t flip_dir;

void init_flip()
{
    if(false == ap.do_flip) {
        ap.do_flip = true;
        flip_state = 0;
        flip_dir = (ahrs.roll_sensor >= 0) ? 1 : -1;
		Log_Write_Event(DATA_BEGIN_FLIP);
    }
}

void roll_flip()
{
    // Pitch
    //g.rc_2.servo_out = get_stabilize_pitch(g.rc_2.control_in);
    get_stabilize_pitch(g.rc_2.control_in);

    int32_t roll = ahrs.roll_sensor * flip_dir;

    // Roll State machine
    switch (flip_state) {
    case 0:
        if (roll < 4500) {
            // Roll control
            g.rc_1.servo_out = AAP_ROLL_OUT * flip_dir;
            set_throttle_out(g.rc_3.control_in + AAP_THR_INC, false);
        }else{
            flip_state++;
        }
        break;

    case 1:
        if((roll >= 4500) || (roll < -9000)) {
		#if FRAME_CONFIG == HELI_FRAME
			g.rc_1.servo_out = get_heli_rate_roll(40000 * flip_dir);
		#else
            g.rc_1.servo_out = get_rate_roll(40000 * flip_dir);
		#endif // HELI_FRAME
        set_throttle_out(g.rc_3.control_in - AAP_THR_DEC, false);
        }else{
            flip_state++;
            flip_timer = 0;
        }
        break;

    case 2:
        if (flip_timer < 100) {
            //g.rc_1.servo_out = get_stabilize_roll(g.rc_1.control_in);
            get_stabilize_roll(g.rc_1.control_in);
            set_throttle_out(g.rc_3.control_in + AAP_THR_INC, false);
            flip_timer++;
        }else{
        	Log_Write_Event(DATA_END_FLIP);
            ap.do_flip = false;
            flip_state = 0;
        }
        break;
    }
}
