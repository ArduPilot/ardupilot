// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_manual.pde - manual control mode
 */

/*
 * update_manual - runs the manual controller
 *  called at 50hz while control_mode is 'MANUAL'
 */
static void update_manual(void)
{
    // copy yaw and pitch input to output
    channel_yaw.radio_out = constrain_int16(channel_yaw.radio_in, channel_yaw.radio_min, channel_yaw.radio_max);
    channel_pitch.radio_out = constrain_int16(channel_pitch.radio_in, channel_pitch.radio_min, channel_pitch.radio_max);

    // send output to servos
    channel_yaw.output();
    channel_pitch.output();
}

/*
 * update_manual_angle - runs the manual controller in angle mode
 * called at 50hz while control_mode is 'MANUAL'
 * 
 * This function converts RC inputs into nav_pitch and nav_bearing
 */
static void update_manual_angle(void)
{
    int yaw_in;
    int pitch_in;
    
    pitch_in = constrain_int16(channel_pitch.radio_in, channel_pitch.radio_min, channel_pitch.radio_max);
    yaw_in = constrain_int16(channel_yaw.radio_in, channel_yaw.radio_min, channel_yaw.radio_max);
     
    // rc input to angle conversion
    nav_status.bearing = (float)(yaw_in - channel_yaw.radio_min)/(float)(channel_yaw.radio_max - channel_yaw.radio_min) * 360.0;
    nav_status.pitch = ( (float)(pitch_in - channel_pitch.radio_min)/(float)(channel_pitch.radio_max - channel_pitch.radio_min) * 180.0) - 90.0;
    
    float yaw = wrap_180_cd((nav_status.bearing+g.yaw_trim)*100) * 0.01f;
    //float yaw = nav_status.bearing
    float pitch = constrain_float(nav_status.pitch+g.pitch_trim, -90, 90);

    if (pitch > g.pitch_range)
    {
      pitch = g.pitch_range;
    }
    if (pitch < -g.pitch_range)
    {
      pitch = -g.pitch_range;
    }

    update_pitch_servo(pitch);
    update_yaw_servo(yaw);
    
}