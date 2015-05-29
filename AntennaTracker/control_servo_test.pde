// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_servo_test.pde - GCS controlled servo test mode
 */

/*
 * update_servo_test - runs the servo test controller
 *  called at 50hz while control_mode is 'SERVO_TEST' mode
 *  tracker switches into this mode if it ever receives a do-set-servo command from the GCS
 */
static void update_servo_test(void)
{
    
    // CR Servo test routine
    //
    // Runs at 50Hz to keep servos pointed at set angles
    
    
    if ( (enum ServoType)g.servo_type.get() == SERVO_TYPE_CR)
    {
        
        float yaw = wrap_180_cd((nav_status.bearing+g.yaw_trim)*100) * 0.01f;
        update_yaw_servo(yaw);
        
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
      
    }
}

/*
 * servo_test_set_servo - sets the yaw or pitch servo pwm directly
 *  servo_num are 1 for yaw, 2 for pitch
 */
static bool servo_test_set_servo(uint8_t servo_num, uint16_t pwm)
{
    // convert servo_num from 1~2 to 0~1 range
    servo_num--;

    // exit immediately if servo_num is invalid
    if (servo_num != CH_YAW && servo_num != CH_PITCH) {
        return false;
    }

    // ensure we are in servo test mode
    if (control_mode != SERVO_TEST) {
        set_mode(SERVO_TEST);
    }

    // set yaw servo pwm and send output to servo
    if (servo_num == CH_YAW && (enum ServoType)g.servo_type.get() != SERVO_TYPE_CR) {
        channel_yaw.radio_out = constrain_int16(pwm, channel_yaw.radio_min, channel_yaw.radio_max);
        channel_yaw.output();
    }

    // set pitch servo pwm and send output to servo
    if (servo_num == CH_PITCH && (enum ServoType)g.servo_type.get() != SERVO_TYPE_CR) {
        channel_pitch.radio_out = constrain_int16(pwm, channel_pitch.radio_min, channel_pitch.radio_max);
        channel_pitch.output();
    }
    
    // CR servo test
    int yaw_in;
    int pitch_in;
    
    pitch_in = constrain_int16(pwm, channel_pitch.radio_min, channel_pitch.radio_max);
    yaw_in = constrain_int16(pwm, channel_yaw.radio_min, channel_yaw.radio_max);
    
    if (servo_num == CH_YAW && (enum ServoType)g.servo_type.get() == SERVO_TYPE_CR)
    {
        nav_status.bearing = (float)(yaw_in - channel_yaw.radio_min)/(float)(channel_yaw.radio_max - channel_yaw.radio_min) * 360.0;
    }
    
    if (servo_num == CH_PITCH && (enum ServoType)g.servo_type.get() == SERVO_TYPE_CR)
    {
        nav_status.pitch = ( (float)(pitch_in - channel_pitch.radio_min)/(float)(channel_pitch.radio_max - channel_pitch.radio_min) * 180.0) - 90.0;
    }
    
    // return success
    return true;
}
