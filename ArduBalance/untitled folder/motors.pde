/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

static void init_motors_out()
{
    APM_RC.SetFastOutputChannels(_BV(CH_1) | _BV(CH_2) | _BV(CH_3) | _BV(CH_4), g.rc_speed);
}

static void motors_output_enable()
{
    APM_RC.enable_out(CH_1);
    APM_RC.enable_out(CH_2);
    APM_RC.enable_out(CH_3);
    APM_RC.enable_out(CH_4);
    APM_RC.enable_out(CH_5);
    APM_RC.enable_out(CH_6);
}


static void init_disarm_motors()
{

}

static void init_arm_motors()
{


}
/*****************************************
 * Set the flight control servos based on the current calculated values
 *****************************************/
static void
update_servos()
{

/*
	L     R
    -     +
    0 ^^^ 1
    +     -
*/

	/*if(labs(ahrs.pitch_sensor) > 3000 || labs(ahrs.roll_sensor) > 3000){
		APM_RC.OutputCh(CH_1, 0);
		APM_RC.OutputCh(CH_2, 0);
	}*/

    static int8_t counter = 0;
    counter++;

    int8_t dir_left, dir_right;


    motor_out[LEFT_MOT_CH]  = get_pwm_from_speed_wheel_mixer_left(); // left motor
    motor_out[RIGHT_MOT_CH] = get_pwm_from_speed_wheel_mixer_right(); // righ motor


    motor_out[LEFT_MOT_CH]  = constrain(motor_out[LEFT_MOT_CH] ,  -2000, 2000);
    motor_out[RIGHT_MOT_CH] = constrain(motor_out[RIGHT_MOT_CH] , -2000, 2000);

    //motor_out[0] = convert_rpm_to_PWM(pitch_rpm); // corrected output
    //motor_out[RIGHT_MOT_CH] = convert_rpm_to_PWM(pitch_rpm); // corrected output

    //dir_left 	= (motor_out[LEFT_MOT_CH] <= 0)  ? FORWARD : BACKWARD;
    //dir_right 	= (motor_out[RIGHT_MOT_CH] <= 0) ? BACKWARD : FORWARD;

//#define FORWARD LOW
//#define BACKWARD HIGH works

    dir_right	= LOW;
    dir_left	= LOW;

    if(counter == 10){
        counter = 0;
        ///*
        Serial.printf("pitch:%ld p_rpm:%d, y_rpm:%d \tmL:%d, mR:%d, \n",
        		ahrs.pitch_sensor,
        		pitch_rpm,
        		yaw_rpm,
        		motor_out[LEFT_MOT_CH],
        		motor_out[RIGHT_MOT_CH]);
        //*/
    }

	APM_RC.OutputCh(CH_1, abs(motor_out[LEFT_MOT_CH])); // left motor
	APM_RC.OutputCh(CH_2, abs(motor_out[RIGHT_MOT_CH])); // right motor

    digitalWrite(COPTER_LED_1, dir_left);
    digitalWrite(COPTER_LED_2, dir_right);
}

static void
set_servos_direct(int16_t pwm)
{
    int8_t dir1 = BACKWARD;
    int8_t dir2 = FORWARD;

    //motor_out[0] = get_pwm_from_speed_wheel_mixer_left();
    //motor_out[1] = get_pwm_from_speed_wheel_mixer_right();

    motor_out[0] = pwm;
    motor_out[1] = pwm;


    dir1 = (motor_out[0] <= 0) ? BACKWARD : FORWARD;
    dir2 = (motor_out[1] <= 0) ? FORWARD : BACKWARD;

	APM_RC.OutputCh(CH_1, abs(motor_out[0]));
	APM_RC.OutputCh(CH_2, abs(motor_out[1]));

    digitalWrite(COPTER_LED_1, dir1);
    digitalWrite(COPTER_LED_2, dir2);
}

