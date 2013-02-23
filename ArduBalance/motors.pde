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
	if(tilt_start == false){
		APM_RC.OutputCh(CH_1, 0);
		APM_RC.OutputCh(CH_2, 0);
		reset_I_all();
		return;
	}

    uint8_t dir_left;
    uint8_t dir_right;

#if USE_WHEEL_LUT == ENABLED
	/*
    motor_out[LEFT_MOT_CH]  = get_pwm_from_speed_wheel_mixer_left(); // left motor
    motor_out[RIGHT_MOT_CH] = get_pwm_from_speed_wheel_mixer_right(); // righ motor

    motor_out[LEFT_MOT_CH]  = constrain(motor_out[LEFT_MOT_CH],  -2000, 2000);
    motor_out[RIGHT_MOT_CH] = constrain(motor_out[RIGHT_MOT_CH], -2000, 2000);

    dir_left 	= (motor_out[LEFT_MOT_CH]  < 0) ? LOW : HIGH;
    dir_right 	= (motor_out[RIGHT_MOT_CH] < 0) ? LOW : HIGH;

	APM_RC.OutputCh(CH_1, abs(motor_out[LEFT_MOT_CH])); // left motor
	APM_RC.OutputCh(CH_2, abs(motor_out[RIGHT_MOT_CH])); // right motor
	*/
#else
    motor_out[LEFT_MOT_CH]  = (float)(pitch_speed + yaw_speed) * g.pid_wheel_left_mixer.kP(); // left motor
    motor_out[RIGHT_MOT_CH] = (float)(pitch_speed - yaw_speed) * g.pid_wheel_right_mixer.kP(); // righ motor

    motor_out[LEFT_MOT_CH]  = constrain(motor_out[LEFT_MOT_CH],  -2000, 2000);
    motor_out[RIGHT_MOT_CH] = constrain(motor_out[RIGHT_MOT_CH], -2000, 2000);

    dir_left 	= (motor_out[LEFT_MOT_CH]  < 0) ? LOW : HIGH;
    dir_right 	= (motor_out[RIGHT_MOT_CH] < 0) ? LOW : HIGH;

	APM_RC.OutputCh(CH_1, abs(motor_out[LEFT_MOT_CH])  + g.dead_zone); // left motor
	APM_RC.OutputCh(CH_2, abs(motor_out[RIGHT_MOT_CH]) + g.dead_zone); // right motor
#endif

    digitalWriteFast(COPTER_LED_2, dir_left); // left
    digitalWriteFast(COPTER_LED_1, dir_right); // right
}

static void
set_servos_direct(int16_t pwm)
{
    uint8_t dir_left;
    uint8_t dir_right;

#if USE_WHEEL_LUT == ENABLED
    motor_out[0] = get_pwm_from_speed_wheel_mixer_left();
    motor_out[1] = get_pwm_from_speed_wheel_mixer_right();
#else
    motor_out[0] = pwm;
    motor_out[1] = pwm;
#endif

    dir_left 	= (motor_out[LEFT_MOT_CH]  < 0) ? LOW : HIGH;
    dir_right 	= (motor_out[RIGHT_MOT_CH] < 0) ? LOW : HIGH;

	APM_RC.OutputCh(CH_1, abs(motor_out[LEFT_MOT_CH])); // left motor
	APM_RC.OutputCh(CH_2, abs(motor_out[RIGHT_MOT_CH])); // right motor

    digitalWriteFast(COPTER_LED_2, dir_left); // left
    digitalWriteFast(COPTER_LED_1, dir_right); // right
}

