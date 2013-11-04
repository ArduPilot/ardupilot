#if GIMBAL == ENABLED

static void
setup_gimbal_control()
{
	hal.rcout->enable_ch(CH_11);
}

static void
update_gimbal_control()
{
	// are we in manual control?
	switch (yaw_mode){


		case YAW_LOOK_AT_LOCATION:
		case YAW_CIRCLE:
		case YAW_LOOK_AT_HOME:
			output_gimbal_pwm();
		break;

		default:
			//send CH 6 out to gimbal
			if(g.radio_tuning == 0)
	   			hal.rcout->write(CH_11, g.rc_6.radio_in);
	   		else
	   			hal.rcout->write(CH_11, 1500); // ??
		break;

	}
}


static void
output_gimbal_pwm()
{
	Vector3f position = inertial_nav.get_position();

    float deltaX = yaw_look_at_WP.x - position.x;
    float deltaY = yaw_look_at_WP.y - position.y;
	float wp_distance = safe_sqrt(deltaX * deltaX + deltaY * deltaY);
	float angle = atan2(position.z, wp_distance);
	angle = constrain_float(angle, 0, 1.57); // 0 to 90

	hal.rcout->write(CH_11, (1000 + angle * 636.94));
}


/*
static void
output_gimbal_pwm_test()
{
	Vector3f position;
	position.x = 0;
	position.y = 0;
	position.z = 100;

	yaw_look_at_WP.x = 0;
	yaw_look_at_WP.y = 200;
	yaw_look_at_WP.z = 0;

    float deltaX = yaw_look_at_WP.x - position.x;
    float deltaY = yaw_look_at_WP.y - position.y;
	float wp_distance = safe_sqrt(deltaX * deltaX + deltaY * deltaY);

	float angle = atan2(position.z, wp_distance);
	//angle = constrain_float(angle, 0, 1.57); // 0 to 90

	int16_t output =  1000 + (angle * 636.94);
    cliSerial->printf_P(PSTR("out: %1.2f, %1.2f, %1.2f, angle: %1.2f, out: %d\n"),
					deltaX,
					deltaY,
					wp_distance,
					angle,
					output);

}
*/

#endif