void init_balance()
{
	wheel_ratio 		= 1000.0 / g.wheel_encoder_speed;
	tilt_start 			= false;
    g_gps->longitude 	= 0;
    g_gps->latitude 	= 0;
    fail 				= 0;
    init_home();

    // init Yaw hold
    nav_yaw     		= ahrs.yaw_sensor;
}

// 815 ticks per revolution

int16_t get_pwm_from_speed_wheel_mixer_left()  // left motor
{
	// mix output speeds
	wheel.left_speed_output = (pitch_speed + yaw_speed);

	// Lookup our PWM output:
	int16_t wheel_ff		= convert_speed_to_PWM(pwm_LUT_L, wheel.left_speed_output);
	int16_t speed_err 		= wheel.left_speed_output - wheel.left_speed;

    int16_t wheel_P 		= g.pid_wheel_left_mixer.get_p(speed_err);
    int16_t wheel_I 		= g.pid_wheel_left_mixer.get_i(speed_err, .02);
    int16_t wheel_D 		= g.pid_wheel_left_mixer.get_d(speed_err, .02);

	/*cliSerial->printf_P(PSTR("%d, %d, %d, %d\n"),
				wheel.left_speed,
				speed_err,
				wheel_ff,
				wheel_pid);
	//*/

	return (wheel_ff + wheel_P + wheel_I + wheel_D);
}



int16_t get_pwm_from_speed_wheel_mixer_right()  // right motor
{
	// mix output speeds
	wheel.right_speed_output 	= (pitch_speed - yaw_speed);

	// Lookup our PWM output:
	int16_t wheel_ff			= convert_speed_to_PWM(pwm_LUT_R, wheel.right_speed_output);	// Lookup our PWM output:
	int16_t speed_err 			= wheel.right_speed_output - wheel.right_speed;

	int16_t wheel_P 			= g.pid_wheel_right_mixer.get_p(speed_err);
    int16_t wheel_I 			= g.pid_wheel_right_mixer.get_i(speed_err, .02);
    int16_t wheel_D 			= g.pid_wheel_right_mixer.get_d(speed_err, .02);

	int16_t output = (wheel_ff + wheel_P + wheel_I + wheel_D);


	/*
							//1   2   3   4   5   6
	cliSerial->printf_P(PSTR("%d, %d, %d, %d, %d, %d\n"),
					(int16_t)ahrs.pitch_sensor,			// 1
					pitch_speed,						// 2
					wheel.right_speed,					// 3
					speed_err,							// 4
					wheel_ff,							// 5
					output);							// 6
	//*/
	return output;
}

void update_wheel_encoders(){

	uint8_t buff[15];

	//read(uint8_t address, uint8_t numberBytes, uint8_t *dataBuffer)
	if (I2c.read((uint8_t)ENCODER_ADDRESS, 15, buff) != 0) {
		fail++;
		return;
	}

	memcpy(bytes_union.bytes, &buff[1], 4);
	wheel.left_distance = bytes_union.int_value * WHEEL_ENCODER_DIR_LEFT;

	memcpy(bytes_union.bytes, &buff[3], 4);
	wheel.right_distance = bytes_union.int_value * WHEEL_ENCODER_DIR_RIGHT;

	memcpy(bytes_union.bytes, &buff[5], 2);
	wheel.left_speed = bytes_union.int_value * WHEEL_ENCODER_DIR_LEFT;

	memcpy(bytes_union.bytes, &buff[7], 2);
	wheel.right_speed = bytes_union.int_value * WHEEL_ENCODER_DIR_RIGHT;

	wheel.speed 	= wheel.speed + ((wheel.left_speed + wheel.right_speed) >> 1);
	wheel.speed 	>>= 1;

	ground_speed 	= convert_encoder_speed_to_ground_speed(wheel.speed);

	// convert wheel.speed to 1rps = 1000
	// 815 * 1.2 = 1000;
	wheel.speed = (float)wheel.speed * wheel_ratio;

	float delta   = (float)(wheel.left_distance + wheel.right_distance) / 2;	// correct
	//int16_t delta   = wheel.right_distance;	// testing

	// using the mm accuracy of the encoders to get an overall location
 	current_encoder_x += cos_yaw_x * delta;
	current_encoder_y += sin_yaw_y * delta;

	if(gps_available == false){
		// scaling the mm accuracy to cm
		current_loc.lng	 = current_encoder_x / 10;
		current_loc.lat  = current_encoder_y / 10;
	} else {
		current_loc.lng	 = ((float)g_gps->longitude * .01) + (current_encoder_x / 10) * .99;
		current_loc.lat  = ((float)g_gps->latitude  * .01) + (current_encoder_y / 10) * .99;
	}

	//if(tilt_start)
		//cliSerial->printf_P(PSTR("y%ld, X:%d, Y:%d \n"), ahrs.yaw_sensor, (int16_t)current_encoder_x, (int16_t)current_encoder_y);

	//Serial.printf("left: %ld, right: %ld, lsp: %d, rsp: %d\n", wheel.left, wheel.right, wheel.left_speed, wheel.right_speed);
}

// ------------------

int16_t convert_groundspeed_to_encoder_speed(int16_t _ground_speed)
{
	return ((int32_t)_ground_speed * (int32_t)g.wheel_encoder_speed ) / WHEEL_DIAMETER_CM;
}

int32_t convert_distance_to_encoder_speed(int32_t _distance)
{
	return ((int32_t)_distance * (int32_t)g.wheel_encoder_speed ) / WHEEL_DIAMETER_CM;
}

int16_t convert_encoder_speed_to_ground_speed(int16_t encoder_speed)
{
	return ((float)encoder_speed * WHEEL_DIAMETER_CM) / (float)g.wheel_encoder_speed;
}

// ------------------

int16_t convert_speed_to_PWM(int16_t lut[], int16_t encoder_speed){
	int8_t flip_sign = (encoder_speed < 0) ? -1 : 1;

	uint16_t input;
	uint8_t index;

	if(abs(encoder_speed) > (PWM_LUT_SIZE * 100)){
		return lut[PWM_LUT_SIZE-1] * flip_sign;
	}

	input 	= constrain(abs(encoder_speed), 0, (PWM_LUT_SIZE * 100));
	index	= input / 100;   //

	int16_t output = lut[index] + ((input - index * 100) * (lut[index + 1] - lut[index])) / 100;
	return output * flip_sign;
}
