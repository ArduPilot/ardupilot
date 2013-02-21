void init_balance()
{
    g_gps->longitude = 0;
    g_gps->latitude = 0;
    init_home();
}

// 815 ticks per revolution

int16_t get_pwm_from_speed_wheel_mixer_left()  // left motor
{
	//return pitch_rpm; // straight output
	//return convert_rpm_to_PWM(pitch_rpm); // corrected output

	int16_t wheel_speed_out   	= convert_RPM_to_encoder_speed(pitch_rpm + yaw_rpm);
	int16_t wheel_ff 			= convert_encoder_to_PWM(wheel_speed_out);
    int16_t wheel_pid 			= g.pid_wheel_left_mixer.get_pid((wheel_speed_out - wheel.left_speed), .02);

	return (wheel_ff + wheel_pid);
}

int16_t get_pwm_from_speed_wheel_mixer_right()  // right motor
{
	//return pitch_rpm; // straight output
	//return convert_rpm_to_PWM(pitch_rpm); // corrected output

	int16_t wheel_speed_out   	= convert_RPM_to_encoder_speed(pitch_rpm - yaw_rpm);
	int16_t wheel_ff 			= convert_encoder_to_PWM(wheel_speed_out);
	int16_t wheel_pid 			= g.pid_wheel_right_mixer.get_pid((wheel_speed_out - wheel.right_speed), .02);

	return (wheel_ff + wheel_pid);
}

void update_wheel_encoders(){

	uint8_t buff[15];

	//read(uint8_t address, uint8_t numberBytes, uint8_t *dataBuffer)
	if (I2c.read((uint8_t)ENCODER_ADDRESS, 15, buff) != 0) {
		Serial.println("blah");
		return;
	}

	memcpy(bytes_union.bytes, &buff[1], 4);
	wheel.left = bytes_union.int_value;

	memcpy(bytes_union.bytes, &buff[3], 4);
	wheel.right = bytes_union.int_value;

	memcpy(bytes_union.bytes, &buff[5], 2);
	wheel.left_speed = bytes_union.int_value * WHEEL_ENCODER_DIR_LEFT;

	memcpy(bytes_union.bytes, &buff[7], 2);
	wheel.right_speed = bytes_union.int_value * WHEEL_ENCODER_DIR_RIGHT;

	//wheel.speed 	= (wheel.left_speed + wheel.right_speed) >> 1;
	wheel.speed 	= wheel.left_speed + wheel.right_speed;

	wheel.odometer 	+= (wheel.left + wheel.right); //>> 1;

	//Serial.printf("left: %ld, right: %ld, lsp: %d, rsp: %d\n", wheel.left, wheel.right, wheel.left_speed, wheel.right_speed);
}



int16_t convert_rpm_to_PWM(int16_t rpm_speed)
{
	return convert_encoder_to_PWM(convert_RPM_to_encoder_speed(rpm_speed));
}

int16_t convert_RPM_to_encoder_speed(int16_t rpm)
{
	return ((int32_t)g.wheel_encoder * (int32_t)rpm) / 1000l;
}

int16_t convert_encoder_speed_to_RPM(int16_t encoder_speed)
{
	return ((int32_t)encoder_speed * 1000l) / (int32_t)g.wheel_encoder;
}

int16_t convert_groundspeed_to_RPM(int16_t groundspeed)
{
	return (groundspeed * 100000) / WHEEL_DIAMETER;
}

int16_t convert_encoder_to_PWM(int16_t encoder_speed){
	// perform LUT on encoder_speed number
	int16_t tmp1, tmp2;
	tmp1 	= constrain(encoder_speed, 0, 3600);
	tmp2	= tmp1 / 100;   // 0:9
	return pwm_LUT[tmp2] + (tmp1 - tmp2 * 100) * (pwm_LUT[tmp2 + 1] - pwm_LUT[tmp2]) / 100;
}
