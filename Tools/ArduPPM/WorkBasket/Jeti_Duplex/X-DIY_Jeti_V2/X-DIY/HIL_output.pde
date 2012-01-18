// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#if ENABLE_HIL
// An Xplane/Flightgear output
// Use with the GPS_IMU to do Hardware in teh loop simulations

byte buf_len = 0;
byte out_buffer[32];

void output_HIL(void)
{
	// output real-time sensor data
	Serial.print("AAA"); 		 						// 		Message preamble
	output_int((int)(servo_out[CH_ROLL]));				// 	0	bytes 0, 1
	output_int((int)(servo_out[CH_PITCH]));				// 	1	bytes 2, 3
	output_int((int)(servo_out[CH_THROTTLE]));			// 	2	bytes 4, 5
	output_int((int)(servo_out[CH_RUDDER]));			// 	3	bytes 6, 7
	output_int((int)wp_distance);						// 	4	bytes 8,9
	output_int((int)bearing_error);						// 	5	bytes 10,11
	output_int((int)next_WP.alt / 100);					// 	6	bytes 12, 13
	output_int((int)energy_error);						// 	7	bytes 14,15
	output_byte(wp_index);								// 	8	bytes 16
	output_byte(control_mode);							// 	9	bytes 17
	
	// print out the buffer and checksum
	// ---------------------------------
	print_buffer();
}

// This is for debugging only!, 
// I just move the underscore to keep the above version pristene.
void output_HIL_(void)
{
	// output real-time sensor data
	Serial.print("AAA"); 		 						// 		Message preamble
	output_int((int)(servo_out[CH_ROLL]));				// 	0	bytes 0, 1
	output_int((int)(servo_out[CH_PITCH]));				// 	1	bytes 2, 3
	output_int((int)(servo_out[CH_THROTTLE]));			// 	2	bytes 4, 5
	output_int((int)(servo_out[CH_RUDDER]));			// 	3	bytes 6, 7
	output_int((int)wp_distance);						// 	4	bytes 8, 9
	output_int((int)bearing_error);						// 	5	bytes 10,11
	output_int((int)roll_sensor);						// 	6	bytes 12,13
	output_int((int)loiter_total);						// 	7	bytes 14,15
	output_byte(wp_index);								// 	8	bytes 16
	output_byte(control_mode);							// 	9	bytes 17
	
	// print out the buffer and checksum
	// ---------------------------------
	print_buffer();
}

void output_int(int value)
{
	//buf_len += 2;
	out_buffer[buf_len++]	= value & 0xff;
	out_buffer[buf_len++]	= (value >> 8) & 0xff;
}

void output_byte(byte value)
{
	out_buffer[buf_len++]	= value;
}

void print_buffer()
{
	byte ck_a = 0;
	byte ck_b = 0;
	for (byte i = 0;i < buf_len; i++){
		Serial.print (out_buffer[i]);
	}
	Serial.print('\r');
	Serial.print('\n');
	buf_len = 0;
}


#endif

