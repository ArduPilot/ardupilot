// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if HIL_MODE != HIL_MODE_DISABLED && HIL_PROTOCOL == HIL_PROTOCOL_XPLANE


void HIL_XPLANE::output_HIL(void)
{
	// output real-time sensor data
	Serial.print("AAA"); 		 						// 		Message preamble
	output_int((int)(g.rc_1.servo_out));				// 	0	bytes 0, 1
	output_int((int)(g.rc_2.servo_out));				// 	1	bytes 2, 3
	output_int((int)(g.rc_3.servo_out));				// 	2	bytes 4, 5
	output_int((int)(g.rc_4.servo_out));				// 	3	bytes 6, 7
	output_int((int)wp_distance);						// 	4	bytes 8,9
	output_int((int)bearing_error);						// 	5	bytes 10,11
	output_int((int)altitude_error);					// 	6	bytes 12, 13
	output_int((int)energy_error);						// 	7	bytes 14,15
	output_byte((int)g.waypoint_index);					// 	8	bytes 16
	output_byte(control_mode);							// 	9	bytes 17

	// print out the buffer and checksum
	// ---------------------------------
	print_buffer();
}

void HIL_XPLANE::output_int(int value)
{
	//buf_len += 2;
	out_buffer[buf_len++]	= value & 0xff;
	out_buffer[buf_len++]	= (value >> 8) & 0xff;
}

void HIL_XPLANE::output_byte(byte value)
{
	out_buffer[buf_len++]	= value;
}

void HIL_XPLANE::print_buffer()
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



HIL_XPLANE::HIL_XPLANE() :
	buf_len(0)
{
}

void
HIL_XPLANE::init(BetterStream * port)
{
    HIL_Class::init(port);
	hilPacketDecoder = new AP_GPS_IMU(port);
	hilPacketDecoder->init();
}

void
HIL_XPLANE::update(void)
{
	hilPacketDecoder->update();
	airspeed = (float)hilPacketDecoder->airspeed; 	//airspeed is * 100 coming in from Xplane for accuracy
	calc_airspeed_errors();
	dcm.setHil(hilPacketDecoder->roll_sensor*M_PI/18000,
		hilPacketDecoder->pitch_sensor*M_PI/18000,
		hilPacketDecoder->ground_course*M_PI/18000,
		0,0,0);

        // set gps hil sensor
        g_gps->setHIL(hilPacketDecoder->time/1000.0,(float)hilPacketDecoder->latitude/1.0e7,(float)hilPacketDecoder->longitude/1.0e7,(float)hilPacketDecoder->altitude/1.0e2,
                (float)hilPacketDecoder->speed_3d/1.0e2,(float)hilPacketDecoder->ground_course/1.0e2,0,0);
}

void
HIL_XPLANE::send_message(uint8_t id, uint32_t param)
{
	// TODO: split output by actual request
    uint64_t timeStamp = micros();
    switch(id) {

    case MSG_HEARTBEAT:
		break;
    case MSG_EXTENDED_STATUS:
		break;
    case MSG_ATTITUDE:
        break;
    case MSG_LOCATION:
        break;
    case MSG_LOCAL_LOCATION:
        break;
    case MSG_GPS_RAW:
        break;
    case MSG_SERVO_OUT:
		output_HIL();
        break;
    case MSG_RADIO_OUT:
        break;
    case MSG_RAW_IMU:
        break;
    case MSG_GPS_STATUS:
        break;
    case MSG_CURRENT_WAYPOINT:
        break;
    defualt:
        break;
	}
}

void
HIL_XPLANE::send_text(uint8_t severity, const char *str)
{
}

void
HIL_XPLANE::send_text(uint8_t severity, const prog_char_t *str)
{
}

void
HIL_XPLANE::acknowledge(uint8_t id, uint8_t sum1, uint8_t sum2)
{
}

#endif
