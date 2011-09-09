// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#if HIL_MODE != HIL_MODE_DISABLED && HIL_PROTOCOL == HIL_PROTOCOL_MAVLINK

#include "Mavlink_Common.h"

HIL_MAVLINK::HIL_MAVLINK() :
    packet_drops(0)
{
}

void
HIL_MAVLINK::init(BetterStream * port)
{
    HIL_Class::init(port);
    mavlink_comm_0_port = port;
	chan = MAVLINK_COMM_0;
}

void
HIL_MAVLINK::update(void)
{
  	mavlink_message_t msg;
    mavlink_status_t status;

    // process received bytes
    while(comm_get_available(chan))
    {
        uint8_t c = comm_receive_ch(chan);

        // Try to get a new message
        if(mavlink_parse_char(chan, c, &msg, &status)) handleMessage(&msg);
    }

    // Update packet drops counter
    packet_drops += status.packet_rx_drop_count;
}

void
HIL_MAVLINK::send_message(uint8_t id, uint32_t param)
{
	mavlink_send_message(chan,id,param,packet_drops);
}

void
HIL_MAVLINK::send_text(uint8_t severity, const char *str)
{
	mavlink_send_text(chan,severity,str);
}

void
HIL_MAVLINK::acknowledge(uint8_t id, uint8_t sum1, uint8_t sum2)
{
	mavlink_acknowledge(chan,id,sum1,sum2);
}

void
HIL_MAVLINK::handleMessage(mavlink_message_t* msg)
{
    switch (msg->msgid)	{

	// handle incoming vehicle position
	case MAVLINK_MSG_ID_GLOBAL_POSITION:
	{
    	// decode
        mavlink_global_position_t packet;
        mavlink_msg_global_position_decode(msg, &packet);
        //if (mavlink_check_target(packet.target_system,packet.target_component)) break;
		trackVehicle_loc.id = 0;
		trackVehicle_loc.p1 = 0;
		trackVehicle_loc.alt = packet.alt;
		trackVehicle_loc.lat = packet.lat;
		trackVehicle_loc.lng = packet.lon;
		Serial.println("received global position packet");
	}

	// This is used both as a sensor and to pass the location
	// in HIL_ATTITUDE mode.
 	case MAVLINK_MSG_ID_GPS_RAW:
    {
		// decode
        mavlink_gps_raw_t packet;
        mavlink_msg_gps_raw_decode(msg, &packet);

        // set gps hil sensor
        gps.setHIL(packet.usec/1000.0,packet.lat,packet.lon,packet.alt,
                packet.v,packet.hdg,0,0);
        break;
    }

	case MAVLINK_MSG_ID_AIRSPEED:
    {
		// decode
        mavlink_airspeed_t packet;
        mavlink_msg_airspeed_decode(msg, &packet);

        // set airspeed
        airspeed = 100*packet.airspeed;
        break;
    }

#if HIL_MODE == HIL_MODE_ATTITUDE

 	case MAVLINK_MSG_ID_ATTITUDE:
    {
		// decode
        mavlink_attitude_t packet;
        mavlink_msg_attitude_decode(msg, &packet);

        // set gps hil sensor
        dcm.setHil(packet.roll,packet.pitch,packet.yaw,packet.rollspeed,
				packet.pitchspeed,packet.yawspeed);
        break;
    }

#elif HIL_MODE == HIL_MODE_SENSORS

       case MAVLINK_MSG_ID_RAW_IMU:
    {
        // decode
        mavlink_raw_imu_t packet;
        mavlink_msg_raw_imu_decode(msg, &packet);

        // set imu hil sensors
        // TODO: check scaling for temp/absPress
        float temp = 70;
        float absPress = 1;
		//Serial.printf_P(PSTR("\nreceived accel:\t%d\t%d\t%d\n"), packet.xacc, packet.yacc, packet.zacc);
		//Serial.printf_P(PSTR("\nreceived gyro:\t%d\t%d\t%d\n"), packet.xgyro, packet.ygyro, packet.zgyro);

        adc.setHIL(packet.xgyro,packet.ygyro,packet.zgyro,temp,
                packet.xacc,packet.yacc,packet.zacc,absPress);
        compass.setHIL(packet.xmag,packet.ymag,packet.zmag);
        break;
    }

    case MAVLINK_MSG_ID_RAW_PRESSURE:
    {
        // decode
        mavlink_raw_pressure_t packet;
        mavlink_msg_raw_pressure_decode(msg, &packet);

        // set pressure hil sensor
        // TODO: check scaling
        float temp = 70;
        pitot.setHIL(temp,packet.press_diff1);
        break;
    }
#endif // HIL_MODE

    } // end switch
}
#endif

