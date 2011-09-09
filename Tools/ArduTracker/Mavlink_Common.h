#ifndef Mavlink_Common_H
#define Mavlink_Common_H

#if HIL_PROTOCOL == HIL_PROTOCOL_MAVLIK || GCS_PROTOCOL == GCS_PROTOCOL_MAVLINK

uint16_t system_type = MAV_FIXED_WING;

static uint8_t mavlink_check_target(uint8_t sysid, uint8_t compid)
{
    if (sysid != mavlink_system.sysid)
    {
        return 1;
    }
    else if (compid != mavlink_system.compid)
    {
		gcs.send_text(SEVERITY_LOW,"component id mismatch");
        return 0; // XXX currently not receiving correct compid from gcs
    }
    else return 0; // no error
}

/**
 * @brief Send low-priority messages at a maximum rate of xx Hertz
 *
 * This function sends messages at a lower rate to not exceed the wireless
 * bandwidth. It sends one message each time it is called until the buffer is empty.
 * Call this function with xx Hertz to increase/decrease the bandwidth.
 */
static void mavlink_queued_send(mavlink_channel_t chan)
{
    //send parameters one by one
    if (global_data.parameter_i < global_data.param_count)
    {
        char param_name[ONBOARD_PARAM_NAME_LENGTH];			/// XXX HACK
        strcpy_P(param_name, getParamName(global_data.parameter_i));	/// XXX HACK
        mavlink_msg_param_value_send(chan,
                                     (int8_t*)param_name,
                                     getParamAsFloat(global_data.parameter_i),
                                     global_data.param_count,global_data.parameter_i);
        global_data.parameter_i++;
    }

    // request waypoints one by one
    if (global_data.waypoint_receiving &&
            global_data.waypoint_request_i < get(PARAM_WP_TOTAL))
    {
        mavlink_msg_waypoint_request_send(chan,
                                          global_data.waypoint_dest_sysid,
                                          global_data.waypoint_dest_compid ,global_data.waypoint_request_i);
    }
}

void mavlink_send_message(mavlink_channel_t chan, uint8_t id, uint32_t param, uint16_t packet_drops)
{
    uint64_t timeStamp = micros();
    switch(id) {

    case MSG_HEARTBEAT:
		mavlink_msg_heartbeat_send(chan,system_type,MAV_AUTOPILOT_ARDUPILOTMEGA);
		break;

    case MSG_EXTENDED_STATUS:
    {
        uint8_t mode = MAV_MODE_UNINIT;
        uint8_t nav_mode = MAV_NAV_VECTOR;

		switch(control_mode) {
		case MANUAL:
			mode = MAV_MODE_MANUAL;
			break;
		case CIRCLE:
			mode = MAV_MODE_TEST3;
			break;
		case STABILIZE:
			mode = MAV_MODE_GUIDED;
			break;
		case FLY_BY_WIRE_A:
			mode = MAV_MODE_TEST1;
			break;
		case FLY_BY_WIRE_B:
			mode = MAV_MODE_TEST2;
			break;
		case AUTO: 
			mode = MAV_MODE_AUTO;
			nav_mode = MAV_NAV_WAYPOINT;
			break;
		case RTL: 
			mode = MAV_MODE_AUTO;
			nav_mode = MAV_NAV_RETURNING;
			break;
		case LOITER: 
			mode = MAV_MODE_AUTO;
			nav_mode = MAV_NAV_HOLD;
			break;
		case TAKEOFF:
			mode = MAV_MODE_AUTO;
			nav_mode = MAV_NAV_LIFTOFF;
			break;
		case LAND:
			mode = MAV_MODE_AUTO;
			nav_mode = MAV_NAV_LANDING;
			break;
		}
		uint8_t status = MAV_STATE_ACTIVE;
        uint8_t motor_block = false;

        mavlink_msg_sys_status_send(chan,mode,nav_mode,status,load*1000,
            battery_voltage1*1000,motor_block,packet_drops);
		break;
    }

    case MSG_ATTITUDE:
    {
		Vector3f omega = dcm.get_gyro();
        mavlink_msg_attitude_send(chan,timeStamp,dcm.roll,dcm.pitch,dcm.yaw,
		omega.x,omega.y,omega.z);
        break;
    }
    case MSG_LOCATION:
    {
        float gamma = dcm.pitch; // neglecting angle of attack for now
		float yaw = dcm.yaw;
        mavlink_msg_global_position_send(chan,timeStamp,current_loc.lat/1.0e7,
			current_loc.lng/1.0e7,current_loc.alt/1.0e2,gps.ground_speed/1.0e2*cos(gamma)*cos(yaw),
			gps.ground_speed/1.0e2*cos(gamma)*sin(yaw),gps.ground_speed/1.0e2*sin(gamma));
        break;
    }
    case MSG_LOCAL_LOCATION:
    {
        float gamma = dcm.pitch; // neglecting angle of attack for now
		float yaw = dcm.yaw;
        mavlink_msg_local_position_send(chan,timeStamp,ToRad((current_loc.lat-home.lat)/1.0e7)*radius_of_earth,
			ToRad((current_loc.lng-home.lng)/1.0e7)*radius_of_earth*cos(ToRad(home.lat/1.0e7)),
			(current_loc.alt-home.alt)/1.0e2, gps.ground_speed/1.0e2*cos(gamma)*cos(yaw),
			 gps.ground_speed/1.0e2*cos(gamma)*sin(yaw),gps.ground_speed/1.0e2*sin(gamma));
        break;
    }
    case MSG_GPS_RAW:
    {
        mavlink_msg_gps_raw_send(chan,timeStamp,gps.status(),
			gps.latitude/1.0e7,gps.longitude/1.0e7,gps.altitude/100.0,
			2.0,10.0,gps.ground_speed/100.0,gps.ground_course/100.0);
        break;
    }
    case MSG_AIRSPEED:
    {
        mavlink_msg_airspeed_send(chan,float(airspeed)/100.0);
        break;
    }
    case MSG_SERVO_OUT:
    {
        uint8_t rssi = 1; // TODO: can we calculated this?
        // receive signal strength 0(0%)-255(100%)
		Serial.printf_P(PSTR("sending servo out: %d %d %d %d\n"), 
				servo_out[0],servo_out[1], servo_out[2], servo_out[3]);
        mavlink_msg_rc_channels_scaled_send(chan,
            servo_out[0],servo_out[1],
			servo_out[2]*100, // account for throttle scaling 0-100
            servo_out[3],servo_out[4],servo_out[5],
            servo_out[6],servo_out[7],rssi);
        break;
    }
    case MSG_RADIO_OUT:
    {
        uint8_t rssi = 1; // TODO: can we calculated this?
        // receive signal strength 0(0%)-255(100%)
        mavlink_msg_rc_channels_raw_send(chan,
            radio_out[0],radio_out[1],radio_out[2],
            radio_out[3],radio_out[4],radio_out[5],
            radio_out[6],radio_out[7],rssi);
        break;
    }

#if HIL_MODE != HIL_MODE_ATTITUDE
    case MSG_RAW_IMU:
    {
		Vector3f accel = imu.get_accel();
		Vector3f gyro = imu.get_gyro();
		//Serial.printf_P(PSTR("sending accel: %f %f %f\n"), accel.x, accel.y, accel.z);
		//Serial.printf_P(PSTR("sending gyro: %f %f %f\n"), gyro.x, gyro.y, gyro.z);
        mavlink_msg_raw_imu_send(chan,timeStamp,
            accel.x*1000.0/gravity,accel.y*1000.0/gravity,accel.z*1000.0/gravity,
			gyro.x*1000.0,gyro.y*1000.0,gyro.z*1000.0,
            compass.mag_x,compass.mag_y,compass.mag_z);
        mavlink_msg_raw_pressure_send(chan,timeStamp,
            adc.Ch(AIRSPEED_CH),pitot.RawPress,0);
        break;
    }
#endif // HIL_PROTOCOL != HIL_PROTOCOL_ATTITUDE

    case MSG_GPS_STATUS:
    {
        mavlink_msg_gps_status_send(chan,gps.num_sats,NULL,NULL,NULL,NULL,NULL);
        break;
    }

    case MSG_CURRENT_WAYPOINT:
    {
        mavlink_msg_waypoint_current_send(chan,get(PARAM_WP_INDEX));
        break;
    }

    defualt:
        break;
    }
}

void mavlink_send_text(mavlink_channel_t chan, uint8_t severity, const char *str)
{
	mavlink_msg_statustext_send(chan,severity,(const int8_t*)str);
}

void mavlink_acknowledge(mavlink_channel_t chan, uint8_t id, uint8_t sum1, uint8_t sum2)
{
}

#endif // mavlink in use

#endif // inclusion guard
