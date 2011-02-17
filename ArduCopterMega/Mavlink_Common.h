#ifndef Mavlink_Common_H
#define Mavlink_Common_H

#if HIL_PROTOCOL == HIL_PROTOCOL_MAVLINK || GCS_PROTOCOL == GCS_PROTOCOL_MAVLINK

uint16_t system_type = MAV_FIXED_WING;
byte mavdelay = 0;

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
    AP_Var      *vp;
    float       value;

    // send parameters one by one and prevent cross port comms
    if (NULL != global_data.parameter_p && global_data.requested_interface == chan) {

        // if the value can't be represented as a float, we will skip it here
        vp = global_data.parameter_p;
        value = vp->cast_to_float();
        if (!isnan(value)) {
            char param_name[ONBOARD_PARAM_NAME_LENGTH];         /// XXX HACK

            vp->copy_name(param_name, sizeof(param_name));
            mavlink_msg_param_value_send(chan,
                                         (int8_t*)param_name,
                                         value,
                                         256,
                                         vp->meta_get_handle());
        }

        // remember the next variable we're going to send
        global_data.parameter_p = vp->next();
    }

    // this is called at 50hz, count runs to prevent flooding serialport and delayed to allow eeprom write
    mavdelay++;

    // request waypoints one by one
    if (global_data.waypoint_receiving && global_data.requested_interface == chan &&
            global_data.waypoint_request_i <= g.waypoint_total && mavdelay > 15) // limits to 3.33 hz
    {
        mavlink_msg_waypoint_request_send(chan,
                                          global_data.waypoint_dest_sysid,
                                          global_data.waypoint_dest_compid ,global_data.waypoint_request_i);
                                          mavdelay = 0;
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
        Matrix3f rot = dcm.get_dcm_matrix(); // neglecting angle of attack for now
        mavlink_msg_global_position_int_send(chan,current_loc.lat,
			current_loc.lng,current_loc.alt*10,gps.ground_speed/1.0e2*rot.a.x,
			gps.ground_speed/1.0e2*rot.b.x,gps.ground_speed/1.0e2*rot.c.x);
        break;
    }
    case MSG_LOCAL_LOCATION:
    {
        Matrix3f rot = dcm.get_dcm_matrix(); // neglecting angle of attack for now
        mavlink_msg_local_position_send(chan,timeStamp,ToRad((current_loc.lat-home.lat)/1.0e7)*radius_of_earth,
			ToRad((current_loc.lng-home.lng)/1.0e7)*radius_of_earth*cos(ToRad(home.lat/1.0e7)),
			(current_loc.alt-home.alt)/1.0e2, gps.ground_speed/1.0e2*rot.a.x,
			gps.ground_speed/1.0e2*rot.b.x,gps.ground_speed/1.0e2*rot.c.x);
        break;
    }
    case MSG_GPS_RAW:
    {
        mavlink_msg_gps_raw_send(chan,timeStamp,gps.status(),
			gps.latitude/1.0e7,gps.longitude/1.0e7,gps.altitude/100.0,
			gps.hdop,0.0,gps.ground_speed/100.0,gps.ground_course/100.0);
        break;
    }
    case MSG_SERVO_OUT:
    {
        uint8_t rssi = 1;
		// normalized values scaled to -10000 to 10000
		// This is used for HIL.  Do not change without discussing with HIL maintainers
        mavlink_msg_rc_channels_scaled_send(chan,
			10000*rc[0]->norm_output(),
			10000*rc[1]->norm_output(),
			10000*rc[2]->norm_output(),
			10000*rc[3]->norm_output(),
			0,0,0,0,rssi);
        break;
    }
    case MSG_RADIO_IN:
    {
        uint8_t rssi = 1;
        mavlink_msg_rc_channels_raw_send(chan,
			rc[0]->radio_in,
			rc[1]->radio_in,
			rc[2]->radio_in,
			rc[3]->radio_in,
			0/*rc[4]->radio_in*/,       // XXX currently only 4 RC channels defined
			0/*rc[5]->radio_in*/,
			0/*rc[6]->radio_in*/,
			0/*rc[7]->radio_in*/,
			rssi);
        break;
    }
    case MSG_RADIO_OUT:
    {
        mavlink_msg_servo_output_raw_send(chan,
			rc[0]->radio_out,
			rc[1]->radio_out,
			rc[2]->radio_out,
			rc[3]->radio_out,
			0/*rc[4]->radio_out*/,       // XXX currently only 4 RC channels defined
			0/*rc[5]->radio_out*/,
			0/*rc[6]->radio_out*/,
			0/*rc[7]->radio_out*/);
        break;
    }
    case MSG_VFR_HUD:
    {
        mavlink_msg_vfr_hud_send(chan, (float)airspeed/100.0, (float)gps.ground_speed/100.0, dcm.yaw_sensor, current_loc.alt/100.0,
			climb_rate, (int)rc[CH_THROTTLE]->servo_out);
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
            adc.Ch(AIRSPEED_CH),barometer.RawPress,0,0);
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
        mavlink_msg_waypoint_current_send(chan,g.waypoint_index);
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
