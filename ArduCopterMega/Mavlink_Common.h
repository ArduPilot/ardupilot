/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef Mavlink_Common_H
#define Mavlink_Common_H

#if HIL_PROTOCOL == HIL_PROTOCOL_MAVLINK || GCS_PROTOCOL == GCS_PROTOCOL_MAVLINK

byte mavdelay = 0;


static uint8_t mavlink_check_target(uint8_t sysid, uint8_t compid)
{
//Serial.print("target = "); Serial.print(sysid, DEC); Serial.print("\tcomp = "); Serial.println(compid, DEC);
    if (sysid != mavlink_system.sysid){
        return 1;

	// Currently we are not checking for correct compid since APM is not passing mavlink info to any subsystem
	// If it is addressed to our system ID we assume it is for us
    //}else if(compid != mavlink_system.compid){
	//	gcs.send_text_P(SEVERITY_LOW,PSTR("component id mismatch"));
    //    return 1; // XXX currently not receiving correct compid from gcs

    }else{
    	return 0; // no error
    }
}


void mavlink_send_message(mavlink_channel_t chan, uint8_t id, uint32_t param, uint16_t packet_drops)
{
    uint64_t timeStamp = micros();
	switch(id) {

		case MSG_HEARTBEAT:
			mavlink_msg_heartbeat_send(
					chan,
					mavlink_system.type,
					MAV_AUTOPILOT_ARDUPILOTMEGA);
			break;

		case MSG_EXTENDED_STATUS:
		{
			uint8_t mode 	 = MAV_MODE_UNINIT;
			uint8_t nav_mode = MAV_NAV_VECTOR;

			switch(control_mode) {
				case LOITER:
					mode 		= MAV_MODE_AUTO;
					nav_mode 	= MAV_NAV_HOLD;
					break;
				case AUTO:
					mode 		= MAV_MODE_AUTO;
					nav_mode 	= MAV_NAV_WAYPOINT;
					break;
				case RTL:
					mode 		= MAV_MODE_AUTO;
					nav_mode 	= MAV_NAV_RETURNING;
					break;
				case GUIDED:
					mode 		= MAV_MODE_GUIDED;
					break;
				default:
					mode 		= control_mode + 100;

			}

			uint8_t status 		= MAV_STATE_ACTIVE;
			uint16_t battery_remaining = 1000.0 * (float)(g.pack_capacity - current_total)/(float)g.pack_capacity;	//Mavlink scaling 100% = 1000

			mavlink_msg_sys_status_send(
					chan,
					mode,
					nav_mode,
					status,
					load * 1000,
					battery_voltage * 1000,
					battery_remaining,
					packet_drops);
			break;
		}

		case MSG_ATTITUDE:
		{
			//Vector3f omega = dcm.get_gyro();
			mavlink_msg_attitude_send(
					chan,
					timeStamp,
					dcm.roll,
					dcm.pitch,
					dcm.yaw,
					omega.x,
					omega.y,
					omega.z);
			break;
		}

		case MSG_LOCATION:
		{
			Matrix3f rot = dcm.get_dcm_matrix(); // neglecting angle of attack for now
			mavlink_msg_global_position_int_send(
					chan,
					current_loc.lat,
					current_loc.lng,
					/*current_loc.alt * 10,*/ // changed to absolute altitude
					g_gps->altitude,
					g_gps->ground_speed * rot.a.x,
					g_gps->ground_speed * rot.b.x,
					g_gps->ground_speed * rot.c.x);
			break;
		}

		case MSG_NAV_CONTROLLER_OUTPUT:
        {
            //if (control_mode != MANUAL) {
                mavlink_msg_nav_controller_output_send(
					chan,
                    nav_roll / 1.0e2,
                    nav_pitch / 1.0e2,
                    nav_bearing / 1.0e2,
                    target_bearing / 1.0e2,
                    wp_distance,
                    altitude_error / 1.0e2,
                    0,
                    crosstrack_error);
            //}
			break;
        }

		case MSG_LOCAL_LOCATION:
		{
			Matrix3f rot = dcm.get_dcm_matrix(); // neglecting angle of attack for now
			mavlink_msg_local_position_send(
					chan,
					timeStamp,
					ToRad((current_loc.lat - home.lat) / 1.0e7) * radius_of_earth,
					ToRad((current_loc.lng - home.lng) / 1.0e7) * radius_of_earth * cos(ToRad(home.lat / 1.0e7)),
					(current_loc.alt - home.alt) / 1.0e2,
					g_gps->ground_speed / 1.0e2 * rot.a.x,
					g_gps->ground_speed / 1.0e2 * rot.b.x,
					g_gps->ground_speed / 1.0e2 * rot.c.x);
			break;
		}

		case MSG_GPS_RAW:
		{
			mavlink_msg_gps_raw_send(
					chan,
					timeStamp,
					g_gps->status(),
					g_gps->latitude / 1.0e7,
					g_gps->longitude / 1.0e7,
					g_gps->altitude / 100.0,
					g_gps->hdop,
					0.0,
					g_gps->ground_speed / 100.0,
					g_gps->ground_course / 100.0);
			break;
		}

		case MSG_SERVO_OUT:
		{
			uint8_t rssi = 1;
			// normalized values scaled to -10000 to 10000
			// This is used for HIL.  Do not change without discussing with HIL maintainers
			mavlink_msg_rc_channels_scaled_send(
					chan,
					10000 * g.rc_1.norm_output(),
					10000 * g.rc_2.norm_output(),
					10000 * g.rc_3.norm_output(),
					10000 * g.rc_4.norm_output(),
					0,
					0,
					0,
					0,
					rssi);
			break;
		}

		case MSG_RADIO_IN:
		{
			uint8_t rssi = 1;
			mavlink_msg_rc_channels_raw_send(
					chan,
					 g.rc_1.radio_in,
					 g.rc_2.radio_in,
					 g.rc_3.radio_in,
					 g.rc_4.radio_in,
					 g.rc_5.radio_in,
					 g.rc_6.radio_in,
					 g.rc_7.radio_in,
					 g.rc_8.radio_in,
					 rssi);
			break;
		}

		case MSG_RADIO_OUT:
		{
			mavlink_msg_servo_output_raw_send(
					chan,
					motor_out[0],
					motor_out[1],
					motor_out[2],
					motor_out[3],
					motor_out[4],
					motor_out[5],
					motor_out[6],
					motor_out[7]);
			break;
		}

		case MSG_VFR_HUD:
		{
			mavlink_msg_vfr_hud_send(
					chan,
					(float)airspeed / 100.0,
					(float)g_gps->ground_speed / 100.0,
					(dcm.yaw_sensor / 100) % 360,
					g.rc_3.servo_out/10,
					/*current_loc.alt / 100.0,*/ // changed to absolute altitude
					g_gps->altitude/100.0,
					climb_rate);
			break;
		}

		#if HIL_MODE != HIL_MODE_ATTITUDE
		case MSG_RAW_IMU:
		{
			Vector3f accel = imu.get_accel();
			Vector3f gyro = imu.get_gyro();
			//Serial.printf_P(PSTR("sending accel: %f %f %f\n"), accel.x, accel.y, accel.z);
			//Serial.printf_P(PSTR("sending gyro: %f %f %f\n"), gyro.x, gyro.y, gyro.z);
			mavlink_msg_raw_imu_send(
					chan,
					timeStamp,
					accel.x * 1000.0 / gravity,
					accel.y * 1000.0 / gravity,
					accel.z * 1000.0 / gravity,
					gyro.x * 1000.0,
					gyro.y * 1000.0,
					gyro.z * 1000.0,
					compass.mag_x,
					compass.mag_y,
					compass.mag_z);

			/*  This message is not working.  Why?
			mavlink_msg_scaled_pressure_send(
					chan,
					timeStamp,
					(float)barometer.Press/100.,
					(float)adc.Ch(AIRSPEED_CH),		// We don't calculate the differential pressure value anywhere, so use raw
					(int)(barometer.Temp*100));
			*/
			break;
		}
		#endif // HIL_PROTOCOL != HIL_PROTOCOL_ATTITUDE

		case MSG_GPS_STATUS:
		{
			mavlink_msg_gps_status_send(
					chan,
					g_gps->num_sats,
					NULL,
					NULL,
					NULL,
					NULL,
					NULL);
			break;
		}

		case MSG_CURRENT_WAYPOINT:
		{
			mavlink_msg_waypoint_current_send(
					chan,
					g.waypoint_index);
			break;
		}

		default:
			break;
	}
}

void mavlink_send_text(mavlink_channel_t chan, uint8_t severity, const char *str)
{
	mavlink_msg_statustext_send(
				chan,
				severity,
				(const int8_t*) str);
}

void mavlink_acknowledge(mavlink_channel_t chan, uint8_t id, uint8_t sum1, uint8_t sum2)
{
}

#endif // mavlink in use

#endif // inclusion guard
