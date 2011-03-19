// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if GCS_PROTOCOL == GCS_PROTOCOL_MAVLINK || HIL_PROTOCOL == HIL_PROTOCOL_MAVLINK

#include "Mavlink_Common.h"

GCS_MAVLINK::GCS_MAVLINK() :
packet_drops(0)
{
}

void
GCS_MAVLINK::init(BetterStream * port)
{
    GCS_Class::init(port);
    if (port == &Serial) { // to split hil vs gcs
        mavlink_comm_0_port = port;
        chan = MAVLINK_COMM_0;
    }else{
        mavlink_comm_1_port = port;
        chan = MAVLINK_COMM_1;
    }
}

void
GCS_MAVLINK::update(void)
{
    // receive new packets
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






    // send out queued params/ waypoints
    _queued_send();

    // stop waypoint sending if timeout
    if (global_data.waypoint_sending && (millis() - global_data.waypoint_timelast_send) > global_data.waypoint_send_timeout){
        send_text_P(SEVERITY_LOW,PSTR("waypoint send timeout"));
        global_data.waypoint_sending = false;
    }

    // stop waypoint receiving if timeout
    if (global_data.waypoint_receiving && (millis() - global_data.waypoint_timelast_receive) > global_data.waypoint_receive_timeout){
        send_text_P(SEVERITY_LOW,PSTR("waypoint receive timeout"));
        global_data.waypoint_receiving = false;
    }
}

void
GCS_MAVLINK::data_stream_send(uint16_t freqMin, uint16_t freqMax)
{
    if (freqLoopMatch(global_data.streamRateRawSensors,freqMin,freqMax))
    	send_message(MSG_RAW_IMU);

    if (freqLoopMatch(global_data.streamRateExtendedStatus,freqMin,freqMax)) {
        send_message(MSG_EXTENDED_STATUS);
        send_message(MSG_GPS_STATUS);
        send_message(MSG_CURRENT_WAYPOINT);
        send_message(MSG_GPS_RAW);            // TODO - remove this message after location message is working
    }

    if (freqLoopMatch(global_data.streamRatePosition,freqMin,freqMax)) {
		send_message(MSG_LOCATION);
	}

    if (freqLoopMatch(global_data.streamRateRawController,freqMin,freqMax)) {
        // This is used for HIL.  Do not change without discussing with HIL maintainers
        send_message(MSG_SERVO_OUT);
    }

    if (freqLoopMatch(global_data.streamRateRCChannels,freqMin,freqMax)) {
        send_message(MSG_RADIO_OUT);
        send_message(MSG_RADIO_IN);
    }

    if (freqLoopMatch(global_data.streamRateExtra1,freqMin,freqMax)){   // Use Extra 1 for AHRS info
	    send_message(MSG_ATTITUDE);
    }

    if (freqLoopMatch(global_data.streamRateExtra2,freqMin,freqMax)){    // Use Extra 2 for additional HIL info
    	send_message(MSG_VFR_HUD);
    }

    if (freqLoopMatch(global_data.streamRateExtra3,freqMin,freqMax)){
        // Available datastream
    }
}

void
GCS_MAVLINK::send_message(uint8_t id, uint32_t param)
{
    mavlink_send_message(chan,id,param,packet_drops);
}

void
GCS_MAVLINK::send_text(uint8_t severity, const char *str)
{
    mavlink_send_text(chan,severity,str);
}

void
GCS_MAVLINK::send_text_P(uint8_t severity, const char *str)
{
	mavlink_statustext_t m;
	uint8_t i;
	for (i=0; i<sizeof(m.text); i++) {
		m.text[i] = pgm_read_byte(str++);
	}
	if (i < sizeof(m.text)) m.text[i] = 0;
    mavlink_send_text(chan, severity, (const char *)m.text);
}

void
GCS_MAVLINK::acknowledge(uint8_t id, uint8_t sum1, uint8_t sum2)
{
    mavlink_acknowledge(chan,id,sum1,sum2);
}

void GCS_MAVLINK::handleMessage(mavlink_message_t* msg)
{
	struct 	Location tell_command;				// command for telemetry
    switch (msg->msgid) {

    case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
        {
            // decode
            mavlink_request_data_stream_t packet;
            mavlink_msg_request_data_stream_decode(msg, &packet);
            if (mavlink_check_target(packet.target_system,packet.target_component)) break;

            int freq = 0; // packet frequency

            if (packet.start_stop == 0) freq = 0; // stop sending
            else if (packet.start_stop == 1) freq = packet.req_message_rate; // start sending
            else break;

			switch(packet.req_stream_id){

				case MAV_DATA_STREAM_ALL:
					global_data.streamRateRawSensors 		= freq;
					global_data.streamRateExtendedStatus 	= freq;
					global_data.streamRateRCChannels 		= freq;
					global_data.streamRateRawController 	= freq;
					global_data.streamRatePosition 			= freq;
					global_data.streamRateExtra1 			= freq;
					global_data.streamRateExtra2 			= freq;
					global_data.streamRateExtra3 			= freq;
					break;
				case MAV_DATA_STREAM_RAW_SENSORS:
					global_data.streamRateRawSensors = freq;
					break;
				case MAV_DATA_STREAM_EXTENDED_STATUS:
					global_data.streamRateExtendedStatus = freq;
					break;
				case MAV_DATA_STREAM_RC_CHANNELS:
					global_data.streamRateRCChannels = freq;
					break;
				case MAV_DATA_STREAM_RAW_CONTROLLER:
					global_data.streamRateRawController = freq;
					break;
					//case MAV_DATA_STREAM_RAW_SENSOR_FUSION:
					//    global_data.streamRateRawSensorFusion = freq;
					//    break;
				case MAV_DATA_STREAM_POSITION:
					global_data.streamRatePosition = freq;
					break;
				case MAV_DATA_STREAM_EXTRA1:
					global_data.streamRateExtra1 = freq;
					break;
				case MAV_DATA_STREAM_EXTRA2:
					global_data.streamRateExtra2 = freq;
					break;
				case MAV_DATA_STREAM_EXTRA3:
					global_data.streamRateExtra3 = freq;
					break;
				default:
					break;
			}
            break;
        }

    case MAVLINK_MSG_ID_ACTION:
        {
            // decode
            mavlink_action_t packet;
            mavlink_msg_action_decode(msg, &packet);
            if (mavlink_check_target(packet.target,packet.target_component)) break;

            // do action
            send_text_P(SEVERITY_LOW,PSTR("action received"));
			switch(packet.action){

				case MAV_ACTION_LAUNCH:
					//set_mode(TAKEOFF);
					break;

				case MAV_ACTION_RETURN:
					set_mode(RTL);
					break;

				case MAV_ACTION_EMCY_LAND:
					//set_mode(LAND);
					break;

				case MAV_ACTION_HALT:
					do_loiter_at_location();
					break;

					/* No mappable implementation in APM 2.0
				case MAV_ACTION_MOTORS_START:
				case MAV_ACTION_CONFIRM_KILL:
				case MAV_ACTION_EMCY_KILL:
				case MAV_ACTION_MOTORS_STOP:
				case MAV_ACTION_SHUTDOWN:
					break;
				*/

				case MAV_ACTION_CONTINUE:
					process_next_command();
					break;

				case MAV_ACTION_SET_MANUAL:
					set_mode(STABILIZE);
					break;

				case MAV_ACTION_SET_AUTO:
					set_mode(AUTO);
					break;

				case MAV_ACTION_STORAGE_READ:
					AP_Var::load_all();
					break;

				case MAV_ACTION_STORAGE_WRITE:
					AP_Var::save_all();
					break;

				case MAV_ACTION_CALIBRATE_RC: break;
					trim_radio();
					break;

				case MAV_ACTION_CALIBRATE_GYRO:
				case MAV_ACTION_CALIBRATE_MAG:
				case MAV_ACTION_CALIBRATE_ACC:
				case MAV_ACTION_CALIBRATE_PRESSURE:
				case MAV_ACTION_REBOOT:  // this is a rough interpretation
					//startup_IMU_ground();
					break;

				/*    For future implemtation
				case MAV_ACTION_REC_START: break;
				case MAV_ACTION_REC_PAUSE: break;
				case MAV_ACTION_REC_STOP: break;
				*/

				/* Takeoff is not an implemented flight mode in APM 2.0
				case MAV_ACTION_TAKEOFF:
					set_mode(TAKEOFF);
					break;
				*/

				case MAV_ACTION_NAVIGATE:
					set_mode(AUTO);
					break;

				/* Land is not an implemented flight mode in APM 2.0
				case MAV_ACTION_LAND:
					set_mode(LAND);
					break;
				*/

				case MAV_ACTION_LOITER:
					set_mode(LOITER);
					break;

				default: break;
				}
            break;
        }

    case MAVLINK_MSG_ID_WAYPOINT_REQUEST_LIST:
        {
            //send_text_P(SEVERITY_LOW,PSTR("waypoint request list"));

            // decode
            mavlink_waypoint_request_list_t packet;
            mavlink_msg_waypoint_request_list_decode(msg, &packet);

            if (mavlink_check_target(packet.target_system,packet.target_component))
            	break;

            // Start sending waypoints
            mavlink_msg_waypoint_count_send(
            	chan,msg->sysid,
	            msg->compid,
	            g.waypoint_total + 1); // + home

            global_data.waypoint_timelast_send 	= millis();
            global_data.waypoint_sending 		= true;
            global_data.waypoint_receiving 		= false;
            global_data.waypoint_dest_sysid 	= msg->sysid;
            global_data.waypoint_dest_compid 	= msg->compid;
            global_data.requested_interface 	= chan;
            break;
        }

    case MAVLINK_MSG_ID_WAYPOINT_REQUEST:
        {
            //send_text_P(SEVERITY_LOW,PSTR("waypoint request"));

            // Check if sending waypiont
            if (!global_data.waypoint_sending) break;

            // decode
            mavlink_waypoint_request_t packet;
            mavlink_msg_waypoint_request_decode(msg, &packet);
            if (mavlink_check_target(packet.target_system,packet.target_component)) break;

            // send waypoint
            tell_command = get_wp_with_index(packet.seq);

            // set frame of waypoint
            uint8_t frame = MAV_FRAME_GLOBAL; // reference frame
            float param1 = 0, param2 = 0 , param3 = 0, param4 = 0;

            // time that the mav should loiter in milliseconds
            uint8_t current = 0; // 1 (true), 0 (false)
            if (packet.seq == g.waypoint_index) current = 1;
            uint8_t autocontinue = 1; // 1 (true), 0 (false)
            float x = 0, y = 0, z = 0;

            if (tell_command.id < MAV_CMD_NAV_LAST) {
                // command needs scaling
                x = tell_command.lat/1.0e7; // local (x), global (latitude)
                y = tell_command.lng/1.0e7; // local (y), global (longitude)
                z = tell_command.alt/1.0e2; // local (z), global (altitude)
            }

			switch (tell_command.id) {					// Switch to map APM command fields inot MAVLink command fields
			case MAV_CMD_NAV_LOITER_TURNS:
			case MAV_CMD_NAV_TAKEOFF:
			case MAV_CMD_CONDITION_CHANGE_ALT:
			case MAV_CMD_DO_SET_HOME:
				param1 = tell_command.p1;
				break;

			case MAV_CMD_NAV_LOITER_TIME:
				param1 = tell_command.p1*10;	// APM loiter time is in ten second increments
				break;

			case MAV_CMD_CONDITION_DELAY:
			case MAV_CMD_CONDITION_DISTANCE:
				param1 = tell_command.lat;
				break;

			case MAV_CMD_DO_JUMP:
				param2 = tell_command.lat;
				param1 = tell_command.p1;
				break;

			case MAV_CMD_DO_REPEAT_SERVO:
				param4 = tell_command.lng;
			case MAV_CMD_DO_REPEAT_RELAY:
			case MAV_CMD_DO_CHANGE_SPEED:
				param3 = tell_command.lat;
				param2 = tell_command.alt;
				param1 = tell_command.p1;
				break;

			case MAV_CMD_DO_SET_PARAMETER:
			case MAV_CMD_DO_SET_RELAY:
			case MAV_CMD_DO_SET_SERVO:
				param2 = tell_command.alt;
				param1 = tell_command.p1;
				break;
			}

            mavlink_msg_waypoint_send(chan,msg->sysid,
            msg->compid,packet.seq,frame,tell_command.id,current,autocontinue,
            param1,param2,param3,param4,x,y,z);

            // update last waypoint comm stamp
            global_data.waypoint_timelast_send = millis();
            break;
        }

    case MAVLINK_MSG_ID_WAYPOINT_ACK:
        {
            //send_text_P(SEVERITY_LOW,PSTR("waypoint ack"));

            // decode
            mavlink_waypoint_ack_t packet;
            mavlink_msg_waypoint_ack_decode(msg, &packet);
            if (mavlink_check_target(packet.target_system,packet.target_component)) break;

            // check for error
            uint8_t type = packet.type; // ok (0), error(1)

            // turn off waypoint send
            global_data.waypoint_sending = false;
            break;
        }

    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
        {
            //send_text_P(SEVERITY_LOW,PSTR("param request list"));

            // decode
            mavlink_param_request_list_t packet;
            mavlink_msg_param_request_list_decode(msg, &packet);
            if (mavlink_check_target(packet.target_system,packet.target_component)) break;

            // Start sending parameters - next call to ::update will kick the first one out

            _queued_parameter = AP_Var::first();
            _queued_parameter_index = 0;
            break;
        }

    case MAVLINK_MSG_ID_WAYPOINT_CLEAR_ALL:
        {
            //send_text_P(SEVERITY_LOW,PSTR("waypoint clear all"));

            // decode
            mavlink_waypoint_clear_all_t packet;
            mavlink_msg_waypoint_clear_all_decode(msg, &packet);
            if (mavlink_check_target(packet.target_system,packet.target_component)) break;

            // clear all waypoints
            uint8_t type = 0; // ok (0), error(1)
            g.waypoint_total.set_and_save(0);

            // send acknowledgement 3 times to makes sure it is received
            for (int i=0;i<3;i++) mavlink_msg_waypoint_ack_send(chan,msg->sysid,msg->compid,type);

            break;
        }

    case MAVLINK_MSG_ID_WAYPOINT_SET_CURRENT:
        {
            //send_text_P(SEVERITY_LOW,PSTR("waypoint set current"));

            // decode
            mavlink_waypoint_set_current_t packet;
            mavlink_msg_waypoint_set_current_decode(msg, &packet);
            if (mavlink_check_target(packet.target_system,packet.target_component)) break;

            // set current waypoint
            g.waypoint_index.set_and_save(packet.seq);

            {
                Location temp;  // XXX this is gross
                temp = get_wp_with_index(packet.seq);
                set_next_WP(&temp);
            }

            mavlink_msg_waypoint_current_send(chan, g.waypoint_index);
            break;
        }

    case MAVLINK_MSG_ID_WAYPOINT_COUNT:
        {
            //send_text_P(SEVERITY_LOW,PSTR("waypoint count"));

            // decode
            mavlink_waypoint_count_t packet;
            mavlink_msg_waypoint_count_decode(msg, &packet);
            if (mavlink_check_target(packet.target_system,packet.target_component)) break;

            // start waypoint receiving
            if (packet.count > MAX_WAYPOINTS) {
                packet.count = MAX_WAYPOINTS;
            }
            g.waypoint_total.set_and_save(packet.count - 1);
            global_data.waypoint_timelast_receive = millis();
            global_data.waypoint_receiving 	= true;
            global_data.waypoint_sending 	= false;
            global_data.waypoint_request_i 	= 0;
            break;
        }

    case MAVLINK_MSG_ID_WAYPOINT:
        {
            // Check if receiving waypiont
            if (!global_data.waypoint_receiving) break;

            // decode
            mavlink_waypoint_t packet;
            mavlink_msg_waypoint_decode(msg, &packet);
            if (mavlink_check_target(packet.target_system,packet.target_component)) break;

            // check if this is the requested waypoint
            if (packet.seq != global_data.waypoint_request_i) break;

            // store waypoint
            uint8_t loadAction = 0; // 0 insert in list, 1 exec now

            // defaults
            tell_command.id = packet.command;

            switch (packet.frame)
            {
            case MAV_FRAME_GLOBAL:
            case MAV_FRAME_MISSION:
                {
                    tell_command.lat = 1.0e7*packet.x; // in as DD converted to * t7
                    tell_command.lng = 1.0e7*packet.y; // in as DD converted to * t7
                    tell_command.alt = packet.z*1.0e2; // in as m converted to cm
                    break;
                }

            case MAV_FRAME_LOCAL: // local (relative to home position)
                {
                    tell_command.lat = 1.0e7*ToDeg(packet.x/
                    (radius_of_earth*cos(ToRad(home.lat/1.0e7)))) + home.lat;
                    tell_command.lng = 1.0e7*ToDeg(packet.y/radius_of_earth) + home.lng;
                    tell_command.alt = -packet.z*1.0e2 + home.alt;
                    break;
                }
            }

			switch (tell_command.id) {					// Switch to map APM command fields inot MAVLink command fields
			case MAV_CMD_NAV_LOITER_TURNS:
			case MAV_CMD_NAV_TAKEOFF:
			case MAV_CMD_DO_SET_HOME:
				tell_command.p1 = packet.param1;
				break;

			case MAV_CMD_CONDITION_CHANGE_ALT:
				tell_command.p1 = packet.param1 * 100;
				break;

			case MAV_CMD_NAV_LOITER_TIME:
				tell_command.p1 = packet.param1 / 10;	// APM loiter time is in ten second increments
				break;

			case MAV_CMD_CONDITION_DELAY:
			case MAV_CMD_CONDITION_DISTANCE:
				tell_command.lat = packet.param1;
				break;

			case MAV_CMD_DO_JUMP:
				tell_command.lat = packet.param2;
				tell_command.p1 = packet.param1;
				break;

			case MAV_CMD_DO_REPEAT_SERVO:
				tell_command.lng = packet.param4;
			case MAV_CMD_DO_REPEAT_RELAY:
			case MAV_CMD_DO_CHANGE_SPEED:
				tell_command.lat = packet.param3;
				tell_command.alt = packet.param2;
				tell_command.p1 = packet.param1;
				break;

			case MAV_CMD_DO_SET_PARAMETER:
			case MAV_CMD_DO_SET_RELAY:
			case MAV_CMD_DO_SET_SERVO:
				tell_command.alt = packet.param2;
				tell_command.p1 = packet.param1;
				break;
			}



            // save waypoint - and prevent re-setting home
            if (packet.seq != 0)
            set_wp_with_index(tell_command, packet.seq);

            // update waypoint receiving state machine
            global_data.waypoint_timelast_receive = millis();
            global_data.waypoint_request_i++;

            if (global_data.waypoint_request_i > g.waypoint_total)
            {
                uint8_t type = 0; // ok (0), error(1)

                mavlink_msg_waypoint_ack_send(
                	chan,
                	msg->sysid,
                	msg->compid,
                	type);

                send_text_P(SEVERITY_LOW,PSTR("flight plan received"));
                global_data.waypoint_receiving = false;
                // XXX ignores waypoint radius for individual waypoints, can
                // only set WP_RADIUS parameter
            }
            break;
        }

    case MAVLINK_MSG_ID_PARAM_SET:
        {
            AP_Var                  *vp;
            AP_Meta_class::Type_id  var_type;

            // decode
            mavlink_param_set_t packet;
            mavlink_msg_param_set_decode(msg, &packet);
            if (mavlink_check_target(packet.target_system,packet.target_component))
            break;

            // set parameter

			char key[ONBOARD_PARAM_NAME_LENGTH+1];
			strncpy(key, (char *)packet.param_id, ONBOARD_PARAM_NAME_LENGTH);
			key[ONBOARD_PARAM_NAME_LENGTH] = 0;

            // find the requested parameter
            vp = AP_Var::find(key);
            if ((NULL != vp) &&                             // exists
                    !isnan(packet.param_value) &&               // not nan
                    !isinf(packet.param_value)) {               // not inf

				// add a small amount before casting parameter values
				// from float to integer to avoid truncating to the
				// next lower integer value.
				const float rounding_addition = 0.01;

                // fetch the variable type ID
                var_type = vp->meta_type_id();

                // handle variables with standard type IDs
                if (var_type == AP_Var::k_typeid_float) {
                    ((AP_Float *)vp)->set_and_save(packet.param_value);

                } else if (var_type == AP_Var::k_typeid_float16) {
                    ((AP_Float16 *)vp)->set_and_save(packet.param_value);

                } else if (var_type == AP_Var::k_typeid_int32) {
                    ((AP_Int32 *)vp)->set_and_save(packet.param_value+rounding_addition);

                } else if (var_type == AP_Var::k_typeid_int16) {
                    ((AP_Int16 *)vp)->set_and_save(packet.param_value+rounding_addition);

                } else if (var_type == AP_Var::k_typeid_int8) {
                    ((AP_Int8 *)vp)->set(packet.param_value+rounding_addition);
                } else {
                    // we don't support mavlink set on this parameter
                    break;
                }

                // Report back the new value if we accepted the change
                // we send the value we actually set, which could be
                // different from the value sent, in case someone sent
                // a fractional value to an integer type
                mavlink_msg_param_value_send(
                    chan,
                    (int8_t *)key,
                    vp->cast_to_float(),
                    _count_parameters(),
                    -1); // XXX we don't actually know what its index is...
            }

            break;
        } // end case

#if ALLOW_RC_OVERRIDE == ENABLED
	case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
	    {
			// allow override of RC channel values for HIL
			// or for complete GCS control of switch position
			// and RC PWM values.
			mavlink_rc_channels_raw_t packet;
			int16_t v[8];
			mavlink_msg_rc_channels_raw_decode(msg, &packet);
			v[0] = packet.chan1_raw;
			v[1] = packet.chan2_raw;
			v[2] = packet.chan3_raw;
			v[3] = packet.chan4_raw;
			v[4] = packet.chan5_raw;
			v[5] = packet.chan6_raw;
			v[6] = packet.chan7_raw;
			v[7] = packet.chan8_raw;
			APM_RC.setHIL(v);
			break;
		}
#endif

#if HIL_MODE != HIL_MODE_DISABLED
        // This is used both as a sensor and to pass the location
        // in HIL_ATTITUDE mode.
    case MAVLINK_MSG_ID_GPS_RAW:
        {
            // decode
            mavlink_gps_raw_t packet;
            mavlink_msg_gps_raw_decode(msg, &packet);

            // set gps hil sensor
            g_gps->setHIL(packet.usec/1000.0,packet.lat,packet.lon,packet.alt,
            packet.v,packet.hdg,0,0);
            break;
        }

        //    Is this resolved? - MAVLink protocol change.....
    case MAVLINK_MSG_ID_VFR_HUD:
        {
            // decode
            mavlink_vfr_hud_t packet;
            mavlink_msg_vfr_hud_decode(msg, &packet);

            // set airspeed
            airspeed = 100*packet.airspeed;
            break;
        }

#endif
#if HIL_MODE == HIL_MODE_ATTITUDE
    case MAVLINK_MSG_ID_ATTITUDE:
        {
            // decode
            mavlink_attitude_t packet;
            mavlink_msg_attitude_decode(msg, &packet);

            // set dcm hil sensor
            dcm.setHil(packet.roll,packet.pitch,packet.yaw,packet.rollspeed,
            packet.pitchspeed,packet.yawspeed);
            break;
        }
#endif
#if HIL_MODE == HIL_MODE_SENSORS

    case MAVLINK_MSG_ID_RAW_IMU:
        {
            // decode
            mavlink_raw_imu_t packet;
            mavlink_msg_raw_imu_decode(msg, &packet);

            // set imu hil sensors
            // TODO: check scaling for temp/absPress
            float temp = 70;
            float absPress = 1;
            //      Serial.printf_P(PSTR("accel:\t%d\t%d\t%d\n"), packet.xacc, packet.yacc, packet.zacc);
            //      Serial.printf_P(PSTR("gyro:\t%d\t%d\t%d\n"), packet.xgyro, packet.ygyro, packet.zgyro);

            // rad/sec
            Vector3f gyros;
            gyros.x = (float)packet.xgyro / 1000.0;
            gyros.y = (float)packet.ygyro / 1000.0;
            gyros.z = (float)packet.zgyro / 1000.0;
            // m/s/s
            Vector3f accels;
            accels.x = (float)packet.xacc / 1000.0;
            accels.y = (float)packet.yacc / 1000.0;
            accels.z = (float)packet.zacc / 1000.0;

            imu.set_gyro(gyros);

            imu.set_accel(accels);

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
            barometer.setHIL(temp,packet.press_diff1);
            break;
        }
#endif // HIL_MODE
    } // end switch
} // end handle mavlink

uint16_t
GCS_MAVLINK::_count_parameters()
{
    // if we haven't cached the parameter count yet...
    if (0 == _parameter_count) {
        AP_Var  *vp;

        vp = AP_Var::first();
        do {
            // if a parameter responds to cast_to_float then we are going to be able to report it
            if (!isnan(vp->cast_to_float())) {
                _parameter_count++;
            }
        } while (NULL != (vp = vp->next()));
    }
    return _parameter_count;
}

AP_Var *
GCS_MAVLINK::_find_parameter(uint16_t index)
{
    AP_Var  *vp;

    vp = AP_Var::first();
    while (NULL != vp) {

        // if the parameter is reportable
        if (!(isnan(vp->cast_to_float()))) {
            // if we have counted down to the index we want
            if (0 == index) {
                // return the parameter
                return vp;
            }
            // count off this parameter, as it is reportable but not
            // the one we want
            index--;
        }
        // and move to the next parameter
        vp = vp->next();
    }
    return NULL;
}

/**
* @brief Send low-priority messages at a maximum rate of xx Hertz
*
* This function sends messages at a lower rate to not exceed the wireless
* bandwidth. It sends one message each time it is called until the buffer is empty.
* Call this function with xx Hertz to increase/decrease the bandwidth.
*/
void
GCS_MAVLINK::_queued_send()
{

    // Check to see if we are sending parameters
    while (NULL != _queued_parameter) {
        AP_Var      *vp;
        float       value;

        // copy the current parameter and prepare to move to the next
        vp = _queued_parameter;
        _queued_parameter = _queued_parameter->next();

        // if the parameter can be cast to float, report it here and break out of the loop
        value = vp->cast_to_float();
        if (!isnan(value)) {

            char param_name[ONBOARD_PARAM_NAME_LENGTH];         /// XXX HACK
            vp->copy_name(param_name, sizeof(param_name));

            mavlink_msg_param_value_send(
            	chan,
	            (int8_t*)param_name,
    	        value,
        	    _count_parameters(),
            	_queued_parameter_index);

            _queued_parameter_index++;
            break;
        }




    }

    // this is called at 50hz, count runs to prevent flooding serialport and delayed to allow eeprom write
	mavdelay++;

    // request waypoints one by one
    // XXX note that this is pan-interface
	if (global_data.waypoint_receiving &&
		(global_data.requested_interface == chan) &&
		global_data.waypoint_request_i <= g.waypoint_total &&
		mavdelay > 15) { // limits to 3.33 hz
		mavlink_msg_waypoint_request_send(
			chan,
			global_data.waypoint_dest_sysid,
			global_data.waypoint_dest_compid,
			global_data.waypoint_request_i);

		mavdelay = 0;
	}
}

#endif

