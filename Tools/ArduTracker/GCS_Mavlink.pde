// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#if GCS_PROTOCOL == GCS_PROTOCOL_MAVLINK

#include "Mavlink_Common.h"

GCS_MAVLINK::GCS_MAVLINK() :
    packet_drops(0)
{
}

void
GCS_MAVLINK::init(BetterStream * port)
{
    GCS_Class::init(port);
    mavlink_comm_1_port = port;
	chan = MAVLINK_COMM_1;
}

void
GCS_MAVLINK::update(void)
{
	// recieve new packets
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
	mavlink_queued_send(chan);

	// send parameters communication_queued_send(chan); 
    // stop waypoint sending if timeout
    if (global_data.waypoint_sending &&
            millis() - global_data.waypoint_timelast_send >
            global_data.waypoint_send_timeout)
    {
		send_text(SEVERITY_LOW,"waypoint send timeout");
        global_data.waypoint_sending = false;
    }

    // stop waypoint receiving if timeout
    if (global_data.waypoint_receiving &&
            millis() - global_data.waypoint_timelast_receive >
            global_data.waypoint_receive_timeout)
    {
		send_text(SEVERITY_LOW,"waypoint receive timeout");
        global_data.waypoint_receiving = false;
    }
}

void 
GCS_MAVLINK::data_stream_send(uint16_t freqMin, uint16_t freqMax)
{
    if (freqLoopMatch(global_data.streamRateRawSensors,freqMin,freqMax))
        send_message(MSG_RAW_IMU);

    if (freqLoopMatch(global_data.streamRateExtendedStatus,freqMin,freqMax))
        send_message(MSG_EXTENDED_STATUS);

    if (freqLoopMatch(global_data.streamRateRCChannels,freqMin,freqMax))
        send_message(MSG_RADIO_OUT);

    if (freqLoopMatch(global_data.streamRateRawController,freqMin,freqMax))
        send_message(MSG_SERVO_OUT);

    //if (freqLoopMatch(global_data.streamRateRawSensorFusion,freqMin,freqMax))

    if (freqLoopMatch(global_data.streamRatePosition,freqMin,freqMax))
        send_message(MSG_LOCATION);

    if (freqLoopMatch(global_data.streamRateExtra1,freqMin,freqMax))
        send_message(MSG_GPS_STATUS);

    if (freqLoopMatch(global_data.streamRateExtra2,freqMin,freqMax))
        send_message(MSG_CURRENT_WAYPOINT);

    if (freqLoopMatch(global_data.streamRateExtra3,freqMin,freqMax))
	{
		send_message(MSG_LOCAL_LOCATION);
		send_message(MSG_ATTITUDE);
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
GCS_MAVLINK::acknowledge(uint8_t id, uint8_t sum1, uint8_t sum2)
{
	mavlink_acknowledge(chan,id,sum1,sum2);
}

void GCS_MAVLINK::handleMessage(mavlink_message_t* msg)
{
	switch (msg->msgid)	{

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

        switch(packet.req_stream_id)
        {
        case MAV_DATA_STREAM_ALL:
            global_data.streamRateRawSensors = freq; 
            global_data.streamRateExtendedStatus = freq; 
            global_data.streamRateRCChannels = freq; 
            global_data.streamRateRawController = freq; 
            global_data.streamRateRawSensorFusion = freq; 
            global_data.streamRatePosition = freq; 
            global_data.streamRateExtra1 = freq; 
            global_data.streamRateExtra2 = freq; 
            global_data.streamRateExtra3 = freq; 
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
        case MAV_DATA_STREAM_RAW_SENSOR_FUSION:
            global_data.streamRateRawSensorFusion = freq; 
            break;
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
    }

    case MAVLINK_MSG_ID_ACTION:
    {
        // decode
        mavlink_action_t packet;
        mavlink_msg_action_decode(msg, &packet);
        if (mavlink_check_target(packet.target,packet.target_component)) break;

        // do action
		gcs.send_text(SEVERITY_LOW,"action received");
        switch(packet.action)
        {

            case MAV_ACTION_LAUNCH:
                set_mode(TAKEOFF);
                break;

            case MAV_ACTION_RETURN:
                set_mode(RTL);
                break;

            case MAV_ACTION_EMCY_LAND:
                set_mode(LAND);
                break;

            case MAV_ACTION_HALT: 
                loiter_at_location();
                break;

            // can't implement due to APM configuration
            // just setting to manual to be safe
            case MAV_ACTION_MOTORS_START:
            case MAV_ACTION_CONFIRM_KILL:
            case MAV_ACTION_EMCY_KILL:
            case MAV_ACTION_MOTORS_STOP:
            case MAV_ACTION_SHUTDOWN: 
                set_mode(MANUAL);
                break;

            case MAV_ACTION_CONTINUE:
                process_next_command();
                break;

            case MAV_ACTION_SET_MANUAL: 
                set_mode(MANUAL);
                break;

            case MAV_ACTION_SET_AUTO:
                set_mode(AUTO);
                break; 

            case MAV_ACTION_STORAGE_READ:
                //read_EEPROM_startup();
                //read_EEPROM_airstart_critical();
                //read_command_index();
                //read_EEPROM_flight_modes();
                break; 

            case MAV_ACTION_STORAGE_WRITE: 
                //save_EEPROM_trims();
                //save_EEPROM_waypoint_info();
                //save_EEPROM_gains();
                //save_command_index();
                //save_pressure_data();
                //save_EEPROM_radio_minmax();
                //save_user_configs();
                //save_EEPROM_flight_modes();
                break;

            case MAV_ACTION_CALIBRATE_RC: break; 
                trim_radio();
                break;
            
            case MAV_ACTION_CALIBRATE_GYRO:
            case MAV_ACTION_CALIBRATE_MAG: 
            case MAV_ACTION_CALIBRATE_ACC: 
            case MAV_ACTION_CALIBRATE_PRESSURE:
            case MAV_ACTION_REBOOT:  // this is a rough interpretation
                startup_IMU_ground();     
                break; 

            case MAV_ACTION_REC_START: break; 
            case MAV_ACTION_REC_PAUSE: break; 
            case MAV_ACTION_REC_STOP: break; 

            case MAV_ACTION_TAKEOFF:
                set_mode(TAKEOFF);
                break; 

            case MAV_ACTION_NAVIGATE:
                set_mode(AUTO);
                break; 

            case MAV_ACTION_LAND:
                set_mode(LAND);
                break; 

            case MAV_ACTION_LOITER:
                set_mode(LOITER);
                break; 

            default: break;
        }
    }
    break;

    case MAVLINK_MSG_ID_WAYPOINT_REQUEST_LIST:
    {
		//send_text(SEVERITY_LOW,"waypoint request list");

        // decode
        mavlink_waypoint_request_list_t packet;
        mavlink_msg_waypoint_request_list_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component)) break;

        // Start sending waypoints
        mavlink_msg_waypoint_count_send(chan,msg->sysid,
                                        msg->compid,get(PARAM_WP_TOTAL));
        global_data.waypoint_timelast_send = millis();
        global_data.waypoint_sending = true;
        global_data.waypoint_receiving = false;
        global_data.waypoint_dest_sysid = msg->sysid;
        global_data.waypoint_dest_compid = msg->compid;

    }
    break;

    case MAVLINK_MSG_ID_WAYPOINT_REQUEST:
    {
		//send_text(SEVERITY_LOW,"waypoint request");

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
        uint8_t action = MAV_ACTION_NAVIGATE; // action
        uint8_t orbit_direction = 0; // clockwise(0), counter-clockwise(1)
        float orbit = 0; // loiter radius
        float param1 = 0, param2 = 0;

        switch(tell_command.id)
        {

        case CMD_WAYPOINT: // navigate
            action = MAV_ACTION_NAVIGATE; // action
            break;

        case CMD_LOITER_TIME: // loiter
            orbit = get(PARAM_WP_RADIUS); // XXX setting loiter radius as waypoint acceptance radius
            action = MAV_ACTION_LOITER; // action
            param1 = get(PARAM_WP_RADIUS);
            param2 = tell_command.p1*100; // loiter time
            break;

        case CMD_TAKEOFF: // takeoff
            action = MAV_ACTION_TAKEOFF;
            break;

        case CMD_LAND: // land
            action = MAV_ACTION_LAND;
            break; 

        defaut:
			gcs.send_text(SEVERITY_LOW,"command not handled");
            break;
        }

        // time that the mav should loiter in milliseconds
        uint8_t current = 0; // 1 (true), 0 (false)
        if (packet.seq == get(PARAM_WP_INDEX)) current = 1;
        float yaw_dir = 0; // yaw orientation in radians, 0 = north XXX: what does this do?
        uint8_t autocontinue = 1; // 1 (true), 0 (false)
        float x = tell_command.lng/1.0e7; // local (x), global (longitude)
        float y = tell_command.lat/1.0e7; // local (y), global (latitude)
        float z = tell_command.alt/1.0e2; // local (z), global (altitude)
        // note XXX: documented x,y,z order does not match with gps raw
        mavlink_msg_waypoint_send(chan,msg->sysid,
                                  msg->compid,packet.seq,frame,action,
                                  orbit,orbit_direction,param1,param2,current,x,y,z,yaw_dir,autocontinue);

        // update last waypoint comm stamp
        global_data.waypoint_timelast_send = millis();
    }
    break;

    case MAVLINK_MSG_ID_WAYPOINT_ACK:
    {
		//send_text(SEVERITY_LOW,"waypoint ack");

        // decode
        mavlink_waypoint_ack_t packet;
        mavlink_msg_waypoint_ack_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component)) break;

        // check for error
        uint8_t type = packet.type; // ok (0), error(1)

        // turn off waypoint send
        global_data.waypoint_sending = false;
    }
    break;

    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
    {
		//send_text(SEVERITY_LOW,"param request list");

        // decode
        mavlink_param_request_list_t packet;
        mavlink_msg_param_request_list_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component)) break;

        // Start sending parameters
        global_data.parameter_i = 0;
    }
    break;

    case MAVLINK_MSG_ID_WAYPOINT_CLEAR_ALL:
    {
		//send_text(SEVERITY_LOW,"waypoint clear all");

        // decode
        mavlink_waypoint_clear_all_t packet;
        mavlink_msg_waypoint_clear_all_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component)) break;

        // clear all waypoints
        uint8_t type = 0; // ok (0), error(1)
        set(PARAM_WP_TOTAL,0);

        // send acknowledgement 3 times to makes sure it is received
        for (int i=0;i<3;i++) mavlink_msg_waypoint_ack_send(chan,msg->sysid,msg->compid,type);

        break;
    }

    case MAVLINK_MSG_ID_WAYPOINT_SET_CURRENT:
    {
		//send_text(SEVERITY_LOW,"waypoint set current");

        // decode
        mavlink_waypoint_set_current_t packet;
        mavlink_msg_waypoint_set_current_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component)) break;

        // set current waypoint
        set(PARAM_WP_INDEX,packet.seq);
		{
			Location temp;	// XXX this is gross
			temp = get_wp_with_index(packet.seq);
			set_next_WP(&temp);
		}
        mavlink_msg_waypoint_current_send(chan,get(PARAM_WP_INDEX));
        break;
    }

    case MAVLINK_MSG_ID_WAYPOINT_COUNT:
    {
		//send_text(SEVERITY_LOW,"waypoint count");

        // decode
        mavlink_waypoint_count_t packet;
        mavlink_msg_waypoint_count_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component)) break;

        // start waypoint receiving
        set(PARAM_WP_TOTAL,packet.count);
        if (get(PARAM_WP_TOTAL) > MAX_WAYPOINTS)
            set(PARAM_WP_TOTAL,MAX_WAYPOINTS);
        global_data.waypoint_timelast_receive = millis();
        global_data.waypoint_receiving = true;
        global_data.waypoint_sending = false;
        global_data.waypoint_request_i = 0;
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

        switch (packet.frame)
        {
            case MAV_FRAME_GLOBAL:
            {
                tell_command.lng = 1.0e7*packet.x;
                tell_command.lat = 1.0e7*packet.y;
                tell_command.alt = packet.z*1.0e2;
                break;
            }

            case MAV_FRAME_LOCAL: // local (relative to home position)
            {
                tell_command.lng = 1.0e7*ToDeg(packet.x/
                        (radius_of_earth*cos(ToRad(home.lat/1.0e7)))) + home.lng;
                tell_command.lat = 1.0e7*ToDeg(packet.y/radius_of_earth) + home.lat;
                tell_command.alt = -packet.z*1.0e2 + home.alt;
                break;
            }
        }

        // defaults
        tell_command.id = CMD_BLANK;

        switch (packet.action)
        {
        
            case MAV_ACTION_TAKEOFF:
            {
                tell_command.id = CMD_TAKEOFF;
                break;
            } 
            case MAV_ACTION_LAND:
            {
                tell_command.id = CMD_LAND;
                break;
            }

            case MAV_ACTION_NAVIGATE:
            {
                tell_command.id = CMD_WAYPOINT;
                break;
            }

            case MAV_ACTION_LOITER:
            {
                tell_command.id = CMD_LOITER_TIME;
                tell_command.p1 = packet.param2/1.0e2;
                break;
            }
        }

        // save waypoint
        set_wp_with_index(tell_command, packet.seq);

        // update waypoint receiving state machine
        global_data.waypoint_timelast_receive = millis();
        global_data.waypoint_request_i++;

        if (global_data.waypoint_request_i == get(PARAM_WP_TOTAL))
        {
			gcs.send_text(SEVERITY_LOW,"flight plane received");
            uint8_t type = 0; // ok (0), error(1)
            mavlink_msg_waypoint_ack_send(chan,msg->sysid,msg->compid,type);
            global_data.waypoint_receiving = false;
            set(PARAM_WP_RADIUS,packet.param1); // XXX takes last waypoint radius for all
            //save_EEPROM_waypoint_info();
        }
        break;
    }

    case MAVLINK_MSG_ID_PARAM_SET:
    {
        // decode
        mavlink_param_set_t packet;
        mavlink_msg_param_set_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component))
            break;

        // set parameter
        const char * key = (const char*) packet.param_id;

        // iterate known parameters
        // XXX linear search; would be better to sort params & use a binary search
        for (uint16_t i = 0; i < global_data.param_count; i++) {

            // compare key with parameter name
            if (!strcmp_P(key, getParamName(i))) {

                // sanity-check the new parameter
                if (!isnan(packet.param_value) &&						// not nan
					!isinf(packet.param_value)) {						// not inf

                    setParamAsFloat(i,packet.param_value);

                    // Report back new value
                    char param_name[ONBOARD_PARAM_NAME_LENGTH];			/// XXX HACK
                    strcpy_P(param_name, getParamName(i));	/// XXX HACK
                    mavlink_msg_param_value_send(chan,
                                                 (int8_t*)param_name,
                                                 getParamAsFloat(i),
												 global_data.param_count,i);
					// call load if required
					if (i >= PARAM_RLL2SRV_P && i <= PARAM_RLL2SRV_IMAX) pidServoRoll.load_gains();
					if (i >= PARAM_PTCH2SRV_P && i <= PARAM_PTCH2SRV_IMAX) pidServoPitch.load_gains();
					if (i >= PARAM_YW2SRV_P && i <= PARAM_YW2SRV_IMAX) pidServoRudder.load_gains();
					if (i >= PARAM_HDNG2RLL_P && i <= PARAM_HDNG2RLL_IMAX) pidNavRoll.load_gains();
					if (i >= PARAM_ARSPD2PTCH_P && i <= PARAM_ARSPD2PTCH_IMAX) pidNavPitchAirspeed.load_gains();
					if (i >= PARAM_ALT2PTCH_P && i <= PARAM_ALT2PTCH_IMAX) pidNavPitchAltitude.load_gains();
					if (i >= PARAM_ENRGY2THR_P && i <= PARAM_ENRGY2THR_IMAX) pidTeThrottle.load_gains();
					if (i >= PARAM_ALT2THR_P && i <= PARAM_ALT2THR_IMAX) pidAltitudeThrottle.load_gains();
                }
                break;
            }
        }
        break;
    } // end case
  	} // end switch
} // end handle mavlink


#endif

