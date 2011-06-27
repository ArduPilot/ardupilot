// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if GCS_PROTOCOL == GCS_PROTOCOL_MAVLINK || HIL_PROTOCOL == HIL_PROTOCOL_MAVLINK

#include "Mavlink_Common.h"

GCS_MAVLINK::GCS_MAVLINK(AP_Var::Key key) :
packet_drops(0),

// parameters
// note, all values not explicitly initialised here are zeroed
waypoint_send_timeout(1000), // 1 second
waypoint_receive_timeout(1000), // 1 second

// stream rates
_group	(key, key == Parameters::k_param_streamrates_port0 ? PSTR("SR0_"): PSTR("SR3_")),
				// AP_VAR					//ref	 //index, default, 	name
				streamRateRawSensors		(&_group, 0,		 0, 	PSTR("RAW_SENS")),
				streamRateExtendedStatus	(&_group, 1,		 0, 	PSTR("EXT_STAT")),
				streamRateRCChannels		(&_group, 2,		 0, 	PSTR("RC_CHAN")),
				streamRateRawController		(&_group, 3,		 0, 	PSTR("RAW_CTRL")),
				streamRatePosition			(&_group, 4,		 0, 	PSTR("POSITION")),
				streamRateExtra1			(&_group, 5,		 0, 	PSTR("EXTRA1")),
				streamRateExtra2			(&_group, 6,		 0, 	PSTR("EXTRA2")),
				streamRateExtra3			(&_group, 7,		 0, 	PSTR("EXTRA3"))
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
	_queued_parameter = NULL;

	// temporary
	streamRateRawSensors.set_and_save(0);
	streamRateExtendedStatus.set_and_save(0);
	streamRateRCChannels.set_and_save(0);
	streamRateRawController.set_and_save(0);
	streamRatePosition.set_and_save(0);
	streamRateExtra1.set_and_save(0);
	streamRateExtra2.set_and_save(0);
	streamRateExtra3.set_and_save(0);
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
	if (waypoint_sending && (millis() - waypoint_timelast_send) > waypoint_send_timeout){
		send_text_P(SEVERITY_LOW,PSTR("waypoint send timeout"));
		waypoint_sending = false;
	}

	// stop waypoint receiving if timeout
	if (waypoint_receiving && (millis() - waypoint_timelast_receive) > waypoint_receive_timeout){
		send_text_P(SEVERITY_LOW,PSTR("waypoint receive timeout"));
		waypoint_receiving = false;
	}
}

/*
mav2 1
mav3 3
mav5 3
mav6 10
mav7 10
mav6 10
mav7 10
mav6 10
mav7 10
*/

void
GCS_MAVLINK::data_stream_send(uint16_t freqMin, uint16_t freqMax)
{
	if (waypoint_sending == false && waypoint_receiving == false && _queued_parameter == NULL) {

		if (freqLoopMatch(streamRateRawSensors, freqMin, freqMax)){
			send_message(MSG_RAW_IMU);
			//Serial.printf("mav1 %d\n", (int)streamRateRawSensors.get());
		}

		if (freqLoopMatch(streamRateExtendedStatus, freqMin, freqMax)) {
			send_message(MSG_EXTENDED_STATUS);
			send_message(MSG_GPS_STATUS);
			send_message(MSG_CURRENT_WAYPOINT);
			send_message(MSG_GPS_RAW);			// TODO - remove this message after location message is working
			send_message(MSG_NAV_CONTROLLER_OUTPUT);
			//Serial.printf("mav2 %d\n", (int)streamRateExtendedStatus.get());
		}

		if (freqLoopMatch(streamRatePosition, freqMin, freqMax)) {
			// sent with GPS read
			//send_message(MSG_LOCATION);
			//Serial.printf("mav3 %d\n", (int)streamRatePosition.get());
		}

		if (freqLoopMatch(streamRateRawController, freqMin, freqMax)) {
			// This is used for HIL.  Do not change without discussing with HIL maintainers
			send_message(MSG_SERVO_OUT);
			//Serial.printf("mav4 %d\n", (int)streamRateRawController.get());
		}

		if (freqLoopMatch(streamRateRCChannels, freqMin, freqMax)) {
			send_message(MSG_RADIO_OUT);
			send_message(MSG_RADIO_IN);
			//Serial.printf("mav5 %d\n", (int)streamRateRCChannels.get());
		}

		if (freqLoopMatch(streamRateExtra1, freqMin, freqMax)){	 // Use Extra 1 for AHRS info
			send_message(MSG_ATTITUDE);
			//Serial.printf("mav6 %d\n", (int)streamRateExtra1.get());
		}

		if (freqLoopMatch(streamRateExtra2, freqMin, freqMax)){		// Use Extra 2 for additional HIL info
			send_message(MSG_VFR_HUD);
			//Serial.printf("mav7 %d\n", (int)streamRateExtra2.get());
		}

		if (freqLoopMatch(streamRateExtra3, freqMin, freqMax)){
		// Available datastream
			//Serial.printf("mav8 %d\n", (int)streamRateExtra3.get());
		}
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
GCS_MAVLINK::send_text(uint8_t severity, const prog_char_t *str)
{
	mavlink_statustext_t m;
	uint8_t i;
	for (i=0; i<sizeof(m.text); i++) {
		m.text[i] = pgm_read_byte((const prog_char *)(str++));
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

			if (mavlink_check_target(packet.target_system,packet.target_component))
				break;

			int freq = 0; // packet frequency

			if (packet.start_stop == 0)
				freq = 0; // stop sending
			else if (packet.start_stop == 1)
				freq = packet.req_message_rate; // start sending
			else
				break;

			switch(packet.req_stream_id){

				case MAV_DATA_STREAM_ALL:
					streamRateRawSensors		= freq;
					streamRateExtendedStatus	= freq;
					streamRateRCChannels		= freq;
					streamRateRawController		= freq;
					streamRatePosition			= freq;
					streamRateExtra1			= freq;
					streamRateExtra2			= freq;
					//streamRateExtra3.set_and_save(freq);	// We just do set and save on the last as it takes care of the whole group.
					streamRateExtra3			= freq;	// Don't save!!

					break;

				case MAV_DATA_STREAM_RAW_SENSORS:
					streamRateRawSensors = freq;		// We do not set and save this one so that if HIL is shut down incorrectly
														// we will not continue to broadcast raw sensor data at 50Hz.
					break;
				case MAV_DATA_STREAM_EXTENDED_STATUS:
					//streamRateExtendedStatus.set_and_save(freq);
					streamRateExtendedStatus = freq;
					break;

				case MAV_DATA_STREAM_RC_CHANNELS:
					streamRateRCChannels  = freq;
					break;

				case MAV_DATA_STREAM_RAW_CONTROLLER:
					streamRateRawController = freq;
					break;
					//case MAV_DATA_STREAM_RAW_SENSOR_FUSION:
					//	streamRateRawSensorFusion.set_and_save(freq);
					//	break;
				case MAV_DATA_STREAM_POSITION:
					streamRatePosition = freq;
					break;

				case MAV_DATA_STREAM_EXTRA1:
					streamRateExtra1 = freq;
					break;

				case MAV_DATA_STREAM_EXTRA2:
					streamRateExtra2 = freq;
					//streamRateExtra2.set_and_save(freq);
					break;

				case MAV_DATA_STREAM_EXTRA3:
					streamRateExtra3 = freq;
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

			uint8_t result = 0;

			// do action
			send_text_P(SEVERITY_LOW,PSTR("action received: "));
//Serial.println(packet.action);
			switch(packet.action){

				case MAV_ACTION_LAUNCH:
					//set_mode(TAKEOFF);
					break;

				case MAV_ACTION_RETURN:
					set_mode(RTL);
					result=1;
					break;

				case MAV_ACTION_EMCY_LAND:
					//set_mode(LAND);
					break;

				case MAV_ACTION_HALT:
					do_loiter_at_location();
					result=1;
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
					result=1;
					break;

				case MAV_ACTION_SET_MANUAL:
					set_mode(STABILIZE);
					result=1;
					break;

				case MAV_ACTION_SET_AUTO:
					set_mode(AUTO);
					result=1;
					break;

				case MAV_ACTION_STORAGE_READ:
					AP_Var::load_all();
					result=1;
					break;

				case MAV_ACTION_STORAGE_WRITE:
					AP_Var::save_all();
					result=1;
					break;

				case MAV_ACTION_CALIBRATE_RC: break;
					trim_radio();
					result=1;
					break;

				case MAV_ACTION_CALIBRATE_GYRO:
				case MAV_ACTION_CALIBRATE_MAG:
				case MAV_ACTION_CALIBRATE_ACC:
				case MAV_ACTION_CALIBRATE_PRESSURE:
				case MAV_ACTION_REBOOT:  // this is a rough interpretation
					//startup_IMU_ground();
					//result=1;
					break;

				/*	For future implemtation
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
					result=1;
					break;

				/* Land is not an implemented flight mode in APM 2.0
				case MAV_ACTION_LAND:
					set_mode(LAND);
					break;
				*/

				case MAV_ACTION_LOITER:
					set_mode(LOITER);
					result=1;
					break;

				default: break;
				}

				mavlink_msg_action_ack_send(
					chan,
					packet.action,
					result
					);

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

			waypoint_timelast_send		= millis();
			waypoint_sending			= true;
			waypoint_receiving			= false;
			waypoint_dest_sysid			= msg->sysid;
			waypoint_dest_compid		= msg->compid;
			requested_interface			= chan;
			break;
		}

	// XXX read a WP from EEPROM and send it to the GCS
	case MAVLINK_MSG_ID_WAYPOINT_REQUEST:
		{
			//send_text_P(SEVERITY_LOW,PSTR("waypoint request"));

			// Check if sending waypiont
			if (!waypoint_sending)
				break;

			// decode
			mavlink_waypoint_request_t packet;
			mavlink_msg_waypoint_request_decode(msg, &packet);

 			if (mavlink_check_target(packet.target_system,packet.target_component))
 				break;

			// send waypoint
			tell_command = get_command_with_index(packet.seq);

			// set frame of waypoint
			uint8_t frame;

			if (tell_command.options & WP_OPTION_ALT_RELATIVE) {
				frame = MAV_FRAME_GLOBAL_RELATIVE_ALT; // reference frame
			} else {
				frame = MAV_FRAME_GLOBAL; // reference frame
			}

			float param1 = 0, param2 = 0 , param3 = 0, param4 = 0;

			// time that the mav should loiter in milliseconds
			uint8_t current = 0; // 1 (true), 0 (false)

			if (packet.seq == g.waypoint_index)
				current = 1;

			uint8_t autocontinue = 1; // 1 (true), 0 (false)

			float x = 0, y = 0, z = 0;

			if (tell_command.id < MAV_CMD_NAV_LAST) {
				// command needs scaling
				x = tell_command.lat/1.0e7; // local (x), global (latitude)
				y = tell_command.lng/1.0e7; // local (y), global (longitude)
				// ACM is processing alt inside each command. so we save and load raw values. - this is diffrent to APM
				z = tell_command.alt/1.0e2; // local (z), global/relative (altitude)
			}


			switch (tell_command.id) {					// Switch to map APM command fields inot MAVLink command fields

				case MAV_CMD_NAV_LOITER_TURNS:
				case MAV_CMD_CONDITION_CHANGE_ALT:
				case MAV_CMD_DO_SET_HOME:
					param1 = tell_command.p1;
					break;

				case MAV_CMD_NAV_TAKEOFF:
					param1 = 0;
					break;

				case MAV_CMD_NAV_LOITER_TIME:
					param1 = tell_command.p1;	// APM loiter time is in ten second increments
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

				case MAV_CMD_NAV_WAYPOINT:
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
										msg->compid,
										packet.seq,
										frame,
										tell_command.id,
										current,
										autocontinue,
										param1,
										param2,
										param3,
										param4,
										x,
										y,
										z);

			// update last waypoint comm stamp
			waypoint_timelast_send = millis();
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
			waypoint_sending = false;
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
			_queued_parameter_count = _count_parameters();
			requested_interface = chan;
			break;
		}

	case MAVLINK_MSG_ID_WAYPOINT_CLEAR_ALL:
		{
			//send_text_P(SEVERITY_LOW,PSTR("waypoint clear all"));

			// decode
			mavlink_waypoint_clear_all_t packet;
			mavlink_msg_waypoint_clear_all_decode(msg, &packet);
			if (mavlink_check_target(packet.target_system, packet.target_component)) break;

			// clear all waypoints
			uint8_t type = 0; // ok (0), error(1)
			g.waypoint_total.set_and_save(0);

			// send acknowledgement 3 times to makes sure it is received
			for (int i=0;i<3;i++)
				mavlink_msg_waypoint_ack_send(chan, msg->sysid, msg->compid, type);

			break;
		}

	case MAVLINK_MSG_ID_WAYPOINT_SET_CURRENT:
		{
			//send_text_P(SEVERITY_LOW,PSTR("waypoint set current"));

			// decode
			mavlink_waypoint_set_current_t packet;
			mavlink_msg_waypoint_set_current_decode(msg, &packet);
			if (mavlink_check_target(packet.target_system,packet.target_component)) break;

			// set current command
			change_command(packet.seq);

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

			waypoint_timelast_receive = millis();
			waypoint_receiving   = true;
			waypoint_sending	 = false;
			waypoint_request_i   = 0;
			break;
		}

	// XXX receive a WP from GCS and store in EEPROM
	case MAVLINK_MSG_ID_WAYPOINT:
		{
			// Check if receiving waypiont
			if (!waypoint_receiving) break;

			// decode
			mavlink_waypoint_t packet;
			mavlink_msg_waypoint_decode(msg, &packet);
			if (mavlink_check_target(packet.target_system,packet.target_component)) break;

			// check if this is the requested waypoint
			if (packet.seq != waypoint_request_i) break;

			// store waypoint
			uint8_t loadAction = 0; // 0 insert in list, 1 exec now

			// defaults
			tell_command.id = packet.command;

			/*
			switch (packet.frame){

				case MAV_FRAME_MISSION:
				case MAV_FRAME_GLOBAL:
					{
						tell_command.lat = 1.0e7*packet.x; // in as DD converted to * t7
						tell_command.lng = 1.0e7*packet.y; // in as DD converted to * t7
						tell_command.alt = packet.z*1.0e2; // in as m converted to cm
						tell_command.options = 0;
						break;
					}

				case MAV_FRAME_LOCAL: // local (relative to home position)
					{
						tell_command.lat = 1.0e7*ToDeg(packet.x/
						(radius_of_earth*cos(ToRad(home.lat/1.0e7)))) + home.lat;
						tell_command.lng = 1.0e7*ToDeg(packet.y/radius_of_earth) + home.lng;
						tell_command.alt = packet.z*1.0e2;
						tell_command.options = 1;
						break;
					}
				//case MAV_FRAME_GLOBAL_RELATIVE_ALT: // absolute lat/lng, relative altitude
				default:
					{
						tell_command.lat = 1.0e7 * packet.x; // in as DD converted to * t7
						tell_command.lng = 1.0e7 * packet.y; // in as DD converted to * t7
						tell_command.alt = packet.z * 1.0e2;
						tell_command.options = 1; // store altitude relative!! Always!!
						break;
					}
			}
			*/

			// we only are supporting Abs position, relative Alt
			tell_command.lat = 1.0e7 * packet.x; // in as DD converted to * t7
			tell_command.lng = 1.0e7 * packet.y; // in as DD converted to * t7
			tell_command.alt = packet.z * 1.0e2;
			tell_command.options = 1; // store altitude relative!! Always!!

			switch (tell_command.id) {					// Switch to map APM command fields inot MAVLink command fields
				case MAV_CMD_NAV_LOITER_TURNS:
				case MAV_CMD_DO_SET_HOME:
				case MAV_CMD_DO_SET_ROI:
					tell_command.p1 = packet.param1;
					break;

				case MAV_CMD_NAV_TAKEOFF:
					tell_command.p1 = 0;
					break;

				case MAV_CMD_CONDITION_CHANGE_ALT:
					tell_command.p1 = packet.param1 * 100;
					break;

				case MAV_CMD_NAV_LOITER_TIME:
					tell_command.p1 = packet.param1;	// APM loiter time is in ten second increments
					break;

				case MAV_CMD_CONDITION_DELAY:
				case MAV_CMD_CONDITION_DISTANCE:
					tell_command.lat = packet.param1;
					break;

				case MAV_CMD_DO_JUMP:
					tell_command.lat = packet.param2;
					tell_command.p1  = packet.param1;
					break;

				case MAV_CMD_DO_REPEAT_SERVO:
					tell_command.lng = packet.param4;
				case MAV_CMD_DO_REPEAT_RELAY:
				case MAV_CMD_DO_CHANGE_SPEED:
					tell_command.lat = packet.param3;
					tell_command.alt = packet.param2;
					tell_command.p1 = packet.param1;
					break;

				case MAV_CMD_NAV_WAYPOINT:
					tell_command.p1 = packet.param1;
					break;

				case MAV_CMD_DO_SET_PARAMETER:
				case MAV_CMD_DO_SET_RELAY:
				case MAV_CMD_DO_SET_SERVO:
					tell_command.alt = packet.param2;
					tell_command.p1 = packet.param1;
					break;
			}

			set_command_with_index(tell_command, packet.seq);

			// update waypoint receiving state machine
			waypoint_timelast_receive = millis();
			waypoint_request_i++;

			if (waypoint_request_i > g.waypoint_total){
				uint8_t type = 0; // ok (0), error(1)

				mavlink_msg_waypoint_ack_send(
					chan,
					msg->sysid,
					msg->compid,
					type);

				send_text_P(SEVERITY_LOW,PSTR("flight plan received"));
				waypoint_receiving = false;
				// XXX ignores waypoint radius for individual waypoints, can
				// only set WP_RADIUS parameter
			}
			break;
		}

	case MAVLINK_MSG_ID_PARAM_SET:
		{
			AP_Var				  *vp;
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
			if ((NULL != vp) &&							 		// exists
					!isnan(packet.param_value) &&			   // not nan
					!isinf(packet.param_value)) {			   // not inf

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
					((AP_Int32 *)vp)->set_and_save(packet.param_value + rounding_addition);

				} else if (var_type == AP_Var::k_typeid_int16) {
					((AP_Int16 *)vp)->set_and_save(packet.param_value + rounding_addition);

				} else if (var_type == AP_Var::k_typeid_int8) {
					((AP_Int8 *)vp)->set_and_save(packet.param_value + rounding_addition);
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
	if (NULL != _queued_parameter && (requested_interface == chan) && mavdelay > 1) {
		AP_Var	  *vp;
		float	   value;

		// copy the current parameter and prepare to move to the next
		vp = _queued_parameter;
		_queued_parameter = _queued_parameter->next();

		// if the parameter can be cast to float, report it here and break out of the loop
		value = vp->cast_to_float();
		if (!isnan(value)) {

			char param_name[ONBOARD_PARAM_NAME_LENGTH];		 /// XXX HACK
			vp->copy_name(param_name, sizeof(param_name));

			mavlink_msg_param_value_send(
				chan,
				(int8_t*)param_name,
				value,
				_queued_parameter_count,
				_queued_parameter_index);

			_queued_parameter_index++;
		}
		mavdelay = 0;
	}

	// this is called at 50hz, count runs to prevent flooding serialport and delayed to allow eeprom write
	mavdelay++;

	// request waypoints one by one
	// XXX note that this is pan-interface
	if (waypoint_receiving &&
		(requested_interface == chan) &&
		waypoint_request_i <= g.waypoint_total &&
		mavdelay > 15) { // limits to 3.33 hz

		mavlink_msg_waypoint_request_send(
			chan,
			waypoint_dest_sysid,
			waypoint_dest_compid,
			waypoint_request_i);

		mavdelay = 0;
	}
}

#endif

