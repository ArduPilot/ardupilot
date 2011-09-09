// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

///
/// @file	GCS_Ardupilot.pde
/// @brief	GCS driver for the legacy Ardupilot GCS protocol.
///

#if GCS_PROTOCOL == GCS_PROTOCOL_LEGACY

/*
Message Prefixes
!!!		Position    		Low rate telemetry
+++		Attitude    		High rate telemetry
###		Mode        		Change in control mode
%%%		Waypoint    		Current and previous waypoints
XXX		Alert       		Text alert  - NOTE: Alert message generation is not localized to a function
PPP		IMU Performance		Sent every 20 seconds for performance monitoring

Message Suffix
***    All messages use this suffix
*/

void 
GCS_LEGACY::send_text(uint8_t severity, const char *str)
{
	if(severity >= DEBUG_LEVEL){
		_port->print("MSG: ");
		_port->println(str);
	}
}

void 
GCS_LEGACY::send_message(uint8_t id, uint32_t param)
{
	switch(id) {
		case MSG_ATTITUDE:								// ** Attitude message
			print_attitude();
			break;
		case MSG_LOCATION:								// ** Location/GPS message
			print_position();
			break;
		case MSG_MODE_CHANGE:
		case MSG_HEARTBEAT:								// ** Location/GPS message
			print_control_mode();
			break;
		case MSG_CPU_LOAD:
			//TODO: replace with appropriate message
			_port->printf_P(PSTR("MSG: load %ld%%\n"), param);
			break;
		//case MSG_PERF_REPORT:
		//	printPerfData();
	}	
}

void 
GCS_LEGACY::print_attitude(void)
{
	_port->print("+++");
	_port->print("ASP:");
	_port->print((int)airspeed / 100, DEC);
	_port->print(",THH:");
	_port->print(servo_out[CH_THROTTLE], DEC);
	_port->print (",RLL:");
	_port->print(roll_sensor / 100, DEC);
	_port->print (",PCH:");
	_port->print(pitch_sensor / 100, DEC);
	_port->println(",***");
}

void 
GCS_LEGACY::print_control_mode(void)
{
	switch (control_mode){
		case MANUAL:
			_port->println("###MANUAL***");
			break;
		case STABILIZE:
			_port->println("###STABILIZE***");
			break;
		case CIRCLE:
			_port->println("###CIRCLE***");
			break;
		case FLY_BY_WIRE_A:
			_port->println("###FBW A***");
			break;
		case FLY_BY_WIRE_B:
			_port->println("###FBW B***");
			break;
		case AUTO:
			_port->println("###AUTO***");
			break;
		case RTL:
			_port->println("###RTL***");
			break;
		case LOITER:
			_port->println("###LOITER***");
			break;
		case TAKEOFF:
			_port->println("##TAKEOFF***");
			break;
		case LAND:
			_port->println("##LAND***");
			break;
	}
}

void 
GCS_LEGACY::print_position(void)
{
	_port->print("!!!");
	_port->print("LAT:");
	_port->print(current_loc.lat/10,DEC);
	_port->print(",LON:");
	_port->print(current_loc.lng/10,DEC); //wp_current_lat
	_port->print(",SPD:");
	_port->print(gps.ground_speed/100,DEC);		
	_port->print(",CRT:");
	_port->print(climb_rate,DEC);
	_port->print(",ALT:");
	_port->print(current_loc.alt/100,DEC);
	_port->print(",ALH:");
	_port->print(next_WP.alt/100,DEC);
	_port->print(",CRS:");
	_port->print(yaw_sensor/100,DEC);
	_port->print(",BER:");
	_port->print(target_bearing/100,DEC);
	_port->print(",WPN:");
	_port->print(get(PARAM_WP_INDEX),DEC);//Actually is the waypoint.
	_port->print(",DST:");
	_port->print(wp_distance,DEC);
	_port->print(",BTV:");
	_port->print(battery_voltage,DEC);
	_port->print(",RSP:");
	_port->print(servo_out[CH_ROLL]/100,DEC);
	_port->print(",TOW:");
	_port->print(gps.time);
	_port->println(",***");
}

void 
GCS_LEGACY::print_waypoint(struct Location *cmd,byte index)
{
	_port->print("command #: ");
	_port->print(index, DEC);
	_port->print(" id: ");
	_port->print(cmd->id,DEC);
	_port->print(" p1: ");
	_port->print(cmd->p1,DEC);
	_port->print(" p2: ");
	_port->print(cmd->alt,DEC);
	_port->print(" p3: ");
	_port->print(cmd->lat,DEC);
	_port->print(" p4: ");
	_port->println(cmd->lng,DEC);
}

#endif // GCS_PROTOCOL_LEGACY
