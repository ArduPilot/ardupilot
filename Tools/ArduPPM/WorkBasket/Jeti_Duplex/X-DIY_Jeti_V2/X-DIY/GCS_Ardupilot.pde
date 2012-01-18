
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#if GCS_PROTOCOL == GCS_PROTOCOL_LEGACY

#if GCS_PORT == 3
# define SendSer		Serial3.print
# define SendSerln		Serial3.println
#else
# define SendSer		Serial.print
# define SendSerln		Serial.println
#endif

//	This is the standard GCS output file for ArduPilot

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

/*
void acknowledge(byte id, byte check1, byte check2) {}
void send_message(byte id) {}
void send_message(byte id, long param) {}
void send_message(byte severity, const char *str) {}
*/

void acknowledge(byte id, byte check1, byte check2)
{
}

void send_message(byte severity, const char *str)		// This is the instance of send_message for message 0x05
{
	if(severity >= DEBUG_LEVEL){
		SendSer("MSG: ");
		SendSerln(str);
	}
}

void send_message(byte id)
{
	send_message(id,0l);
}
	
void send_message(byte id, long param)
{
	switch(id) {
		case MSG_ATTITUDE:								// ** Attitude message
			print_attitude();
			break;
		case MSG_LOCATION:								// ** Location/GPS message
			print_position();
			break;
		case MSG_HEARTBEAT:								// ** Location/GPS message
			print_control_mode();
			break;
		//case MSG_PERF_REPORT:
		//	printPerfData();
	}	
}

void print_current_waypoints()
{
}

void print_attitude(void)
{
	SendSer("+++");
	SendSer("ASP:");
	SendSer((int)airspeed / 100, DEC);
	SendSer(",THH:");
	SendSer(servo_out[CH_THROTTLE], DEC);
	SendSer (",RLL:");
	SendSer(roll_sensor / 100, DEC);
	SendSer (",PCH:");
	SendSer(pitch_sensor / 100, DEC);
	SendSerln(",***");
}

void print_control_mode(void)
{
	switch (control_mode){
		case MANUAL:
			SendSerln("###MANUAL***");
			break;
		case STABILIZE:
			SendSerln("###STABILIZE***");
			break;
		case CIRCLE:
			SendSerln("###CIRCLE***");
			break;
		case FLY_BY_WIRE_A:
			SendSerln("###FBW A***");
			break;
		case FLY_BY_WIRE_B:
			SendSerln("###FBW B***");
			break;
		case AUTO:
			SendSerln("###AUTO***");
			break;
		case RTL:
			SendSerln("###RTL***");
			break;
		case LOITER:
			SendSerln("###LOITER***");
			break;
		case TAKEOFF:
			SendSerln("##TAKEOFF***");
			break;
		case LAND:
			SendSerln("##LAND***");
			break;
	}
}

void print_position(void)
{
	SendSer("!!!");
	SendSer("LAT:");
	SendSer(current_loc.lat/10,DEC);
	SendSer(",LON:");
	SendSer(current_loc.lng/10,DEC); //wp_current_lat
	SendSer(",SPD:");
	SendSer(GPS.ground_speed/100,DEC);		
	SendSer(",CRT:");
	SendSer(climb_rate,DEC);
	SendSer(",ALT:");
	SendSer(current_loc.alt/100,DEC);
	SendSer(",ALH:");
	SendSer(next_WP.alt/100,DEC);
	SendSer(",CRS:");
	SendSer(yaw_sensor/100,DEC);
	SendSer(",BER:");
	SendSer(target_bearing/100,DEC);
	SendSer(",WPN:");
	SendSer(wp_index,DEC);//Actually is the waypoint.
	SendSer(",DST:");
	SendSer(wp_distance,DEC);
	SendSer(",BTV:");
	SendSer(battery_voltage,DEC);
	SendSer(",RSP:");
	SendSer(servo_out[CH_ROLL]/100,DEC);
	SendSer(",TOW:");
	SendSer(GPS.time);
	SendSerln(",***");
}

void print_waypoint(struct Location *cmd,byte index)
{
	SendSer("command #: ");
	SendSer(index, DEC);
	SendSer(" id: ");
	SendSer(cmd->id,DEC);
	SendSer(" p1: ");
	SendSer(cmd->p1,DEC);
	SendSer(" p2: ");
	SendSer(cmd->alt,DEC);
	SendSer(" p3: ");
	SendSer(cmd->lat,DEC);
	SendSer(" p4: ");
	SendSerln(cmd->lng,DEC);
}

void print_waypoints()
{
}

#endif
