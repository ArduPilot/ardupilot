// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

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
void static acknowledge(byte id, byte check1, byte check2) {}
void static send_message(byte id) {}
void static send_message(byte id, long param) {}
void static send_message(byte severity, const char *str) {}
*/

static void acknowledge(byte id, byte check1, byte check2)
{
}

static void send_message(byte severity, const char *str)		// This is the instance of send_message for message 0x05
{
	if(severity >= DEBUG_LEVEL){
		SendSer("MSG: ");
		SendSerln(str);
	}
}

static void send_message(byte id)
{
	send_message(id,0l);
}

static void send_message(byte id, long param)
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

static void print_current_waypoints()
{
}

static void print_attitude(void)
{
	SendSer("+++");
	SendSer("ASP:");
	SendSer((int)airspeed / 100, DEC);
	SendSer(",THH:");
	SendSer(g.rc_3.servo_out, DEC);
	SendSer (",RLL:");
	SendSer(dcm.roll_sensor / 100, DEC);
	SendSer (",PCH:");
	SendSer(dcm.pitch_sensor / 100, DEC);
	SendSerln(",***");
}

static void print_control_mode(void)
{
	SendSer("###");
	SendSer(flight_mode_strings[control_mode]);
	SendSerln("***");
}

static void print_position(void)
{
	SendSer("!!");
	SendSer("!");
	SendSer("LAT:");
	SendSer(current_loc.lat/10,DEC);
	SendSer(",LON:");
	SendSer(current_loc.lng/10,DEC); //wp_current_lat
	SendSer(",SPD:");
	SendSer(g_gps->ground_speed/100,DEC);
	SendSer(",CRT:");
	SendSer(climb_rate,DEC);
	SendSer(",ALT:");
	SendSer(current_loc.alt/100,DEC);
	SendSer(",ALH:");
	SendSer(next_WP.alt/100,DEC);
	SendSer(",CRS:");
	SendSer(dcm.yaw_sensor/100,DEC);
	SendSer(",BER:");
	SendSer(target_bearing/100,DEC);
	SendSer(",WPN:");
	SendSer(g.waypoint_index,DEC);//Actually is the waypoint.
	SendSer(",DST:");
	SendSer(wp_distance,DEC);
	SendSer(",BTV:");
	SendSer(battery_voltage,DEC);
	SendSer(",RSP:");
	SendSer(g.rc_1.servo_out/100,DEC);
	SendSer(",TOW:");
	SendSer(g_gps->time);
	SendSerln(",***");
}

static void print_waypoint(struct Location *cmd,byte index)
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

static void print_waypoints()
{
}

#endif
