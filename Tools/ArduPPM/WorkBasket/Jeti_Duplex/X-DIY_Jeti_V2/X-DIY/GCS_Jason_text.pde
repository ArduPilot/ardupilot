// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-


#if GCS_PROTOCOL == GCS_PROTOCOL_JASON

// this is my personal GCS - Jason


void send_message(byte severity, const char *str)		// This is the instance of send_message for message 0x05
{
	if(severity >= DEBUG_LEVEL){
		Serial.print("MSG: ");
		Serial.println(str);
	}
}

void send_message(byte id)
{
	send_message(id,(long)0);
}

void send_message(byte id, long param)
{

	switch(id) {
		case MSG_HEARTBEAT:
			print_control_mode();
			break;
		
		case MSG_ATTITUDE:
			print_attitude();
			break;
			
		case MSG_LOCATION:
			print_position();
			break;
		
		case MSG_COMMAND:
			struct Location cmd = get_wp_with_index(param);
			print_waypoint(&cmd, param);
			break;
		
        }
}
			
void pipe()
{
	Serial.print("|");
}

void print_current_waypoints()
{
	Serial.print("MSG: ");
	
	Serial.print("CUR:");
	Serial.print("\t");
	Serial.print(current_loc.lat,DEC);					
	Serial.print(",\t");
	Serial.print(current_loc.lng,DEC);					
	Serial.print(",\t");
	Serial.println(current_loc.alt,DEC);					
	
	Serial.print("NWP:");
	Serial.print(wp_index,DEC);
	Serial.print(",\t");
	Serial.print(next_WP.lat,DEC);
	Serial.print(",\t");
	Serial.print(next_WP.lng,DEC);
	Serial.print(",\t");
	Serial.println(next_WP.alt,DEC);
}

void print_position(void)
{
	Serial.print("!!");
	Serial.print(current_loc.lat,DEC);					// 0
	pipe();
	Serial.print(current_loc.lng,DEC);					// 1
	pipe();
	Serial.print(current_loc.alt,DEC);					// 2
	pipe();
	Serial.print(GPS.ground_speed,DEC);						// 3
	pipe();
	Serial.print(airspeed,DEC);					// 4
	pipe();
	Serial.print(get_altitude_above_home(),DEC);		// 5
	pipe();
	Serial.print(climb_rate,DEC);						// 6
	pipe();
	Serial.print(wp_distance,DEC);						// 7
	pipe();
	Serial.print(throttle_cruise,DEC);					// 8
	pipe();
	Serial.println(altitude_error,DEC);					// 9

}

void print_attitude(void)
{
	//return;
	
	Serial.print("++");
	Serial.print(radio_in[0],DEC);					// 0
	pipe();
	Serial.print(radio_in[1],DEC);					// 1
	pipe();
	Serial.print(radio_in[CH_THROTTLE],DEC);		// 2
	pipe();
	Serial.print(roll_sensor,DEC);					// 3
	pipe();
	Serial.print(pitch_sensor,DEC);					// 4
	pipe();
	Serial.print("0");						// 5	ir_max - removed, no longer applicable
	pipe();
	Serial.print(yaw_sensor,DEC);				// 6
	pipe();
	Serial.print(target_bearing,DEC);				// 7
	pipe();
	Serial.print(nav_roll,DEC);					// 8
	pipe();
	Serial.println(loiter_sum,DEC);					// 8
	
}

// required by Groundstation to plot lateral tracking course 
void print_new_wp_info()
{
	Serial.print("??");
	Serial.print(wp_index,DEC);	//0
	pipe();
	Serial.print(prev_WP.lat,DEC);		//1
	pipe();
	Serial.print(prev_WP.lng,DEC);		//2
	pipe();
	Serial.print(prev_WP.alt,DEC);		//3
	pipe();
	Serial.print(next_WP.lat,DEC);		//4
	pipe();
	Serial.print(next_WP.lng,DEC);		//5
	pipe();
	Serial.print(next_WP.alt,DEC);		//6
	pipe();
	Serial.print(wp_totalDistance,DEC);	//7
	pipe();
	Serial.print(radio_trim[CH_ROLL],DEC);	//8
	pipe();
	Serial.println(radio_trim[CH_PITCH],DEC);	//9
}

void print_control_mode(void)
{
	switch (control_mode){
		case MANUAL:
			Serial.println("##MANUAL");
			break;
		case STABILIZE:
			Serial.println("##STABILIZE");
			break;
		case FLY_BY_WIRE_A:
			Serial.println("##FBW A");
			break;
		case FLY_BY_WIRE_B:
			Serial.println("##FBW B");
			break;
		case AUTO:
			Serial.println("##AUTO");
			break;
		case RTL:
			Serial.println("##RTL");
			break;
		case LOITER:
			Serial.println("##LOITER");
			break;
		case 98:
			Serial.println("##Air Start Complete");
			break;
		case 99:
			Serial.println("##Ground Start Complete");
			break;
	}
}

void print_tuning(void) {
	Serial.print("TUN:");
	Serial.print(servo_out[CH_ROLL]/100);
	Serial.print(",	 ");
	Serial.print(nav_roll/100,DEC);
	Serial.print(",	 ");
	Serial.print(roll_sensor/100,DEC);
	Serial.print(",	 ");
	Serial.print(servo_out[CH_PITCH]/100);
	Serial.print(",	 ");
	Serial.print(nav_pitch/100,DEC);
	Serial.print(",	 ");
	Serial.println(pitch_sensor/100,DEC);
}

void printPerfData(void)
{
}


void print_waypoint(struct Location *cmd,byte index)
{
	Serial.print("command #: ");
	Serial.print(index, DEC);
	Serial.print(" id: ");
	Serial.print(cmd->id,DEC);
	Serial.print(" p1: ");
	Serial.print(cmd->p1,DEC);
	Serial.print(" p2: ");
	Serial.print(cmd->alt,DEC);
	Serial.print(" p3: ");
	Serial.print(cmd->lat,DEC);
	Serial.print(" p4: ");
	Serial.println(cmd->lng,DEC);
}

void print_waypoints()
{
	Serial.println("Commands in memory");
	Serial.print("commands total: ");
	Serial.println(wp_total, DEC);
	// create a location struct to hold the temp Waypoints for printing
	//Location tmp;
	Serial.println("Home: ");
	struct Location cmd = get_wp_with_index(0);
	print_waypoint(&cmd, 0);
	Serial.println("Commands: ");
	
	for (int i=1; i<wp_total; i++){
		cmd = get_wp_with_index(i);
		print_waypoint(&cmd, i);
	}
}

#endif

