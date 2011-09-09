// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

///
/// @file	GCS_Xplane.pde
/// @brief	GCS driver for the X-plane HIL interface.
///

#if GCS_PROTOCOL == GCS_PROTOCOL_XPLANE

void 
GCS_XPLANE::send_text(uint8_t severity, const char *str)
{
	if(severity >= DEBUG_LEVEL){
		Serial.print("MSG: ");
		Serial.println(str);
	}
}

void 
GCS_XPLANE::send_message(uint8_t id, uint32_t param)
{
	switch(id) {
		case MSG_HEARTBEAT:
			print_control_mode();
		break;
		
		case MSG_ATTITUDE:
			//print_attitude();
		break;
		
		case MSG_LOCATION:
			//print_position();
		break;
		
		case MSG_CPU_LOAD:
			//TODO: implement appropriate routine here if applicable
		break;
		
		case MSG_COMMAND_LIST:
			struct Location cmd = get_wp_with_index(param);
			print_waypoint(&cmd, param);
		break;
		
        }
}

void 
GCS_XPLANE::print_control_mode(void)
{
	Serial.print("MSG: ");
	Serial.print(flight_mode_strings[control_mode]);
}

void 
GCS_XPLANE::print_current_waypoints()
{
	Serial.print("MSG: ");
	Serial.print("CUR:");
	Serial.print("\t");
	Serial.print(current_loc.lat,DEC);					
	Serial.print(",\t");
	Serial.print(current_loc.lng,DEC);					
	Serial.print(",\t");
	Serial.println(current_loc.alt,DEC);					
	
	Serial.print("MSG: ");
	Serial.print("NWP:");
	Serial.print(wp_index,DEC);
	Serial.print(",\t");
	Serial.print(next_WP.lat,DEC);
	Serial.print(",\t");
	Serial.print(next_WP.lng,DEC);
	Serial.print(",\t");
	Serial.println(next_WP.alt,DEC);
}

void
GCS_XPLANE::print_waypoint(struct Location *cmd,byte index)
{
	Serial.print("MSG: command #: ");
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

void
GCS_XPLANE::print_waypoints()
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

#endif // GCS_PROTOCOL_XPLANE
