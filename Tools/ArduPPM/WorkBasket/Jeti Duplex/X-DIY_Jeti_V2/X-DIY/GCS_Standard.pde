// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
// Doug _ command index is a byte and not an int

#if GCS_PROTOCOL == GCS_PROTOCOL_STANDARD

#if GCS_PORT == 3
# define SendSer		Serial3.print
#else
# define SendSer		Serial.print
#endif

// The functions included in this file are for use with the standard binary communication protocol and standard Ground Control Station

void acknowledge(byte id, byte check1, byte check2) {
	byte mess_buffer[6];
	byte mess_ck_a = 0;
	byte mess_ck_b = 0;
	int ck;
			
	SendSer("4D");  // This is the message preamble
	mess_buffer[0] = 0x03;
	ck = 3;
	mess_buffer[1] = 0x00;	 	// Message ID
	mess_buffer[2] = 0x01;			// Message version

	mess_buffer[3] = id;
	mess_buffer[4] = check1;
	mess_buffer[5] = check2;
			

	for (int i = 0; i < ck + 3; i++) SendSer (mess_buffer[i]);
		
	for (int i = 0; i < ck + 3; i++) {
		mess_ck_a += mess_buffer[i];	// Calculates checksums
		mess_ck_b += mess_ck_a;			 
	}
	SendSer(mess_ck_a);
	SendSer(mess_ck_b);
}

void send_message(byte id) {
	send_message(id, 0l);
}
	
void send_message(byte id, long param) {
	byte mess_buffer[54];
	byte mess_ck_a = 0;
	byte mess_ck_b = 0;
	int tempint;
	int ck;
	long templong;
			
	SendSer("4D");  // This is the message preamble
		
	switch(id) {
		case MSG_HEARTBEAT:						// ** System Status message
		mess_buffer[0] = 0x07;
		ck = 7;
		mess_buffer[3] = control_mode;				// Mode
		templong = millis() / 1000;				// Timestamp - Seconds since power - up
		mess_buffer[4] = templong & 0xff;
		mess_buffer[5] = (templong >> 8) & 0xff;
		tempint = battery_voltage1 * 100;			// Battery voltage ( * 100)
		mess_buffer[6] = tempint & 0xff;
		mess_buffer[7] = (tempint >> 8) & 0xff;
		tempint = command_must_index;			// Command Index (waypoint level)
		mess_buffer[8] = tempint & 0xff;
		mess_buffer[9] = (tempint >> 8) & 0xff;
		break;
		
		case MSG_ATTITUDE:								// ** Attitude message
		mess_buffer[0] = 0x06;
		ck = 6;
		tempint = roll_sensor;					// Roll (degrees * 100)
		mess_buffer[3] = tempint & 0xff;
		mess_buffer[4] = (tempint >> 8) & 0xff;
		tempint = pitch_sensor;					// Pitch (degrees * 100)
		mess_buffer[5] = tempint & 0xff;
		mess_buffer[6] = (tempint >> 8) & 0xff;
		tempint = yaw_sensor;					// Yaw (degrees * 100)
		mess_buffer[7] = tempint & 0xff;
		mess_buffer[8] = (tempint >> 8) & 0xff;
		break;
		
		case MSG_LOCATION:								// ** Location / GPS message
		mess_buffer[0] = 0x12;
		ck = 18;
		templong = current_loc.lat; 			// Latitude *10 * *7 in 4 bytes
		mess_buffer[3] = templong & 0xff;
		mess_buffer[4] = (templong >> 8) & 0xff;
		mess_buffer[5] = (templong >> 16) & 0xff;
		mess_buffer[6] = (templong >> 24) & 0xff;
			
		templong = current_loc.lng; 			// Longitude *10 * *7 in 4 bytes
		mess_buffer[7] = templong & 0xff;
		mess_buffer[8] = (templong >> 8) & 0xff;
		mess_buffer[9] = (templong >> 16) & 0xff;
		mess_buffer[10] = (templong >> 24) & 0xff;
			
		tempint = GPS.altitude / 100;			 		// Altitude MSL in meters * 10 in 2 bytes
		mess_buffer[11] = tempint & 0xff;
		mess_buffer[12] = (tempint >> 8) & 0xff;
			
		tempint = GPS.ground_speed;	 			// Speed in M / S * 100 in 2 bytes
		mess_buffer[13] = tempint & 0xff;
		mess_buffer[14] = (tempint >> 8) & 0xff;
				
		tempint = yaw_sensor;					// Ground Course in degreees * 100 in 2 bytes
		mess_buffer[15] = tempint & 0xff;
		mess_buffer[16] = (tempint >> 8) & 0xff;
				
		templong = GPS.time;						// Time of Week (milliseconds) in 4 bytes
		mess_buffer[17] = templong & 0xff;
		mess_buffer[18] = (templong >> 8) & 0xff;
		mess_buffer[19] = (templong >> 16) & 0xff;
		mess_buffer[20] = (templong >> 24) & 0xff;
		break;
		
		case MSG_PRESSURE:								// ** Pressure message
		mess_buffer[0] = 0x04;
		ck = 4;
		tempint 		= current_loc.alt / 10;			 	// Altitude MSL in meters * 10 in 2 bytes
		mess_buffer[3] 	= tempint & 0xff;
		mess_buffer[4] 	= (tempint >> 8) & 0xff;
		tempint 		= (int)airspeed / 100;				// Airspeed pressure
		mess_buffer[5] 	= tempint & 0xff;
		mess_buffer[6] 	= (tempint >> 8) & 0xff;
		break;
		
//		case 0xMSG_STATUS_TEXT:								// ** Status Text message
//		mess_buffer[0]=sizeof(status_message[0])+1;
//		ck=mess_buffer[0];
//		mess_buffer[2] = param&0xff;
//		for (int i=3;i<ck+2;i++) mess_buffer[i] = status_message[i-3];
//		break;
		
		case MSG_PERF_REPORT:								// ** Performance Monitoring message
		mess_buffer[0] = 0x10;
		ck = 16;
		templong 		= millis() - perf_mon_timer; 	// Report interval (milliseconds) in 4 bytes
		mess_buffer[3] 	= templong & 0xff;
		mess_buffer[4] 	= (templong >> 8) & 0xff;
		mess_buffer[5] 	= (templong >> 16) & 0xff;
		mess_buffer[6] 	= (templong >> 24) & 0xff;
		tempint 		= mainLoop_count;				// Main Loop cycles
		mess_buffer[7] 	= tempint & 0xff;
		mess_buffer[8] 	= (tempint >> 8) & 0xff;
		mess_buffer[9] 	= G_Dt_max & 0xff;
		mess_buffer[10] = gyro_sat_count;			// Problem counts
		mess_buffer[11] = adc_constraints;
		mess_buffer[12] = renorm_sqrt_count;
		mess_buffer[13] = renorm_blowup_count;
		mess_buffer[14] = gps_fix_count;
		tempint 		= (int)(imu_health * 1000);		// IMU health metric
		mess_buffer[15] = tempint & 0xff;
		mess_buffer[16] = (tempint >> 8) & 0xff;
		tempint 		= gcs_messages_sent;			// GCS messages sent
		mess_buffer[17] = tempint & 0xff;
		mess_buffer[18] = (tempint >> 8) & 0xff;
		break;
		
		case MSG_VALUE:								// ** Requested Value message
		mess_buffer[0] = 0x06;
		ck = 6;
		templong = param;			 			// Parameter being sent
		mess_buffer[3] = templong & 0xff;
		mess_buffer[4] = (templong >> 8) & 0xff;
		switch(param) {
//			case 0x00:		templong = roll_mode;				break;
//			case 0x01:		templong = pitch_mode;				break;
			case 0x02:		templong = yaw_mode;				break;
//			case 0x03:		templong = throttle_mode;			break;
			case 0x04:		templong = elevon1_trim;			break;
			case 0x05:		templong = elevon2_trim;			break;
			case 0x10:		templong = integrator[0] * 1000;	break;
			case 0x11:		templong = integrator[1] * 1000;	break;
			case 0x12:		templong = integrator[2] * 1000;	break;
			case 0x13:		templong = integrator[3] * 1000;	break;
			case 0x14:		templong = integrator[4] * 1000;	break;
			case 0x15:		templong = integrator[5] * 1000;	break;
			case 0x16:		templong = integrator[6] * 1000;	break;
			case 0x17:		templong = integrator[7] * 1000;	break;
			case 0x1a:		templong = kff[0];					break;
			case 0x1b:		templong = kff[1];					break;
			case 0x1c:		templong = kff[2];					break;
			case 0x20:		templong = target_bearing;			break;
			case 0x21:		templong = nav_bearing;				break;
			case 0x22:		templong = bearing_error;			break;
			case 0x23:		templong = crosstrack_bearing;		break;
			case 0x24:		templong = crosstrack_error;		break;
			case 0x25:		templong = altitude_error;			break;
			case 0x26:		templong = wp_radius;				break;
			case 0x27:		templong = loiter_radius;			break;
//			case 0x28:		templong = wp_mode;					break;
//			case 0x29:		templong = loop_commands;			break;
			case 0x2a:		templong = nav_gain_scaler;			break;
		}
		mess_buffer[5] = templong & 0xff;
		mess_buffer[6] = (templong >> 8) & 0xff;
		mess_buffer[7] = (templong >> 16) & 0xff;
		mess_buffer[8] = (templong >> 24) & 0xff;
		break;
		
		case MSG_COMMAND:						// Command list item message
		mess_buffer[0] = 0x10;
		ck = 16;
		tempint = param;			 				// item number
		mess_buffer[3] 	= tempint & 0xff;
		mess_buffer[4] 	= (tempint >> 8) & 0xff;
		tempint 		= wp_total;			 		// list length (# of commands in mission)
		mess_buffer[5] 	= tempint & 0xff;
		mess_buffer[6] 	= (tempint >> 8) & 0xff;
		tell_command 	= get_wp_with_index((int)param);
		mess_buffer[7]	= tell_command.id;			// command id
		mess_buffer[8]	= tell_command.p1;			// P1
		tempint			= tell_command.alt;				// P2
		mess_buffer[9]	= tempint & 0xff;
		mess_buffer[10] = (tempint >> 8) & 0xff;
		templong 		= tell_command.lat; 			// P3
		mess_buffer[11] = templong & 0xff;
		mess_buffer[12] = (templong >> 8) & 0xff;
		mess_buffer[13] = (templong >> 16) & 0xff;
		mess_buffer[14] = (templong >> 24) & 0xff;
		templong 		= tell_command.lng; 			// P4
		mess_buffer[15] = templong & 0xff;
		mess_buffer[16] = (templong >> 8) & 0xff;
		mess_buffer[17] = (templong >> 16) & 0xff;
		mess_buffer[18] = (templong >> 24) & 0xff;
		break;

		case MSG_TRIMS:								// Radio Trims message
		mess_buffer[0] = 0x10;
		ck = 16;
		for(int i = 0; i < 8; i++) {	
			tempint = radio_trim[i];			 	// trim values
			mess_buffer[3 + 2 * i] = tempint & 0xff;
			mess_buffer[4 + 2 * i] = (tempint >> 8) & 0xff;
		}
		break;
		
		case MSG_MINS:								// Radio Mins message
		mess_buffer[0] = 0x10;
		ck = 16;
		for(int i = 0; i < 8; i++) {	
			tempint = radio_min[i];			 		// min values
			mess_buffer[3 + 2 * i] = tempint & 0xff;
			mess_buffer[4 + 2 * i] = (tempint >> 8) & 0xff;
		}
		break;
		
		case MSG_MAXS:								// Radio Maxs message
		mess_buffer[0] = 0x10;
		ck = 16;
		for(int i = 0; i < 8; i++) {	
			tempint = radio_max[i];				 	// max values
			mess_buffer[3 + 2 * i] = tempint & 0xff;
			mess_buffer[4 + 2 * i] = (tempint >> 8) & 0xff;
		}
		break;
		
		case MSG_PID:								// PID Gains message
		mess_buffer[0] 	= 0x0f;
		ck 				= 15;
		mess_buffer[3] 	= param & 0xff;				// PID set #
		templong 		= (kp[param] * 1000000);		 	// P gain
		mess_buffer[4] 	= templong & 0xff;
		mess_buffer[5] 	= (templong >> 8) & 0xff;
		mess_buffer[6] 	= (templong >> 16) & 0xff;
		mess_buffer[7] 	= (templong >> 24) & 0xff;
		templong 		= (ki[param] * 1000000);		 	// I gain
		mess_buffer[8] 	= templong & 0xff;
		mess_buffer[9] 	= (templong >> 8) & 0xff;
		mess_buffer[10] = (templong >> 16) & 0xff;
		mess_buffer[11] = (templong >> 24) & 0xff;
		templong 		= (kd[param] * 1000000);		 	// D gain
		mess_buffer[12] = templong & 0xff;
		mess_buffer[13] = (templong >> 8) & 0xff;
		mess_buffer[14] = (templong >> 16) & 0xff;
		mess_buffer[15] = (templong >> 24) & 0xff;
		tempint 		= integrator_max[param];				 	// Integrator max value
		mess_buffer[16] = tempint & 0xff;
		mess_buffer[17] = (tempint >> 8) & 0xff;
		break;
	}
	
	mess_buffer[1] = id; 		// Message ID
	mess_buffer[2] = 0x01;			// Message version		 

	for (int i = 0; i < ck + 3; i++) SendSer (mess_buffer[i]);
		
	for (int i = 0; i < ck + 3; i++) {
		mess_ck_a += mess_buffer[i];	// Calculates checksums
		mess_ck_b += mess_ck_a;			 
	}

	SendSer(mess_ck_a);
	SendSer(mess_ck_b);

	gcs_messages_sent++;
}

void send_message(byte severity, const char *str)		// This is the instance of send_message for message MSG_STATUS_TEXT
{
	if(severity >= DEBUG_LEVEL){	
		byte length = strlen(str) + 1;
	
		byte mess_buffer[54];
		byte mess_ck_a = 0;
		byte mess_ck_b = 0;
		int ck;
			
		SendSer("4D");  			// This is the message preamble
		if(length > 50) length = 50;
			mess_buffer[0] = length;
		ck = length;
		mess_buffer[1] = 0x05;	 	// Message ID
		mess_buffer[2] = 0x01;		// Message version
	
		mess_buffer[3] = severity;
		
		for (int i = 3; i < ck + 2; i++)
			mess_buffer[i] = str[i - 3];		// places the text into mess_buffer at locations 3+			
	
		for (int i = 0; i < ck + 3; i++)
			SendSer(mess_buffer[i]);
			
		for (int i = 0; i < ck + 3; i++) {
			mess_ck_a += mess_buffer[i];	// Calculates checksums
			mess_ck_b += mess_ck_a;			 
		}
		SendSer(mess_ck_a);
		SendSer(mess_ck_b);
	}
}

void print_current_waypoints()
{
}

void print_waypoint(struct Location *cmd, byte index)
{
	Serial.print("command #: ");
	Serial.print(index, DEC);
	Serial.print(" id: ");
	Serial.print(cmd->id, DEC);
	Serial.print(" p1: ");
	Serial.print(cmd->p1, DEC);
	Serial.print(" p2: ");
	Serial.print(cmd->alt, DEC);
	Serial.print(" p3: ");
	Serial.print(cmd->lat, DEC);
	Serial.print(" p4: ");
	Serial.println(cmd->lng, DEC);
}

void print_waypoints()
{
}


#endif

#if GCS_PROTOCOL == GCS_PROTOCOL_NONE
void acknowledge(byte id, byte check1, byte check2) {}
void send_message(byte id) {}
void send_message(byte id, long param) {}
void send_message(byte severity, const char *str) {}
void print_current_waypoints(){}
void print_waypoint(struct Location *cmd, byte index){}
void print_waypoints(){}
#endif
