// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#if GCS_PROTOCOL == GCS_PROTOCOL_IMU

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

void send_message(byte id)
{
	send_message(id,0l);
}
	
void send_message(byte id, long param)
{
	switch(id) {
		case MSG_ATTITUDE:
			print_attitude();
			break;
			
		case MSG_LOCATION:								// ** Location/GPS message
			print_location();
			break;
	}	
}

void send_message(byte severity, const char *str)
{
	if(severity >= DEBUG_LEVEL){
		Serial.print("MSG: ");
		Serial.println(str);
	}
}

void print_current_waypoints()
{
}

void print_control_mode(void)
{
}

void print_attitude(void)
{
	//Serial.print("!!!VER:");
	//Serial.print(SOFTWARE_VER);  //output the software version
	//Serial.print(",");
	
// Analogs
		Serial.print("AN0:");
		Serial.print(read_adc(0)); //Reversing the sign. 
		Serial.print(",AN1:");
		Serial.print(read_adc(1));
		Serial.print(",AN2:");
		Serial.print(read_adc(2));  
		Serial.print(",AN3:");
		Serial.print(read_adc(3));
		Serial.print (",AN4:");
		Serial.print(read_adc(4));
		Serial.print (",AN5:");
		Serial.print(read_adc(5));
		Serial.print (",");
      
// DCM
		Serial.print ("EX0:");
		Serial.print(convert_to_dec(DCM_Matrix[0][0]));
		Serial.print (",EX1:");
		Serial.print(convert_to_dec(DCM_Matrix[0][1]));
		Serial.print (",EX2:");
		Serial.print(convert_to_dec(DCM_Matrix[0][2]));
		Serial.print (",EX3:");
		Serial.print(convert_to_dec(DCM_Matrix[1][0]));
		Serial.print (",EX4:");
		Serial.print(convert_to_dec(DCM_Matrix[1][1]));
		Serial.print (",EX5:");
		Serial.print(convert_to_dec(DCM_Matrix[1][2]));
		Serial.print (",EX6:");
		Serial.print(convert_to_dec(DCM_Matrix[2][0]));
		Serial.print (",EX7:");
		Serial.print(convert_to_dec(DCM_Matrix[2][1]));
		Serial.print (",EX8:");
		Serial.print(convert_to_dec(DCM_Matrix[2][2]));
		Serial.print (",");

// Euler
		Serial.print("RLL:");
		Serial.print(ToDeg(roll));
		Serial.print(",PCH:");
		Serial.print(ToDeg(pitch));
		Serial.print(",YAW:");
		Serial.print(ToDeg(yaw));
		Serial.print(",IMUH:");
		Serial.print(((int)imu_health>>8)&0xff);
		Serial.print (",");
      

	/*
	#if MAGNETOMETER == ENABLED
		Serial.print("MGX:");
		Serial.print(magnetom_x);
		Serial.print (",MGY:");
		Serial.print(magnetom_y);
		Serial.print (",MGZ:");
		Serial.print(magnetom_z);
		Serial.print (",MGH:");
		Serial.print(MAG_Heading);
		Serial.print (",");
	#endif
	*/

   		//Serial.print("Temp:");
		//Serial.print(temp_unfilt/20.0);      // Convert into degrees C
		//alti();
		//Serial.print(",Pressure: ");
		//Serial.print(press);            
		//Serial.print(",Alt: ");
		//Serial.print(press_alt/1000);  // Original floating point full solution in meters
		//Serial.print (",");
	Serial.println("***");
}

void print_location(void)
{
	Serial.print("LAT:");
	Serial.print(current_loc.lat);
	Serial.print(",LON:");
	Serial.print(current_loc.lng);
	Serial.print(",ALT:");
	Serial.print(current_loc.alt/100);    // meters
	Serial.print(",COG:");
	Serial.print(GPS.ground_course);
	Serial.print(",SOG:");
	Serial.print(GPS.ground_speed);
	Serial.print(",FIX:");
	Serial.print((int)GPS.fix);
	Serial.print(",SAT:"); 
	Serial.print((int)GPS.num_sats);
	Serial.print (",");      
	Serial.print("TOW:");
	Serial.print(GPS.time);
	Serial.println("***");
}

void print_waypoints()
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

long convert_to_dec(float x)
{
  return x*10000000;
}

#endif
