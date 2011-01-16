void read_control_switch()
{
	byte switchPosition = readSwitch();
	//motor_armed = (switchPosition < 5);
	
	if (oldSwitchPosition != switchPosition){
		if(motor_armed)
			Serial.println("motor_armed");
		else
			Serial.println("motor disarmed");
				
		set_mode(flight_modes[switchPosition]);
		
		oldSwitchPosition = switchPosition;

		// reset navigation integrators
		// -------------------------
		reset_I();
	}
}

byte readSwitch(void){
#if   FLIGHT_MODE_CHANNEL == CH_5
	int pulsewidth = rc_5.radio_in;			// default for Arducopter
#elif FLIGHT_MODE_CHANNEL == CH_6
	int pulsewidth = rc_6.radio_in;			//
#elif FLIGHT_MODE_CHANNEL == CH_7
	int pulsewidth = rc_7.radio_in;			//
#elif FLIGHT_MODE_CHANNEL == CH_8
	int pulsewidth = rc_8.radio_in;			// default for Ardupilot. Don't use for Arducopter! it has a hardware failsafe mux! 
#else
# error Must define FLIGHT_MODE_CHANNEL as CH_5 - CH_8
#endif

	if (pulsewidth > 1230 && pulsewidth <= 1360) 	return 1;
	if (pulsewidth > 1360 && pulsewidth <= 1490) 	return 2;
	if (pulsewidth > 1490 && pulsewidth <= 1620) 	return 3;
	if (pulsewidth > 1620 && pulsewidth <= 1749) 	return 4;	// Software Manual
	if (pulsewidth >= 1750) 						return 5;	// Hardware Manual
	return 0;
}

void reset_control_switch()
{
	oldSwitchPosition = -1;
	read_control_switch();
	//Serial.print("MSG: reset_control_switch");
	//Serial.println(oldSwitchPosition , DEC);
}

void update_servo_switches()
{

}

boolean trim_flag;
unsigned long trim_timer;

// read at 10 hz
// set this to your trainer switch
void read_trim_switch()
{
	// switch is engaged
	if (rc_7.control_in > 500){
		if(trim_flag == false){
			// called once
			trim_timer = millis();
		}
		
		trim_flag = true;
		trim_accel();
		
		
	}else{ // switch is disengaged
	
		if(trim_flag){
			// switch was just released
			if((millis() - trim_timer) > 2000){

				// not being used
				
			} else {
			
				// set the throttle nominal
				if(rc_3.control_in > 50){
					throttle_cruise = rc_3.control_in;
					//Serial.print("tnom ");
					//Serial.println(throttle_cruise, DEC);
					save_EEPROM_throttle_cruise();
				}
			}
			trim_flag = false;
		}
	}
}


void trim_accel()
{
	if(rc_1.control_in > 0){
		imu.ay(imu.ay() - 1);
	}else if (rc_1.control_in < 0){
		imu.ay(imu.ay() + 1);
	}

	if(rc_2.control_in > 0){
		imu.ax(imu.ax() - 1);
	}else if (rc_2.control_in < 0){
		imu.ax(imu.ax() + 1);
	}

	Serial.printf("r:%ld p:%ld ax:%d, ay:%d, az:%d\n", dcm.roll_sensor, dcm.pitch_sensor, imu.ax(), imu.ay(), imu.az());
}

