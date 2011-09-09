// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

//Function that will read the radio data, limit servos and trigger a failsafe
// ----------------------------------------------------------------------------
byte failsafeCounter = 0;		// we wait a second to take over the throttle and send the plane circling

void read_radio()
{
	ch1_temp = APM_RC.InputCh(CH_ROLL);
	ch2_temp = APM_RC.InputCh(CH_PITCH);
	
	if(mix_mode == 0){
		radio_in[CH_ROLL] = ch1_temp;
		radio_in[CH_PITCH] = ch2_temp;
	}else{
		radio_in[CH_ROLL] 	= reverse_elevons * (reverse_ch2_elevon * int(ch2_temp - elevon2_trim) - reverse_ch1_elevon * int(ch1_temp - elevon1_trim)) / 2 + 1500;
		radio_in[CH_PITCH] 	= (reverse_ch2_elevon * int(ch2_temp - elevon2_trim) + reverse_ch1_elevon * int(ch1_temp - elevon1_trim)) / 2 + 1500;
	}

	for (int y = 2; y < 8; y++)
		radio_in[y] = APM_RC.InputCh(y);
	
	#if THROTTLE_REVERSE == 1
		radio_in[CH_THROTTLE] = radio_max(CH_THROTTLE) + radio_min(CH_THROTTLE) - radio_in[CH_THROTTLE];
	#endif
	
	throttle_failsafe(radio_in[CH_THROTTLE]);
	servo_out[CH_THROTTLE] = ((float)(radio_in[CH_THROTTLE] - radio_min(CH_THROTTLE)) /
							  (float)(radio_max(CH_THROTTLE) - radio_min(CH_THROTTLE))) * 100;
	servo_out[CH_THROTTLE] = constrain(servo_out[CH_THROTTLE], 0, 100);
	
	if (servo_out[CH_THROTTLE] > 50) {
		if(AIRSPEED_SENSOR == 1) {
			airspeed_nudge = (get(PARAM_ARSPD_FBW_MAX) - get(PARAM_TRIM_AIRSPEED)) * ((servo_out[CH_THROTTLE]-50) * 2);
		} else {
			throttle_nudge = (get(PARAM_THR_MAX) - get(PARAM_TRIM_THROTTLE)) * ((servo_out[CH_THROTTLE]-50) * 2);
		}
	} else {
		airspeed_nudge = 0;
		throttle_nudge = 0;
	}
}

void throttle_failsafe(uint16_t pwm)
{
	if(get(PARAM_THR_FAILSAFE)== 0)
		return;
		
	//check for failsafe and debounce funky reads
	// ------------------------------------------
	if (pwm < get(PARAM_THR_FS_VALUE)){
		// we detect a failsafe from radio 
		// throttle has dropped below the mark
		failsafeCounter++;
		if (failsafeCounter == 9){
			SendDebug("MSG FS ON ");
			SendDebugln(pwm, DEC);
		}else if(failsafeCounter == 10) {
			ch3_failsafe = true;
			//set_failsafe(true);
			//failsafeCounter = 10;
		}else if (failsafeCounter > 10){
			failsafeCounter = 11;
		}
		
	}else if(failsafeCounter > 0){
		// we are no longer in failsafe condition
		// but we need to recover quickly		
		failsafeCounter--;
		if (failsafeCounter > 3){
			failsafeCounter = 3;
		}		
		if (failsafeCounter == 1){
			SendDebug("MSG FS OFF ");
			SendDebugln(pwm, DEC);
		}else if(failsafeCounter == 0) {
			ch3_failsafe = false;
			//set_failsafe(false);
			//failsafeCounter = -1;
		}else if (failsafeCounter <0){
			failsafeCounter = -1;
		}
	}
}

void trim_control_surfaces()
{
	// Store control surface trim values
	// ---------------------------------
	if(mix_mode == 0){
		set_radio_trim(CH_ROLL,radio_in[CH_ROLL]);
		set_radio_trim(CH_PITCH,radio_in[CH_PITCH]);
		set_radio_trim(CH_RUDDER,radio_in[CH_RUDDER]);
	}else{
		elevon1_trim = ch1_temp;
		elevon2_trim = ch2_temp;
		//Recompute values here using new values for elevon1_trim and elevon2_trim 
		//We cannot use radio_in[CH_ROLL] and radio_in[CH_PITCH] values from read_radio() because the elevon trim values have changed
		uint16_t center = 1500;	
		set_radio_trim(CH_ROLL,center);
		set_radio_trim(CH_PITCH,center);
	}
	// disabled for now
	//save_EEPROM_trims();
}

void trim_radio()
{	
	for (int y = 0; y < 50; y++) {
		read_radio();
	}

	// Store the trim values
	// ---------------------
	if(mix_mode == 0){
		for (int y = 0; y < 8; y++) set_radio_trim(y,radio_in[y]);	
	}else{
		elevon1_trim = ch1_temp;
		elevon2_trim = ch2_temp;
		uint16_t center = 1500;	
		set_radio_trim(CH_ROLL,center);
		set_radio_trim(CH_PITCH,center);
		for (int y = 2; y < 8; y++) set_radio_trim(y,radio_in[y]);	
	}
	//save_EEPROM_trims();
}


#if SET_RADIO_LIMITS == 1
void read_radio_limits()
{
	// set initial servo limits for calibration routine
	// -------------------------------------------------
	set_radio_min(CH_ROLL,radio_in[CH_ROLL] - 150);
	set_radio_max(CH_ROLL,radio_in[CH_ROLL] + 150);

	set_radio_min(CH_PITCH,radio_in[CH_PITCH] - 150);
	set_radio_max(CH_PITCH,radio_in[CH_PITCH] + 150);

	// vars for the radio config routine
	// ---------------------------------
	int counter 	= 0;
	long reminder;
		reminder 		= millis() - 10000;

	uint16_t tempMin[2] = {radio_min(CH_ROLL), radio_min(CH_PITCH)}	;
	uint16_t tempMax[2] = {radio_max(CH_ROLL), radio_max(CH_PITCH)}	;

	// Allows user to set stick limits and calibrate the IR
	// ----------------------------------------------------
	while(counter < 50){
	
				if (millis() - reminder >= 10000) {							// Remind user every 10 seconds what is going on
			GCS.send_text(SEVERITY_LOW,"Reading radio limits:");
			GCS.send_text(SEVERITY_LOW,"Move sticks to: upper right and lower Left");
			GCS.send_text(SEVERITY_LOW,"To Continue, hold the stick in the corner for 2 seconds.");
			GCS.send_text(SEVERITY_LOW," ");
	        //print_radio();
			demo_servos(1);
			reminder = millis();
		}
						 
		delay(40);
		read_radio();

		// AutoSet servo limits
		// --------------------
		if (radio_in[CH_ROLL] > 1000 && radio_in[CH_ROLL] < 2000){
			tempMin[CH_ROLL] = min(radio_in[CH_ROLL], tempMin[CH_ROLL]);
			tempMax[CH_ROLL] = max(radio_in[CH_ROLL], tempMax[CH_ROLL]);
		}
		
		if (radio_in[CH_PITCH] > 1000 && radio_in[CH_PITCH]< 2000){
			tempMin[CH_PITCH] = min(radio_in[CH_PITCH], tempMin[CH_PITCH]);
			tempMax[CH_PITCH] = max(radio_in[CH_PITCH], tempMax[CH_PITCH]);
		}
		if(radio_in[CH_PITCH] < (tempMin[CH_PITCH] + 30) || tempMin[CH_PITCH] > (tempMax[CH_PITCH] -30)){
			SendDebug(".");
			counter++;
		}else{
			if (counter > 0)
				counter--;
		}
	}
	
	// contstrain min values
	// ---------------------
	set_radio_min(CH_ROLL,constrain(tempMin[CH_ROLL], 800, 2200));
	set_radio_max(CH_ROLL,constrain(tempMax[CH_ROLL], 800, 2200));
	set_radio_min(CH_PITCH,constrain(tempMin[CH_PITCH], 800, 2200));
	set_radio_max(CH_PITCH,constrain(tempMax[CH_PITCH], 800, 2200));
	
	SendDebugln(" ");
}
#endif




