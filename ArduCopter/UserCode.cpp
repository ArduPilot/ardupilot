#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up


	//Servo Voltage Watcher
	AP_Notify::flags.low_servo_voltage = false;

	//Killswitch Variables
	killswitch_counter = 0;

	//Herelink Gimbal Control Variables
	function_counter = 0;
	cam_button_debounce_timer = 0;

	ch7_button_pressed = false;
	ch9_button_pressed = false;
	ch10_button_pressed = false;
	ch11_button_pressed = false;
	ch12_button_pressed = false;

	long_press_flag_ch7 = false;
	long_press_flag_ch9 = false;
	long_press_flag_ch10 = false;
	long_press_flag_ch11 = false;
	long_press_flag_ch12 = false;

	short_press_flag_ch7 = false;
	short_press_flag_ch9 = false;
	short_press_flag_ch10 = false;
	short_press_flag_ch11 = false;
	short_press_flag_ch12 = false;

	ch7_button_hold = false;
	ch9_button_hold = false;
	ch10_button_hold = false;
	ch11_button_hold = false;
	ch12_button_hold = false;

	// startup spirit state
	spirit_state = disarm;

	gimbal_pan_spd = 0.0; //not used
	gimbal_tilt_spd = 0.0; //not used

	zoom_in = false;
	zoom_out  = false;
	zoom_stop = true;
	camera_mount.set_mode_to_default();


	/*
	roi_1.lng = g.roi_1_long;
	roi_1.lat = g.roi_1_lat;
	roi_1.set_alt_cm(0, Location::AltFrame::ABOVE_TERRAIN);

//camera_mount.set_roi_target(roi_1);

copter.mode_auto.mission.clear();

AP_Mission::Mission_Command cmd_roi;

  cmd_roi.id = MAV_CMD_DO_SET_ROI;
  cmd_roi.index = 0;
  cmd_roi.p1 = 0;
  cmd_roi.content.location = Location{
	  roi_1.lat,
	  roi_1.lng,
	  roi_1.alt,
      Location::AltFrame::ABOVE_TERRAIN


  };


  copter.mode_auto.mission.add_cmd(cmd_roi);

*/

	//GPIOs for on/off to payload and critical systems

	hal.gpio->pinMode(52, 1);  //Configure as output
	hal.gpio->pinMode(53, 1);  //Configure as output

	hal.gpio->write(52, false);	// Payload Power on at startup
	hal.gpio->write(53, true);  //Turn ESC/Servos on at startup

	//Spool up support
	timer_trigger = false;


	//Set starting RPM if the number of vehicle batteries is set
	float _RPM_hover;

	if(g.battery_number > 0){

		vehicle_weight = 4.0 + ((float)g.battery_number*3.0) + g.payload_weight;
		vehicle_weight = constrain_float(vehicle_weight, 8.0, 13.5);

		_RPM_hover = 156*vehicle_weight + 1553;
		g.rpm_hover = _RPM_hover;

	}

	//RPM compensation support
	start_rpm_comp_time = 0;
	hover_rpm_filter.set_cutoff_frequency(50.0f, 0.25f);
	hover_rpm_filter.reset(g.rpm_hover);
	motors->set_hover_RPM(g.rpm_hover);
	motors->set_aft_rotor_RPM(0.0f);

	num_battery = g.battery_number;
	payload_weight = g.payload_weight;


   if(g.battery_number != 0){
    	auto_config();
    }

/*
   if(g.top_cam){
	   camera_mount.flip_image(true);
   }else{
	   camera_mount.flip_image(false);
   }
*/

}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here

}


#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{

	//detect button input in Herelink at 50Hz
	if(g.herelink_enable){
		Detect_Buttons();
	}

	update_rpm_hover();

	motors->set_aft_rotor_RPM((float)rpm_sensor.get_rpm(1));


	// Don't do this logging
	/*
	float pitch_offset = motors->get_pitch_rpm_offset();
	float roll_offset = motors->get_roll_rpm_offset();
	float hover_rpm = motors->get_hover_RPM();
	Log_Write_Vehicle_State(pitch_offset, roll_offset, hover_rpm);
	 */

		//Call topple sense
	if(motors->armed()){
		topple_sense();
	}


	// State Machine

	if(!motors->armed()){
		spirit_state = disarm;

	//the rest considers the vehicle to be armed
	//always move through spoolup
	}else if(spirit_state == disarm){

		hal.gpio->write(52, false);
		hal.gpio->write(53, true);

		spirit_state = spoolup;
		spoolup_watcher = AP_HAL::millis16();
		gcs().send_text(MAV_SEVERITY_INFO,"SpoolUp");

	}else if(spirit_state == land and (copter.flightmode->is_taking_off() or !ap.land_complete)){

		spirit_state = takeoff;

	}else if(spirit_state == takeoff and (!copter.flightmode->is_taking_off() or copter.flightmode->has_manual_throttle()) ){

		spirit_state = hover;
	//	camera_mount.set_mode(MAV_MOUNT_MODE_RC_TARGETING);

	}else if((spirit_state == hover or spirit_state == landing) and ap.land_complete){

		spirit_state = land;

	}




	switch(spirit_state){


	case disarm:

		//zero out dynamic trim for now
		motors->set_dynamic_trim(0.0, 0.0);


		//Prompt for flashing yellow LEDs if no RC input
		if(RC_Channels::rc_channel(CH_3)->get_radio_in() == 0){
			AP_Notify::flags.no_RC_in = true;
		}else{
			AP_Notify::flags.no_RC_in = false;
		}

		break;

	case spoolup:

		//Looks for issues with startup
		servo_voltage_watcher();

		//Spoolup Watcher.  Disarm if 4 seconds passes without advancing
		if(AP_HAL::millis16() - spoolup_watcher > (uint16_t)4000){
			 copter.arming.disarm();
			 gcs().send_text(MAV_SEVERITY_CRITICAL,"A Rotor Failed to Start: Disarm");
		}

		_fwd_rpm = rpm_sensor.get_rpm(0);
		_aft_rpm = rpm_sensor.get_rpm(1);

		if(_fwd_rpm > 1000.0f){

			if(!timer_trigger){
				spoolup_timer =  AP_HAL::millis16();
				spoolup_watcher = AP_HAL::millis16();
				timer_trigger = true;
			}

			if(_fwd_rpm >= 1000.0f and _aft_rpm >= 1000.0f){  //fully spun up
				motors->spoolup_complete(true);
				spirit_state = land;
				timer_trigger = false;

			}else if((AP_HAL::millis16() - spoolup_timer) > (uint16_t)(g.spool_delta*1000)){
				motors->enable_aft_rotor(true);
			}else{
				spoolup_watcher = AP_HAL::millis16();
			}
		}

        attitude_control->reset_rate_controller_I_terms();
        pos_control->relax_alt_hold_controllers(0.0f);

		break;

	case land:

		break;

	case takeoff:

		break;


	case hover:


		//////// Determine if we are landing //////
		if(copter.flightmode->get_alt_above_ground_cm() <= (int32_t)g2.land_alt_low){   // need to be close to the ground

			if(copter.flightmode->is_landing()){
				spirit_state = landing;
				attitude_control->enable_angle_boost(false);
				//gcs().send_text(MAV_SEVERITY_INFO,"L:auto");
				break;
			// for manual also need to be decending
			}else if(!copter.flightmode->is_autopilot() and copter.flightmode->get_pilot_desired_climb_rate((float)channel_throttle->get_control_in()) < 0.0f){
				spirit_state = landing;
				attitude_control->enable_angle_boost(false);
				//gcs().send_text(MAV_SEVERITY_INFO,"L:dct");
				break;
			}
		}

		break;

	case landing:

		//////cancel landing if no longer auto landing or if pilot is not commanding a decent
			if(!copter.flightmode->is_autopilot() and copter.flightmode->get_pilot_desired_climb_rate(channel_throttle->get_control_in()) >= 0.0f){
				spirit_state = hover;
				attitude_control->enable_angle_boost(true);
			//	gcs().send_text(MAV_SEVERITY_INFO,"H:cr");
				break;
			}

			if(copter.flightmode->is_autopilot() and !copter.flightmode->is_landing()){
				spirit_state = hover;
				attitude_control->enable_angle_boost(true);
			//	gcs().send_text(MAV_SEVERITY_INFO,"H:!land");
				break;
			}

		break;

		}

}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here

	Killswitch();


	if(spirit_state < 4){  //if state is disarmed, spoolup or takeoff, don't use comp and set timer
		motors->enable_rpm_comp(false);
		start_rpm_comp_time = AP_HAL::millis16();
	}else if(AP_HAL::millis16() - start_rpm_comp_time > 3000){
		motors->enable_rpm_comp(true);
	}


}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here

	if(g.herelink_enable){

		Decode_Buttons();

	}


	if( g.battery_number != num_battery){
			auto_config();
			num_battery = g.battery_number;
	}


	if( g.payload_weight != payload_weight){
			auto_config();
			payload_weight = g.payload_weight;
	}

}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here

	if(!motors->armed() and copter.battery.voltage() < 36.0 and (copter.battery.voltage() > 5.0 or !hal.gpio->usb_connected())){
		//AP_Notify::flags.critical_battery_voltage = true;
		gcs().send_text(MAV_SEVERITY_CRITICAL,"Battery Critical");
		hal.gpio->write(52, true);
	}else{
		//AP_Notify::flags.critical_battery_voltage = false;
		hal.gpio->write(52, false);
	}

/*
	   if(g.top_cam){
		   camera_mount.flip_image(true);
	   }else{
		   camera_mount.flip_image(false);
	   }
*/
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(uint8_t ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
}

void Copter::userhook_auxSwitch2(uint8_t ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(uint8_t ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif




void Copter::topple_sense(){

		if(spirit_state == spoolup){
			if(labs(ahrs.pitch_sensor) >1500 or labs(ahrs.roll_sensor) > 1500){
				 copter.arming.disarm();
				 gcs().send_text(MAV_SEVERITY_CRITICAL,"Topple spoolup");
			}

		}else if(spirit_state == takeoff){
			if(fabsf(attitude_control->get_att_error_angle_deg()) > 30.0f and copter.flightmode->get_alt_above_ground_cm() < 150){
				 copter.arming.disarm();
				 gcs().send_text(MAV_SEVERITY_CRITICAL,"Topple takeoff");
			}

		}else if(spirit_state == landing){

			if(fabsf(attitude_control->get_att_error_angle_deg()) > 45.0f){
				copter.arming.disarm();
				 gcs().send_text(MAV_SEVERITY_CRITICAL,"Topple landing");
				 return;
			}

			if(fabsf(attitude_control->get_att_error_angle_deg()) > 30.0f and channel_throttle->get_control_in() == 0){
				 copter.arming.disarm();
				 gcs().send_text(MAV_SEVERITY_CRITICAL,"Topple landing");
			}

			if((RangeFinder::RangeFinder_OutOfRangeLow or rangefinder_state.alt_cm_filt.get() <= 75.0f) and fabsf(attitude_control->get_att_error_angle_deg()) > 20.0f and copter.flightmode->get_alt_above_ground_cm() < 400){
				 copter.arming.disarm();
				 gcs().send_text(MAV_SEVERITY_CRITICAL,"Topple landing");
			}

		}else{

			/////Always the case
			if(fabsf(attitude_control->get_att_error_angle_deg()) > 100.0f and copter.flightmode->get_alt_above_ground_cm() < 400){
				 copter.arming.disarm();
				 gcs().send_text(MAV_SEVERITY_CRITICAL,"Topple crash");
			}

		}
}



void Copter::servo_voltage_watcher(){

		 const float servo_voltage = hal.analogin->servorail_voltage();

		 if(servo_voltage < 4.0 ){
			 copter.arming.disarm();
			 AP_Notify::flags.low_servo_voltage = true;
			 gcs().send_text(MAV_SEVERITY_CRITICAL,"LOW SERVO VOLTAGE");
			}
}



//
void Copter::Spirit_Gimbal_Control(){
	/*
	if(!g.herelink_enable){
		return;
	}


	int16_t  gimbal_pan, gimbal_tilt, gimbal_zoom; // , gimbal_focus;

				gimbal_tilt = RC_Channels::rc_channel(CH_2)->get_radio_in();
				gimbal_pan = RC_Channels::rc_channel(CH_1)->get_radio_in();
				gimbal_zoom = RC_Channels::rc_channel(CH_3)->get_radio_in();
				//gimbal_focus = RC_Channels::rc_channel(CH_4)->get_radio_in();


				gimbal_tilt_spd  = -90.0 + ((float)gimbal_tilt - 1100.0)*(0.225);
				gimbal_pan_spd  = -90.0 + ((float)gimbal_pan - 1100.0)*(0.225);


				gimbal_tilt_spd = (constrain_float(gimbal_tilt_spd, -90.0, 90.0));
				gimbal_pan_spd = (constrain_float(gimbal_pan_spd, -90.0, 90.0));


	int16_t gimbal_zoom;

				if(gimbal_zoom < 1400){
					zoom_in = false;
					zoom_out = true;
				}else if(gimbal_zoom > 1600){
					zoom_in = true;
					zoom_out = false;
				}else{
					zoom_in = false;
					zoom_out = false;
				}

*/
			//	hal.console->printf("value %f", gimbal_pan_spd)  ;

}



void Copter::Killswitch(){

	if(!motors->armed()){
		killswitch_counter = 0;
		return;
	}

	if(RC_Channels::rc_channel(CH_8)->get_radio_in() < 1700){
			killswitch_counter = 0;
			return;
	}


	//todo: killswitch activation time dependent on particular condition (ie flying versus toppling)

	bool killswitch_activate = {
			ap.throttle_zero or
			ap.land_complete or
			channel_throttle->get_control_in() == 0 or
			fabsf(attitude_control->get_att_error_angle_deg()) > 45.0f};

	 if(killswitch_activate) {
			 if(killswitch_counter >= 2){
				 copter.arming.disarm();
				 killswitch_counter = 0;
				 gcs().send_text(MAV_SEVERITY_CRITICAL,"User: Disarm");

			 }else{ killswitch_counter++;  }

	 }else{   killswitch_counter = 0;  }

}



void Copter::Detect_Buttons(){


	if(RC_Channels::rc_channel(CH_7)->get_radio_in() > 1800){
					if(!ch7_button_hold){
						if(!ch7_button_pressed){
							ch7_button_pressed = true;
							ch7_timer = millis();
						}else{
							if( (millis() - ch7_timer) > 750 ){
								long_press_flag_ch7 = true;  //these are reset in the 10Hz loop
								function_counter = 0;
								ch7_button_hold = true;
							}
						}
					}
				}else{
					if(ch7_button_pressed){
						if(!ch7_button_hold){  //if hold was active don't do a short_press
							short_press_flag_ch7 = true;//these are reset in the 10Hz loop
							function_counter = 0;
						}
						ch7_button_hold = false;
						ch7_button_pressed = false; //reset button press flag
					}
				}





	if(RC_Channels::rc_channel(CH_9)->get_radio_in() > 1800){
				if(!ch9_button_hold){
					if(!ch9_button_pressed){
						ch9_button_pressed = true;
						ch9_timer = millis();
					}else{
						if( (millis() - ch9_timer) > 750 ){
							long_press_flag_ch9 = true;  //these are reset in the 10Hz loop
							function_counter = 0;
							ch9_button_hold = true;
						}
					}
				}
			}else{
				if(ch9_button_pressed){
					if(!ch9_button_hold){  //if hold was active don't do a short_press
						short_press_flag_ch9 = true;//these are reset in the 10Hz loop
						function_counter = 0;
					}
					ch9_button_hold = false;
					ch9_button_pressed = false; //reset button press flag
				}
			}


		if(RC_Channels::rc_channel(CH_10)->get_radio_in() > 1800){
					if(!ch10_button_hold){
						if(!ch10_button_pressed){
							ch10_button_pressed = true;
							ch10_timer = millis();
						}else{
							if( (millis() - ch10_timer) > 750 ){
								long_press_flag_ch10 = true;  //these are reset in the 10Hz loop
								function_counter = 0;
								ch10_button_hold = true;
							}
						}
					}
				}else{
					if(ch10_button_pressed){
						if(!ch10_button_hold){  //if hold was active don't do a short_press
							short_press_flag_ch10 = true;//these are reset in the 10Hz loop
							function_counter = 0;
						}
						ch10_button_hold = false;
						ch10_button_pressed = false; //reset button press flag
					}
				}


	if(RC_Channels::rc_channel(CH_11)->get_radio_in() > 1800){
				if(!ch11_button_hold){
					if(!ch11_button_pressed){
						ch11_button_pressed = true;
						ch11_timer = millis();
					}else{
						if( (millis() - ch11_timer) > 750 ){
							long_press_flag_ch11 = true;  //these are reset in the 10Hz loop
							function_counter = 0;
							ch11_button_hold = true;
						}
					}
				}
			}else{
				if(ch11_button_pressed){
					if(!ch11_button_hold){  //if hold was active don't do a short_press
						short_press_flag_ch11 = true;//these are reset in the 10Hz loop
						function_counter = 0;
					}
					ch11_button_hold = false;
					ch11_button_pressed = false; //reset button press flag
				}
			}
		}



void Copter::Decode_Buttons(){

	bool camera_type = g.camera_type;

	if(short_press_flag_ch7){

		camera_mount.toggle_record(camera_type);

		//camera_mount.flip_image(true);

	//	camera_mount.take_picture();

		short_press_flag_ch7 = false;





		}

		if(long_press_flag_ch7){


			camera_mount.toggle_camera_state(camera_type);

			long_press_flag_ch7 = false;
		}




	if(short_press_flag_ch9){

		//camera_mount.center_yaw();
		camera_mount.toggle_PIP();
		short_press_flag_ch9 = false;

		}



		if(long_press_flag_ch9){

			//camera_mount.toggle_tracking();
			camera_mount.center_yaw();
			long_press_flag_ch9 = false;


		}

		if(short_press_flag_ch10){

			AP_Notify::events.user_mode_change = 1;




			/*
			if(camera_mount.get_mode() == MAV_MOUNT_MODE_RC_TARGETING){

				camera_mount.set_mode(MAV_MOUNT_MODE_GPS_POINT);

			}else if(camera_mount.get_mode() == MAV_MOUNT_MODE_GPS_POINT){

				camera_mount.set_mode(MAV_MOUNT_MODE_RC_TARGETING);

			}else{

				camera_mount.set_mode(MAV_MOUNT_MODE_RC_TARGETING);
			}

			*/

			camera_mount.set_mode(MAV_MOUNT_MODE_GPS_POINT);


			short_press_flag_ch10 = false;

		}


		if(long_press_flag_ch10){


			camera_mount.set_camera_point_ROI(ahrs.get_yaw());

			 gcs().send_text(MAV_SEVERITY_INFO,"ROI Selected");

			long_press_flag_ch10 = false;

		}


		if(short_press_flag_ch11){
			short_press_flag_ch11 = false;
			if(flight_mode_switch){
				//copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::GCS_COMMAND);
				copter.set_mode((Mode::Number)copter.flight_modes[1].get(), ModeReason::GCS_COMMAND);

				 AP_Notify::events.user_mode_change = 1;
				flight_mode_switch = false;
			}else{
				if(!copter.set_mode((Mode::Number)copter.flight_modes[2].get(), ModeReason::GCS_COMMAND)){
					 AP_Notify::events.user_mode_change_failed = 1;
				}else{
					AP_Notify::events.user_mode_change = 1;
				}

				flight_mode_switch = true;
			}
		}

		if(long_press_flag_ch11){
			long_press_flag_ch11 = false;
			copter.set_mode(Mode::Number::GUIDED, ModeReason::GCS_COMMAND);
		}

}



void Copter::update_rpm_hover(){

	    if (!motors->armed() || ap.land_complete) {
	        return;
	    }

	    if(spirit_state == land or spirit_state == takeoff or spirit_state == spoolup or spirit_state == landing){
	    	return;
	    }


	    Vector3f accel_ef = ahrs.get_accel_ef_blended();
	    accel_ef.z += GRAVITY_MSS;

	    if(fabsf(accel_ef.z) > 1.0f){
	    	return;
	    }


	    if(!is_zero(pos_control->get_desired_velocity().z)){

	    	return;
	    }

	    if(fabsf(inertial_nav.get_velocity_z()) > 50){
	    	return;
	    }

    	if(labs(ahrs.pitch_sensor) > 500 or labs(ahrs.roll_sensor) > 500){
    		return;
    	}

    	if(position_ok()){
    		if(inertial_nav.get_speed_xy() > 400.0){
    			return;
    		}
    	}

	    if(rpm_sensor.healthy(1)){

	    	hover_rpm_filter.apply(rpm_sensor.get_rpm(1));

	    	if(hover_rpm_filter.get() > 2400.0 and hover_rpm_filter.get() < 4100.0){   //no vehicles hover at this low an RPM

	    		motors->set_hover_RPM(hover_rpm_filter.get());
	    	}

	    }else if(rpm_sensor.healthy(0)){

	    	hover_rpm_filter.apply(rpm_sensor.get_rpm(0));

	    	if(hover_rpm_filter.get() > 2400.0 and hover_rpm_filter.get() < 4100.0){    //no vehicles hover at this low an RPM

	    		motors->set_hover_RPM(hover_rpm_filter.get());
	    	}
	    }

}


void Copter::auto_config(){

	float ang_kp;
	float rate_kp;
	float rate_kd ;
	float ang_rate;
	float _rpm_hover;

	if(g.battery_number == 1){

		vehicle_weight = 7.0 + g.payload_weight;
		vehicle_weight = constrain_float(vehicle_weight, 8.0, 13.5);
		attitude_control->get_rate_pitch_pid().imax(0.5);
		attitude_control->get_rate_roll_pid().imax(0.5);

	}else if(g.battery_number == 2){

		vehicle_weight = 10.0 + g.payload_weight;
		vehicle_weight = constrain_float(vehicle_weight, 10.0, 14.0);
		attitude_control->get_rate_pitch_pid().imax(0.6);
		attitude_control->get_rate_roll_pid().imax(0.6);

	}else{

		return;
	}

	if(g.radio_tuning != TUNING_STABILIZE_ROLL_PITCH_KP){
		ang_kp = (-0.23*vehicle_weight)+7.0;
		ang_kp = constrain_float(ang_kp, 4.0, 5.25);

		attitude_control->set_angle_kp(ang_kp);
	}

	if(g.radio_tuning != TUNING_RATE_ROLL_PITCH_KP){

		rate_kp = (0.0225*vehicle_weight)+0.0211;
		rate_kp = constrain_float(rate_kp, 0.18, 0.35);

		if(g.radio_tuning != TUNING_RATE_PITCH_KP){
			attitude_control->get_rate_pitch_pid().kP(rate_kp);
		}

		if(g.radio_tuning != TUNING_RATE_ROLL_KP){
			attitude_control->get_rate_roll_pid().kP(rate_kp);
		}
	}

	if(g.radio_tuning != TUNING_RATE_ROLL_PITCH_KI){

		rate_kp = (0.0225*vehicle_weight)+0.0211;
		rate_kp = constrain_float(rate_kp, 0.18, 0.35);

		if(g.radio_tuning != TUNING_RATE_PITCH_KI){
			attitude_control->get_rate_pitch_pid().kI(rate_kp-0.06);
		}

		if(g.radio_tuning != TUNING_RATE_ROLL_KI){
			attitude_control->get_rate_roll_pid().kI(rate_kp-0.06);
		}
	}


	if(g.radio_tuning != TUNING_RATE_ROLL_PITCH_KD){

		rate_kd = (0.002*vehicle_weight)-0.01;
		rate_kd = constrain_float(rate_kd, 0.005, 0.018);

		if(g.radio_tuning != TUNING_RATE_PITCH_KD){
			attitude_control->get_rate_pitch_pid().kD(rate_kd);
		}

		if(g.radio_tuning != TUNING_RATE_ROLL_KD){
			attitude_control->get_rate_roll_pid().kD(rate_kd);
		}
	}

	ang_rate = (-9000*vehicle_weight)+180000;
	ang_rate = constrain_float(ang_rate, 58500, 108000);

	attitude_control->set_accel_roll_max(ang_rate);
	attitude_control->set_accel_pitch_max(ang_rate);

	_rpm_hover = 156*vehicle_weight + 1553;
	_rpm_hover = constrain_float(_rpm_hover, 2700, 4000);
	g.rpm_hover = _rpm_hover;

	hover_rpm_filter.reset(g.rpm_hover);
	motors->set_hover_RPM(g.rpm_hover);

}


