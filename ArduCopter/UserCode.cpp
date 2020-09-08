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


	//Herelink Variables
	ap.gimbal_control_active = false;
	speed_setting = 1;
	function_counter = 0;
	cam_button_debounce_timer = 0;

	ch9_button_pressed = false;
	ch10_button_pressed = false;
	ch11_button_pressed = false;
	ch12_button_pressed = false;

	long_press_flag_ch9 = false;
	long_press_flag_ch10 = false;
	long_press_flag_ch11 = false;
	long_press_flag_ch12 = false;

	short_press_flag_ch9 = false;
	short_press_flag_ch10 = false;
	short_press_flag_ch11 = false;
	short_press_flag_ch12 = false;

	ch9_button_hold = false;
	ch10_button_hold = false;
	ch11_button_hold = false;
	ch12_button_hold = false;

	SRV_Channels::set_output_pwm(SRV_Channel::k_gimbal_rec, 1500);
	SRV_Channels::set_output_pwm(SRV_Channel::k_gimbal_mode, 1500);
	SRV_Channels::set_output_pwm(SRV_Channel::k_gimbal_zoom, 1500);
	SRV_Channels::set_output_pwm(SRV_Channel::k_gimbal_tilt, 1500);
	SRV_Channels::set_output_pwm(SRV_Channel::k_gimbal_pan, 1500);


	//hal.gpio->pinMode(53, 1);
	//hal.gpio->write(53, true);
	// startup spirit state
	spirit_state = disarm;

	//hal.gpio->pinMode(52, 1);
	//hal.gpio->write(52, false);

	spoolup_timer = 0;
	timer_trigger = false;

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


	//zero out dynamic trim for now
	motors->set_dynamic_trim(0.0, 0.0);
	// State Machine

	if(!motors->armed()){
		spirit_state = disarm;

	// the rest considers the vehicle to be armed
	}else if(spirit_state == disarm){

	////Servo Voltage Watcher///////////

	if(ap.land_complete and motors->armed() and !hal.gpio->usb_connected()){
		spirit_state = spoolup;
		spoolup_timer = AP_HAL::millis16();
		gcs().send_text(MAV_SEVERITY_INFO,"SpoolUp");

	 const float servo_voltage = hal.analogin->servorail_voltage();
	}else if(spirit_state == land and (copter.flightmode->is_taking_off() or !ap.land_complete)){

		spirit_state = takeoff;
		gcs().send_text(MAV_SEVERITY_INFO,"takeoff");

	}else if(spirit_state == takeoff and (!copter.flightmode->is_taking_off() or copter.flightmode->has_manual_throttle())){

		spirit_state = hover;
		gcs().send_text(MAV_SEVERITY_INFO,"hover");


	}else if(spirit_state == hover and ap.land_complete){

		spirit_state = land;
		gcs().send_text(MAV_SEVERITY_INFO,"land");

	 if(servo_voltage < 4.0 ){
		 copter.arming.disarm();
		 AP_Notify::flags.low_servo_voltage = true;
		 gcs().send_text(MAV_SEVERITY_CRITICAL,"LOW SERVO VOLTAGE");
		}
	}


	//////////////////////////
	switch(spirit_state){


	case disarm:

		//zero out dynamic trim for now
		motors->set_dynamic_trim(0.0, 0.0);

		break;

	case spoolup:

		servo_voltage_watcher();
		topple_sense();

        attitude_control->reset_rate_controller_I_terms();
        pos_control->relax_alt_hold_controllers(0.0f);

		//if spoolup doesn't advance complete 5 seconds between rotors:' disarm
		if(AP_HAL::millis16() - spoolup_timer > (uint16_t)5000){
			 copter.arming.disarm();
			 gcs().send_text(MAV_SEVERITY_CRITICAL,"A Rotor Failed to Start: Disarm");
		}

		_fwd_rpm = rpm_sensor.get_rpm(0);
		_aft_rpm = rpm_sensor.get_rpm(1);

		if(_fwd_rpm > 1000.0f){

			if(!timer_trigger){
				spoolup_timer =  AP_HAL::millis16();
				timer_trigger = true;
			}

			if(_fwd_rpm >= 1000.0f and _aft_rpm > 1000.0f){  //fully spun up
				motors->spoolup_complete(true);
				spirit_state = land;
				gcs().send_text(MAV_SEVERITY_INFO,"landed");
				timer_trigger = false;

			}else if((AP_HAL::millis16() - spoolup_timer) > (uint16_t)(g.spool_delta*1000)){
				motors->enable_aft_rotor(true);
			}
		}

		break;

	case takeoff:

		topple_sense();

		break;

	case land:

		topple_sense();

		break;

	case hover:

		break;




	//Spirit_Land_Detector();
/*
	if( motors->get_throttle() >= motors->get_throttle_hover() ){
		take_off_complete = true;
	}
*/

	//////   ADVANCE RATIO CALC   ///////////


	if(rpm_sensor.healthy(0)){
	_fwd_rpm = rpm_sensor.get_rpm(0);
	}

	if(rpm_sensor.healthy(1)){
	_aft_rpm = rpm_sensor.get_rpm(1);
	}

/*
if(copter.position_ok()){

		Vector3f _vel = inertial_nav.get_velocity();
		float vel_fw = (_vel.x*ahrs.cos_yaw() + _vel.y*ahrs.sin_yaw())/100;
		float vel_right = (-_vel.x*ahrs.sin_yaw() + _vel.y*ahrs.cos_yaw())/100;


		float trim_ff_pitch, trim_ff_roll;


		if(vel_fw > 2.0f){
			trim_ff_pitch = (vel_fw/HIGH_POINT)*MAX_PARAMETER;
		}
		if(vel_fw > HIGH_POINT){
			trim_ff_pitch =- ((vel_fw - HIGH_POINT) / 20.0f)*0.5*MAX_PARAMETER;
		}else{
			trim_ff_pitch = 0;
		}


		if(rpm_sensor.healthy(1)){

			float rad_per_s_aft = (_aft_rpm/60.0f)*2.0f*3.1415f;

			adv_ratio.x = -2.0f*(ahrs.cos_pitch()) * ( vel_fw / (rad_per_s_aft*0.3048) )  ;
			adv_ratio.y = 2.0f*(ahrs.cos_roll())  * ( vel_right / (rad_per_s_aft*0.3048) ) ;

			adv_ratio.x = constrain_float(adv_ratio.x, -0.30f, 0.30f);
			adv_ratio.y = constrain_float(adv_ratio.y, -0.30f, 0.30f);

		}else{

			//adv_ratio.x = -((ahrs.cos_pitch()) * ( vel_fw / (100.0f) ))  ;
			//adv_ratio.y = ((ahrs.cos_roll())  * ( vel_right / (100.0f) )) ;

			//adv_ratio.x = constrain_float(adv_ratio.x, -0.20f, 0.20f);
			//adv_ratio.y = constrain_float(adv_ratio.y, -0.20f, 0.20f);

			adv_ratio.x = 0.001;
			adv_ratio.y = 0.001;

		}

		if((RC_Channels::rc_channel(CH_7)->get_radio_in() > 1700) and (g.herelink_enable == 0)){
			motors->set_dynamic_trim(adv_ratio.x, adv_ratio.y);
		}else{
			motors->set_dynamic_trim(0.0, 0.0);
		}

	}else{

		adv_ratio.x = 0.0;
		adv_ratio.y = 0.0;
		motors->set_dynamic_trim(0.0, 0.0);
	}
*/

	motors->set_dynamic_trim(0.0, 0.0);

	///////  	DETERMINE HOVER RPM		/////
/*
	 if (fabsf(inertial_nav.get_velocity_z()) < 60  and  labs(ahrs.roll_sensor) < 500 and labs(ahrs.pitch_sensor) < 500) {

		 float ave_rpm = ((_aft_rpm + _fwd_rpm)/2.0f);
		 hover_rpm.apply(ave_rpm);
	 }
*/

	 ////   THRUST CALC   //////

	 float accel = copter.ins.get_accel().z + GRAVITY_MSS;

	 float thrust = -accel*(g.vec_weight/32.2);

	 Log_Write_Vehicle_State(adv_ratio.x, adv_ratio.y, 0, thrust);//hover_rpm.get() 0, thrust);

	//motors->set_dynamic_trim(0.0, 0.0);

}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here

	Killswitch();


	if(!motors->armed()){
		return;
	}


}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here


	if(g.herelink_enable){
		Decode_Buttons();
	}else{
		copter.ap.gimbal_control_active = false;
	}


}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here


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

	if(fabsf(attitude_control->get_att_error_angle_deg()) > 15.0f and ap.land_complete){
		 copter.arming.disarm();
		 gcs().send_text(MAV_SEVERITY_CRITICAL,"Topple: Disarm");

	}else if(fabsf(attitude_control->get_att_error_angle_deg()) > 45.0f){

		 copter.arming.disarm();
		 gcs().send_text(MAV_SEVERITY_CRITICAL,"Topple: Disarm");

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

void Copter::Spirit_Gimbal_Control_Auto(){

	if(!g.herelink_enable){
		return;
	}

int16_t  gimbal_pan, gimbal_tilt, gimbal_zoom, gimbal_focus;

		gimbal_tilt = RC_Channels::rc_channel(CH_2)->get_radio_in();
		gimbal_pan = RC_Channels::rc_channel(CH_1)->get_radio_in();
		gimbal_zoom = RC_Channels::rc_channel(CH_3)->get_radio_in();
		gimbal_focus = RC_Channels::rc_channel(CH_4)->get_radio_in();

		SRV_Channels::set_output_pwm(SRV_Channel::k_gimbal_tilt, gimbal_tilt);
		SRV_Channels::set_output_pwm(SRV_Channel::k_gimbal_pan, gimbal_pan);
		SRV_Channels::set_output_pwm(SRV_Channel::k_gimbal_zoom, gimbal_zoom);
		SRV_Channels::set_output_pwm(SRV_Channel::k_gimbal_focus, gimbal_focus);
}



void Copter::Killswitch(){

	if(!motors->armed()){
		killswitch_counter = 0;
		return;
	}

	///// KILL SWITCH ///////

	if(RC_Channels::rc_channel(CH_8)->get_radio_in() < 1700){
			killswitch_counter = 0;
			return;
	}


	//todo: killswitch activation time dependent on particular condition (ie flying versus toppling)

	bool killswitch_activate = {
			ap.throttle_zero or
			ap.land_complete or
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

	if(RC_Channels::rc_channel(CH_7)->get_radio_in() >= 1700){
		copter.ap.gimbal_control_active = true;
		cam_button_pressed = true;
	}else if(cam_button_pressed){
		if(cam_button_debounce_timer >= 1){
			cam_button_pressed = false;
			cam_button_debounce_timer = 0;
		}else{
			cam_button_debounce_timer++;
		}
	}else{
		copter.ap.gimbal_control_active = false;
	}

	if(short_press_flag_ch9){
			if(function_counter < 2){
				SRV_Channels::set_output_pwm(SRV_Channel::k_gimbal_rec, 1500);
				function_counter++;
			}else if(function_counter < 4){
				SRV_Channels::set_output_pwm(SRV_Channel::k_gimbal_rec, 1100);
				function_counter++;
			}else{
				SRV_Channels::set_output_pwm(SRV_Channel::k_gimbal_rec, 1500);
				short_press_flag_ch9 = false;
				function_counter = 0;
			}
		}



		if(long_press_flag_ch9){
			if(function_counter < 2){
				SRV_Channels::set_output_pwm(SRV_Channel::k_gimbal_rec, 1500);
				function_counter++;
			}else if(function_counter < 4){
				SRV_Channels::set_output_pwm(SRV_Channel::k_gimbal_rec, 1900);
				function_counter++;
			}else{
				SRV_Channels::set_output_pwm(SRV_Channel::k_gimbal_rec, 1500);
				long_press_flag_ch9 = false;
				function_counter = 0;
			}

		}

		if(short_press_flag_ch10){

			short_press_flag_ch10 = false;

			if(speed_setting == 3){
				SRV_Channels::set_output_pwm(SRV_Channel::k_gimbal_mode, 1900);
			}else if(speed_setting == 1){
				SRV_Channels::set_output_pwm(SRV_Channel::k_gimbal_mode, 1100);
			}else{
				SRV_Channels::set_output_pwm(SRV_Channel::k_gimbal_mode, 1500);
			}
			speed_setting++;
			if(speed_setting > 3){ speed_setting = 1; }
		}

		if(long_press_flag_ch10){
			if(function_counter < 4){
				SRV_Channels::set_output_pwm(SRV_Channel::k_gimbal_mode, 1500);
				function_counter++;
			}else if(function_counter < 5){
				SRV_Channels::set_output_pwm(SRV_Channel::k_gimbal_mode, 1900);
				function_counter++;
			}else{
				SRV_Channels::set_output_pwm(SRV_Channel::k_gimbal_mode, 1500);
				function_counter++;
				long_press_flag_ch10 = false;
				function_counter = 0;
			}
		}

		if(short_press_flag_ch11){
			short_press_flag_ch11 = false;
			if(!flight_mode_switch){
				copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::GCS_COMMAND);
				 AP_Notify::events.user_mode_change = 1;
				flight_mode_switch = true;
			}else{
				if(!copter.set_mode((Mode::Number)copter.flight_modes[2].get(), ModeReason::GCS_COMMAND)){
					 AP_Notify::events.user_mode_change_failed = 1;
				}else{
					AP_Notify::events.user_mode_change = 1;
				}

				flight_mode_switch = false;
			}
		}

		if(long_press_flag_ch11){
			long_press_flag_ch11 = false;
			copter.set_mode(Mode::Number::BRAKE, ModeReason::GCS_COMMAND);
		}

}





/*
//void Copter::Spirit_Land_Detector()
{

	//////// LAND DETECTOR ////////

	bool near_home;
	bool touchdown_accel;
	bool hover_speed;
	bool touchdown;
	bool topple;




	if(!motors->armed()){
		return;
	}



	if(!take_off_complete){

		near_home = false;
		touchdown_accel = false;
		hover_speed = false;
		touchdown = false;
		topple = false;

		return;

	}


	bool motors_low = ( motors->get_throttle() <= (motors->get_throttle_hover()/2.0f) );
	bool accel_stationary = (land_accel_ef_filter.get().length() <= 1.00f);
	float altitude = inertial_nav.get_position().z;

	if(position_ok()){

		Location current_position;
		ahrs.get_position(current_position);

		if(current_position.get_distance(ahrs.get_home()) < 10){
			near_home = true;
		}else{
			near_home = false;
		}

		hover_speed = (inertial_nav.get_speed_xy() <= 100.0f);

	}else{
		hover_speed = false;
		near_home = false;
	}


	if( (motors->get_throttle() < motors->get_throttle_hover()) and (ahrs.get_accel_ef_blended().z <= -15.0f) ){

			touchdown_accel_counter = 0;
			touchdown_accel = true;
		}


	if(touchdown_accel_counter > 50){

		touchdown_accel_counter = 0;
		touchdown_accel = false;

	}else{

		touchdown_accel_counter++;
	}


	if(motors_low and accel_stationary){
		touchdown = true;
	}else if(near_home and (altitude <= 3.00f) and touchdown_accel and hover_speed){
		touchdown = true;
	}else if( touchdown_accel and (inertial_nav.get_velocity_z() - pos_control->get_vel_target_z()) > 200.0f  ){
		touchdown = true;
	}

	if(fabsf(attitude_control->get_att_error_angle_deg()) > 25.0f){

		topple = true;

	}else{

		topple = false;
	}


	bool const a1a = near_home;
	bool const a1b = motors_low;
	bool const a1c = accel_stationary;
	bool const a1d = hover_speed;
	bool const a1e = touchdown_accel;
	bool const a1f = touchdown;
	bool const a1g = topple;




	        AP::logger().Write(
	            "LAND",
	            "TimeUS,hom,low,acc,speed,TD_ac,TD,top",
	            "s-------",
	            "F-------",
	            "Qbbbbbbb",
	            AP_HAL::micros64(),
				a1a,
				a1b,
				a1c,
				a1d,
				a1e,
				a1f,
				a1g
	        );
}

*/
