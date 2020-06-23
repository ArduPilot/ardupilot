#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up


	killswitch_counter = 0;


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
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here

	if(g.herelink_enable){
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
}



#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
	motors->set_dynamic_trim(0.0, 0.0);
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here



	if(!motors->armed()){
		return;
	}

	if(RC_Channels::rc_channel(CH_8)->get_radio_in() < 1500){
		killswitch_counter = 0;
		return;
	}
	///// KILL SWITCH ///////

	if(RC_Channels::rc_channel(CH_8)->get_radio_in() < 1700){
			killswitch_counter = 0;
			return;
	}

	bool killswitch_activate = {
			ap.throttle_zero or
			ap.land_complete or
			fabsf(attitude_control->get_att_error_angle_deg()) > 45.0f};

	 if(killswitch_activate) {

			 if(killswitch_counter >= 3){
				 copter.arming.disarm();
				 killswitch_counter = 0;
				 gcs().send_text(MAV_SEVERITY_CRITICAL,"User: Disarm");

			 }else{ killswitch_counter++;  }

	 }else{   killswitch_counter = 0;  }

	 ///// KILL SWITCH ///////

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


void Copter::Spirit_Gimbal_Control_Auto(){

	if(!g.herelink_enable){
		return;
	}

int16_t  gimbal_pan, gimbal_tilt, gimbal_zoom, gimbal_focus;

		gimbal_tilt = RC_Channels::rc_channel(CH_2)->get_radio_in();
		gimbal_pan = RC_Channels::rc_channel(CH_1)->get_radio_in();
		gimbal_zoom = RC_Channels::rc_channel(CH_3)->get_radio_in();
		gimbal_focus = RC_Channels::rc_channel(CH_4)->get_radio_in();

		gimbal_tilt = ((1500 - gimbal_tilt) + 1500);  //reverse input

		SRV_Channels::set_output_pwm(SRV_Channel::k_gimbal_tilt, gimbal_tilt);
		SRV_Channels::set_output_pwm(SRV_Channel::k_gimbal_pan, gimbal_pan);
		SRV_Channels::set_output_pwm(SRV_Channel::k_gimbal_zoom, gimbal_zoom);
		SRV_Channels::set_output_pwm(SRV_Channel::k_gimbal_focus, gimbal_focus);
}




void Copter::Decode_Buttons(){


	if(RC_Channels::rc_channel(CH_7)->get_radio_in() >= 1700){
		copter.ap.gimbal_control_active = true;
		cam_button_pressed = true;
	}else if(cam_button_pressed){
		if(cam_button_debounce_timer >= 2){
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


