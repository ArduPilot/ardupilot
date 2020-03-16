#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up


	killswitch_counter = 0;


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
    // put your 50Hz code here
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

	float cos_lean_angle = (ahrs.cos_pitch()*ahrs.cos_roll());

	bool killswitch_activate = {
			ap.throttle_zero or
			ap.land_complete or
			cos_lean_angle <= 85.0f or
			control_mode == Mode::Number::LAND or
			fabsf(attitude_control->get_att_error_angle_deg()) >= 35.0f

	};



	 if(killswitch_activate) {

			 if(killswitch_counter >= 3){

				 copter.arming.disarm();
				 killswitch_counter = 0;
				 gcs().send_text(MAV_SEVERITY_CRITICAL,"KillSwitch: Disarm");
				// Log_Write_Event(DATA_KILLSWITCH_DISARM);
				 return;

			 }else if(killswitch_counter >= 2){
					 killswitch_counter++;

			 }else{

				 killswitch_counter++;
			 }

	 }else{

		 killswitch_counter = 0;

	 }



}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
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
