//First step

#include <AP_Curve.h>       // Curve used to linearlise throttle pwm to thrust
#include <AP_Motors.h>

#define MOTOR_CLASS AP_MotorsQuad
static MOTOR_CLASS motors(g.rc_1, g.rc_2, g.rc_3, g.rc_4);


static void
test_servos_4()
{
	static uint16_t counter = 0;
	static uint8_t start = 0;

	counter += 10;
	if(counter >= 1000) counter = 0;

//	hal.rcout->enable_ch(5);
//	hal.rcout->write(5, counter);

	if(start == 0)
	{
		motors.Init();
		//	motors.set_frame_orientation(g.frame_orientation);
		motors.enable();
	    motors.armed(true);
		start++;
	}


    cliSerial->printf_P(PSTR("h"));

    // set_roll, set_pitch, set_yaw, set_throttle
    motors.set_roll(counter);              // range -4500 ~ 4500
    motors.set_pitch(counter);             // range -4500 ~ 4500
    motors.set_yaw(counter);               // range -4500 ~ 4500
    motors.set_throttle(counter);  	 // range 0 ~ 1000

    motors.output();

    if(counter < 500)
    	hal.gpio->write(13, 0x0);
    else
    	hal.gpio->write(13, 0x1);

}
//end of first step
