/*
 *       Example of RC_Channel library.
 *       Code by Jason Short. 2010
 *       DIYDrones.com
 *
 */
#define CH_1 0
#define CH_2 1
#define CH_3 2
#define CH_4 3
#define CH_5 4
#define CH_6 5
#define CH_7 6
#define CH_8 7

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <RC_Channel.h>

#include <AP_HAL_AVR.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM1
const AP_HAL::HAL& hal = AP_HAL_AVR_APM1;
#endif

RC_Channel rc_1(CH_1);
RC_Channel rc_2(CH_2);
RC_Channel rc_3(CH_3);
RC_Channel rc_4(CH_4);
RC_Channel rc_5(CH_5);
RC_Channel rc_6(CH_6);
RC_Channel rc_7(CH_7);
RC_Channel rc_8(CH_8);
RC_Channel *rc = &rc_1;


void setup()
{
    hal.console->println("ArduPilot RC Channel test");
    hal.scheduler->delay(500);

    // interactive setup
    setup_radio();

    print_radio_values();

    // set type of output, symmetrical angles or a number range;
    // XXX BROKEN
    rc_1.set_angle(4500);
    rc_1.set_dead_zone(80);
    rc_1.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);

    // XXX BROKEN
    rc_2.set_angle(4500);
    rc_2.set_dead_zone(80);
    rc_2.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);

    rc_3.set_range(0,1000);
    rc_3.set_dead_zone(20);

    rc_4.set_angle(6000);
    rc_4.set_dead_zone(500);
    rc_4.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);

    rc_5.set_range(0,1000);
    rc_6.set_range(200,800);

    rc_7.set_range(0,1000);

    rc_8.set_range(0,1000);

    for (int i = 0; i < 30; i++) {
        read_radio();
    }
    rc_1.trim();
    rc_2.trim();
    rc_4.trim();
}

void loop()
{
    hal.scheduler->delay(20);
    debug_rcin();
    read_radio();
    print_pwm();
}

void debug_rcin() {
    uint16_t channels[8];
    hal.rcin->read(channels, 8);
    hal.console->printf_P(
        PSTR("rcin: %u %u %u %u %u %u %u %u\r\n"),
        channels[0],
        channels[1],
        channels[2],
        channels[3],
        channels[4],
        channels[5],
        channels[6],
        channels[7]);
}

void read_radio()
{
    rc_1.set_pwm(hal.rcin->read(CH_1));
    rc_2.set_pwm(hal.rcin->read(CH_2));
    rc_3.set_pwm(hal.rcin->read(CH_3));
    rc_4.set_pwm(hal.rcin->read(CH_4));
    rc_5.set_pwm(hal.rcin->read(CH_5));
    rc_6.set_pwm(hal.rcin->read(CH_6));
    rc_7.set_pwm(hal.rcin->read(CH_7));
    rc_8.set_pwm(hal.rcin->read(CH_8));
}

void print_pwm()
{
    for (int i=0; i<8; i++) {
	    hal.console->printf("ch%u: %4d ", (unsigned)i+1, (int)rc[i].control_in);
    }
    hal.console->printf("\n");
}


void print_radio_values()
{
    for (int i=0; i<8; i++) {
	     hal.console->printf("CH%u: %u|%u\n",
			  (unsigned)i+1, 
			  (unsigned)rc[i].radio_min, 
			  (unsigned)rc[i].radio_max); 
    }
}


void
setup_radio()
{
	hal.console->println("\n\nRadio Setup:");
	uint8_t i;
	
	for(i = 0; i < 100;i++){
		hal.scheduler->delay(20);
		read_radio();
	}
		
	rc_1.radio_min = rc_1.radio_in;
	rc_2.radio_min = rc_2.radio_in;
	rc_3.radio_min = rc_3.radio_in;
	rc_4.radio_min = rc_4.radio_in;
	rc_5.radio_min = rc_5.radio_in;
	rc_6.radio_min = rc_6.radio_in;
	rc_7.radio_min = rc_7.radio_in;
	rc_8.radio_min = rc_8.radio_in;

	rc_1.radio_max = rc_1.radio_in;
	rc_2.radio_max = rc_2.radio_in;
	rc_3.radio_max = rc_3.radio_in;
	rc_4.radio_max = rc_4.radio_in;
	rc_5.radio_max = rc_5.radio_in;
	rc_6.radio_max = rc_6.radio_in;
	rc_7.radio_max = rc_7.radio_in;
	rc_8.radio_max = rc_8.radio_in;

	rc_1.radio_trim = rc_1.radio_in;
	rc_2.radio_trim = rc_2.radio_in;
	rc_4.radio_trim = rc_4.radio_in;
	// 3 is not trimed
	rc_5.radio_trim = 1500;
	rc_6.radio_trim = 1500;
	rc_7.radio_trim = 1500;
	rc_8.radio_trim = 1500;
			
	hal.console->println("\nMove all controls to each extreme. Hit Enter to save:");
	while(1){
		
		hal.scheduler->delay(20);
		// Filters radio input - adjust filters in the radio.pde file
		// ----------------------------------------------------------
		read_radio();

		rc_1.update_min_max();
		rc_2.update_min_max();
		rc_3.update_min_max();
		rc_4.update_min_max();
		rc_5.update_min_max();
		rc_6.update_min_max();
		rc_7.update_min_max();
		rc_8.update_min_max();
		
        if(hal.console->available() > 0) {
            hal.console->println("Radio calibrated, Showing control values:");
            break;
        }
    }
    return;
}

AP_HAL_MAIN();
