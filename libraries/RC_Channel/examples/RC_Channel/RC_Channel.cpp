/*
 *       Example of RC_Channel library.
 *       Based on original sketch by Jason Short. 2010
 */
#define CH_1 0
#define CH_2 1
#define CH_3 2
#define CH_4 3
#define CH_5 4
#define CH_6 5
#define CH_7 6
#define CH_8 7

#include <AP_HAL/AP_HAL.h>
#include <RC_Channel/RC_Channel.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define NUM_CHANNELS 8

static RC_Channel rc_1(CH_1);
static RC_Channel rc_2(CH_2);
static RC_Channel rc_3(CH_3);
static RC_Channel rc_4(CH_4);
static RC_Channel rc_5(CH_5);
static RC_Channel rc_6(CH_6);
static RC_Channel rc_7(CH_7);
static RC_Channel rc_8(CH_8);
static RC_Channel *rc = &rc_1;

static void print_pwm(void);
static void print_radio_values();
static void copy_input_output(void);


void setup()
{
    hal.console->println("ArduPilot RC Channel test");

    print_radio_values();

    // set type of output, symmetrical angles or a number range;
    rc_1.set_angle(4500);
    rc_1.set_default_dead_zone(80);
    rc_1.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);

    rc_2.set_angle(4500);
    rc_2.set_default_dead_zone(80);
    rc_2.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);

    rc_3.set_range(0,1000);
    rc_3.set_default_dead_zone(20);

    rc_4.set_angle(6000);
    rc_4.set_default_dead_zone(500);
    rc_4.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);

    rc_5.set_range(0,1000);
    rc_6.set_range(200,800);

    rc_7.set_range(0,1000);

    rc_8.set_range(0,1000);
    for (int i=0; i<NUM_CHANNELS; i++) {
        rc[i].enable_out();
    }
}

void loop()
{
    RC_Channel::set_pwm_all();
    print_pwm();
    
    copy_input_output();

    hal.scheduler->delay(20);
}

static void print_pwm(void)
{
    for (int i=0; i<NUM_CHANNELS; i++) {
	    hal.console->printf("ch%u: %4d ", (unsigned)i+1, (int)rc[i].control_in);
    }
    hal.console->printf("\n");
}


static void print_radio_values()
{
    for (int i=0; i<NUM_CHANNELS; i++) {
	     hal.console->printf("CH%u: %u|%u\n",
			  (unsigned)i+1, 
			  (unsigned)rc[i].radio_min, 
			  (unsigned)rc[i].radio_max); 
    }
}


/*
  copy scaled input to output
 */
static void copy_input_output(void)
{
    for (int i=0; i<NUM_CHANNELS; i++) {
        rc[i].servo_out = rc[i].control_in;
        rc[i].calc_pwm();
        rc[i].output();
    }
}

AP_HAL_MAIN();
