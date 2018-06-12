/*
 *       Example of RC_Channel library.
 *       Based on original sketch by Jason Short. 2010
 */

#include <AP_HAL/AP_HAL.h>
#include <RC_Channel/RC_Channel.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define NUM_CHANNELS 8

static RC_Channels rc_channels;
static RC_Channel *rc;

static void print_pwm(void);
static void print_radio_values();


void setup()
{
    hal.console->printf("ArduPilot RC Channel test\n");

    rc = RC_Channels::rc_channel(CH_1);
    
    print_radio_values();

    // set type of output, symmetrical angles or a number range;
    rc[CH_1].set_angle(4500);
    rc[CH_1].set_default_dead_zone(80);

    rc[CH_2].set_angle(4500);
    rc[CH_2].set_default_dead_zone(80);

    rc[CH_3].set_range(1000);
    rc[CH_3].set_default_dead_zone(20);

    rc[CH_4].set_angle(6000);
    rc[CH_4].set_default_dead_zone(500);

    rc[CH_5].set_range(1000);
    rc[CH_6].set_range(800);

    rc[CH_7].set_range(1000);

    rc[CH_8].set_range(1000);
}

void loop()
{
    RC_Channels::read_input();
    print_pwm();
    
    hal.scheduler->delay(20);
}

static void print_pwm(void)
{
    for (int i=0; i<NUM_CHANNELS; i++) {
	    hal.console->printf("ch%u: %4d ", (unsigned)i+1, (int)rc[i].get_control_in());
    }
    hal.console->printf("\n");
}


static void print_radio_values()
{
    for (int i=0; i<NUM_CHANNELS; i++) {
	     hal.console->printf("CH%u: %u|%u\n",
			  (unsigned)i+1, 
			  (unsigned)rc[i].get_radio_min(), 
			  (unsigned)rc[i].get_radio_max()); 
    }
}


AP_HAL_MAIN();
